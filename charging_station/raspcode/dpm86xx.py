"""
dpm86xx.py  –  Python API for the Joy-IT JT-DPM86XX Programmable Lab Power Supply
====================================================================================
Supports both communication protocols documented in the official protocol sheet:

  • Simple (ASCII) Protocol  – human-readable command/response lines over RS-232/RS-485
  • MODBUS RTU Protocol      – binary framing via RS-485

Author  : generated from JT-DPM86XX_Communication-protocol_2025-07-22.pdf
Requires: pyserial  (`pip install pyserial`)

Key classes
-----------
  DPM86XXConfig    – connection parameters (port, baud rate, address, protocol, …)
  DPM86XXState     – typed dataclass snapshot of every readable register value
  DPM86XX          – low-level serial API; read_all() returns DPM86XXState
  DPM86XXDevice    – high-level device object: manages connection, caches state,
                     probes connectivity, and renders its own status panel

Quick-start – single device
----------------------------
    from dpm86xx import DPM86XX, DPM86XXConfig, BaudRate, Protocol

    cfg = DPM86XXConfig(port="/dev/ttyUSB0", baud_rate=BaudRate.B9600, address=1)
    with DPM86XX(cfg) as psu:
        psu.set_voltage(12.34)
        psu.set_current(1.500)
        psu.set_output(True)
        psu.display_status()        # pretty-print everything to the console

Quick-start – device objects with state caching
------------------------------------------------
    from dpm86xx import DPM86XXDevice, DPM86XXConfig, display_device_list

    devices = [
        DPM86XXDevice(DPM86XXConfig(port="/dev/ttyUSB0", address=1), name="Bench PSU"),
        DPM86XXDevice(DPM86XXConfig(port="/dev/ttyUSB0", address=2), name="HV Supply"),
    ]

    for d in devices:
        d.update_state()        # reads all registers, caches in d.state

    display_device_list(devices) # compact multi-device summary table
    devices[0].display_state()   # full panel for one device
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Callable

import serial


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class Protocol(IntEnum):
    """Communication protocol selection."""
    SIMPLE     = 0   # proprietary ASCII command protocol
    MODBUS_RTU = 1   # standard MODBUS RTU (RS-485)


class BaudRate(IntEnum):
    """Supported baud rates (both protocols)."""
    B2400   = 2400
    B4800   = 4800
    B9600   = 9600
    B19200  = 19200
    B38400  = 38400
    B57600  = 57600
    B115200 = 115200


class SaveSlot(IntEnum):
    """Named save-slot identifiers for save_settings() / recall_settings()."""
    M0 = 0
    M1 = 1
    M2 = 2
    M3 = 3
    M4 = 4
    M5 = 5
    M6 = 6
    M7 = 7
    M8 = 8
    M9 = 9
    UPPER_LIMIT  = 10   # save current V/I as upper-limit preset
    LOWER_LIMIT  = 11   # save current V/I as lower-limit preset
    CANCEL_LIMIT = 12   # cancel voltage/current limits


# ---------------------------------------------------------------------------
# Configuration dataclass
# ---------------------------------------------------------------------------

@dataclass
class DPM86XXConfig:
    """
    All parameters needed to open a serial connection to one DPM86XX unit.

    Attributes
    ----------
    port :
        OS serial port string, e.g. ``"COM3"`` on Windows or
        ``"/dev/ttyUSB0"`` on Linux.
    baud_rate :
        Must match the baud rate configured on the device.
        Factory default is typically 9600.
    address :
        Device slave address (1–99 for Simple protocol, 1–255 for MODBUS).
        Factory default is 1.
    protocol :
        Which wire protocol to use. Must match the device setting ("5-CS"
        function: 0 = Simple, 1 = MODBUS).
    timeout :
        Read timeout in seconds. Increase on slow / long cable runs.
    write_delay :
        Seconds to wait after writing before reading the response.
        Needed because the DPM86XX is a slow embedded system.
    stale_time :
        Seconds after which cached state is considered stale.  Used by
        :meth:`DPM86XXDevice.is_stale`.
    """
    port             : str      = "COM1"
    baud_rate        : BaudRate = BaudRate.B9600
    address          : int      = 1
    protocol         : Protocol = Protocol.SIMPLE
    timeout          : float    = 1.0
    write_delay      : float    = 0.05
    stale_time       : float    = 5.0


# ---------------------------------------------------------------------------
# Device state dataclass
# ---------------------------------------------------------------------------

class RegulationMode(IntEnum):
    """Internal representation of the CCCV register (0x1000)."""
    OFF = 0
    CV  = 1
    CC  = 2
    
    @classmethod
    def fromInt(cls, value: int) -> Optional[RegulationMode]:
        """Return the RegulationMode corresponding to an integer value."""
        for mode in cls:
            if mode.value == value:
                return mode
        return None

@dataclass
class DPM86XXState:
    """
    Typed snapshot of every readable register value on one DPM86XX unit.

    All fields default to ``None``.  They are populated by
    :meth:`DPM86XX.read_all` or :meth:`DPM86XXDevice.update_state`.
    Fields that are unavailable for the active protocol (e.g.
    ``max_voltage`` under MODBUS RTU) remain ``None`` after a successful
    read.

    Attributes
    ----------
    voltage_set :
        Programmed voltage setpoint [V].
    current_set :
        Programmed current (limit) setpoint [A].
    output_enabled :
        ``True`` when the output is switched ON.
    voltage_measured :
        Actual measured output voltage [V].
    current_measured :
        Actual measured output current [A].
    power_measured :
        Derived output power voltage × current [W].  Not a register on
        the device; computed from the two measured values.
    regulation_mode :
        Active regulation mode: ``RegulationMode.OFF``, ``RegulationMode.CV`` or ``RegulationMode.CC``.
    temperature :
        Device internal temperature [°C].
    max_voltage :
        Factory-rated model maximum voltage [V].
        Populated only when using the Simple protocol.
    max_current :
        Factory-rated model maximum current [A].
        Populated only when using the Simple protocol.
    timestamp :
        ``time.time()`` value recorded at the end of the last successful
        :meth:`DPM86XX.read_all` call.  ``None`` means the state has
        never been populated.
    """
    # ── Setpoints ──────────────────────────────────────────────────────────
    voltage_set:      Optional[float] = None   # [V]   programmed setpoint
    current_set:      Optional[float] = None   # [A]   programmed setpoint
    # ── Output switch ──────────────────────────────────────────────────────
    output_enabled:   Optional[bool]  = None   # True = ON
    # ── Measured values ────────────────────────────────────────────────────
    voltage_measured: Optional[float] = None   # [V]   actual output voltage
    current_measured: Optional[float] = None   # [A]   actual output current
    power_measured:   Optional[float] = None   # [W]   derived (V × I)
    # ── Status registers ───────────────────────────────────────────────────
    regulation_mode:  Optional[RegulationMode] = None   # OFF | CV | CC
    temperature:      Optional[int] = None     # [°C]  internal sensor
    # ── Model info (Simple protocol only) ──────────────────────────────────
    max_voltage:      Optional[float] = None   # [V]   hardware maximum
    max_current:      Optional[float] = None   # [A]   hardware maximum
    # ── Metadata ───────────────────────────────────────────────────────────
    timestamp:        Optional[float] = None   # time.time() at last update

    # ── Convenience accessors ──────────────────────────────────────────────

    def is_valid(self) -> bool:
        """Return ``True`` if this state has been populated at least once."""
        return self.timestamp is not None

    def age(self) -> Optional[float]:
        """
        Seconds elapsed since the last successful update.

        Returns ``None`` if the state has never been populated.
        """
        return None if self.timestamp is None else time.time() - self.timestamp

    def is_stale(self, max_age_seconds: float = 5.0) -> bool:
        """
        Return ``True`` when the cached values are older than *max_age_seconds*
        or have never been fetched.
        """
        a = self.age()
        return a is None or a > max_age_seconds


# ---------------------------------------------------------------------------
# Custom exceptions
# ---------------------------------------------------------------------------

class DPM86XXError(Exception):
    """Base class for all DPM86XX errors."""

class DPM86XXTimeoutError(DPM86XXError):
    """Device did not respond within the configured timeout."""

class DPM86XXChecksumError(DPM86XXError):
    """MODBUS RTU CRC check failed on a received frame."""

class DPM86XXValueError(DPM86XXError):
    """A parameter value is outside the allowed range."""

class DPM86XXProtocolError(DPM86XXError):
    """The requested operation is not available on the active protocol."""

class DPM86XXConnectionError(DPM86XXError):
    """No connection to serial port or serial port not established."""

# ---------------------------------------------------------------------------
# Internal MODBUS CRC-16 helpers
# ---------------------------------------------------------------------------

def _crc16_modbus(data: bytes) -> int:
    """Calculate the standard MODBUS RTU CRC-16/IBM checksum."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _crc16_append(payload: bytes) -> bytes:
    """Return *payload* with its 2-byte little-endian CRC appended."""
    return payload + struct.pack("<H", _crc16_modbus(payload))


def _crc16_verify(frame: bytes) -> None:
    """Raise DPM86XXChecksumError if the trailing 2-byte CRC is incorrect."""
    if len(frame) < 4:
        raise DPM86XXChecksumError(f"Frame too short ({len(frame)} bytes) to contain a CRC")
    payload, recv = frame[:-2], frame[-2:]
    calc = struct.pack("<H", _crc16_modbus(payload))
    if calc != recv:
        raise DPM86XXChecksumError(
            f"CRC mismatch – calculated {calc.hex().upper()}, received {recv.hex().upper()}"
        )


# ---------------------------------------------------------------------------
# Main API class
# ---------------------------------------------------------------------------

class DPM86XX:
    """
    Full serial API for the Joy-IT JT-DPM86XX programmable lab power supply.

    All physical quantities use SI base units:
      • Voltage  → float in Volts      (wire integer has radix 2: 1234 = 12.34 V)
      • Current  → float in Amperes    (wire integer has radix 3: 12345 = 12.345 A)
      • Power    → float in Watts      (derived, not transmitted)
      • Temp.    → float in °C         (wire integer has radix 0)

    The class can be used as a context manager::

        with DPM86XX(cfg) as psu:
            psu.set_voltage(5.0)

    Or manually::

        psu = DPM86XX(cfg)
        psu.open()
        psu.set_voltage(5.0)
        psu.close()
    """

    # Radix (decimal places) for each physical quantity on the wire
    _RADIX_V = 2    # 1 unit = 0.01 V
    _RADIX_A = 3    # 1 unit = 0.001 A

    def __init__(self, config: DPM86XXConfig):
        self.config   : DPM86XXConfig     = config
        self._ser     : Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Context manager / lifecycle
    # ------------------------------------------------------------------

    def open(self) -> "DPM86XX":
        """Open the serial port.  Returns *self* for one-liner chaining."""
        c = self.config
        self._ser = serial.Serial(
            port        = c.port,
            baudrate    = int(c.baud_rate),
            bytesize    = serial.EIGHTBITS,
            parity      = serial.PARITY_NONE,
            stopbits    = serial.STOPBITS_ONE,
            timeout     = c.timeout,
        )
        return self

    def close(self) -> None:
        """Close the serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "DPM86XX":
        return self.open()

    def __exit__(self, *_) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        """True when the serial port is currently open."""
        return self._ser is not None and self._ser.is_open

    def __repr__(self) -> str:
        proto = "Simple" if self.config.protocol == Protocol.SIMPLE else "MODBUS RTU"
        state = "open" if self.is_open else "closed"
        return (f"<DPM86XX {state} port={self.config.port!r} "
                f"addr={self.config.address} {proto}>")

    # ------------------------------------------------------------------
    # Low-level Simple-protocol helpers
    # ------------------------------------------------------------------

    def _simple_send(self, func: str, member: int, operand: str = "0") -> str:
        """
        Build, transmit, and return the response for one Simple-protocol transaction.

        Wire format (sent):    ``:{addr:02d}{func}{member:02d}={operand},\r\n``
        Wire format (returned):``:{addr:02d}{func/r}{member:02d}={value},``
        """
        if not self.is_open:
            raise DPM86XXConnectionError("Serial port is not open")
        
        addr = self.config.address
        cmd  = f":{addr:02d}{func}{member:02d}={operand},\r\n"
        self._ser.reset_input_buffer()
        self._ser.write(cmd.encode("ascii"))
        time.sleep(self.config.write_delay)

        raw = self._ser.readline()
        if not raw:
            raise DPM86XXTimeoutError(
                f"No response for Simple command {cmd.strip()!r}"
            )
        return raw.decode("ascii", errors="replace").strip()

    @staticmethod
    def _simple_extract(response: str) -> str:
        """
        Pull the value out of a Simple-protocol response string.

        E.g.  ``:01r10=1234,``  →  ``"1234"``
        """
        sep = "="
        if sep in response:
            _, _, tail = response.partition(sep)
            return tail.rstrip(",").strip()
        raise DPM86XXError(
            f"Cannot extract value from Simple-protocol response: {response!r}"
        )

    # ------------------------------------------------------------------
    # Low-level MODBUS RTU helpers
    # ------------------------------------------------------------------

    def _modbus_exchange(self, frame: bytes) -> bytes:
        """
        Transmit a complete MODBUS RTU frame and return the raw response bytes.
        The caller is responsible for CRC appending and response parsing.
        """
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        time.sleep(self.config.write_delay)
        # Generous read to cover the longest possible response
        response = self._ser.read(256)
        if len(response) < 4:
            raise DPM86XXTimeoutError(
                f"MODBUS: short or empty response ({len(response)} bytes)"
            )
        return response

    def _modbus_read_regs(self, start: int, count: int) -> list[int]:
        """
        MODBUS function 0x03 – Read Holding Registers.

        Parameters
        ----------
        start : first register address (0-based)
        count : number of 16-bit registers to read

        Returns
        -------
        list of *count* unsigned 16-bit integers
        """
        addr    = self.config.address
        payload = struct.pack(">BBHH", addr, 0x03, start, count)
        frame   = _crc16_append(payload)
        raw     = self._modbus_exchange(frame)
        _crc16_verify(raw)

        # Response layout: addr(1) func(1) byte_count(1) data(2*n) crc(2)
        if raw[1] & 0x80:
            raise DPM86XXError(f"MODBUS exception code 0x{raw[2]:02X} on read")
        byte_count = raw[2]
        if len(raw) < 3 + byte_count + 2:
            raise DPM86XXError("MODBUS: response data truncated")
        return [
            struct.unpack(">H", raw[3 + i*2 : 5 + i*2])[0]
            for i in range(count)
        ]

    def _modbus_write_single(self, reg: int, value: int) -> None:
        """
        MODBUS function 0x06 – Write Single Register.

        Parameters
        ----------
        reg   : register address (0-based)
        value : unsigned 16-bit value to write
        """
        addr    = self.config.address
        payload = struct.pack(">BBHH", addr, 0x06, reg, value & 0xFFFF)
        raw     = self._modbus_exchange(_crc16_append(payload))
        _crc16_verify(raw)
        if raw[1] & 0x80:
            raise DPM86XXError(f"MODBUS exception code 0x{raw[2]:02X} on write")

    def _modbus_write_multiple(self, start: int, values: list[int]) -> None:
        """
        MODBUS function 0x10 – Write Multiple Registers.

        Parameters
        ----------
        start  : first register address (0-based)
        values : list of unsigned 16-bit values to write consecutively
        """
        addr       = self.config.address
        count      = len(values)
        byte_count = count * 2
        data       = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
        payload    = (struct.pack(">BBHHB", addr, 0x10, start, count, byte_count) + data)
        raw        = self._modbus_exchange(_crc16_append(payload))
        _crc16_verify(raw)
        if raw[1] & 0x80:
            raise DPM86XXError(f"MODBUS exception code 0x{raw[2]:02X} on multi-write")

    # ------------------------------------------------------------------
    # Protocol-transparent scalar read / write helpers
    # ------------------------------------------------------------------

    def _read_scaled(self, simple_member: int, modbus_reg: int, radix: int) -> float:
        """
        Read one register via whichever protocol is active and scale it.

        The wire value is divided by 10**radix to produce the physical quantity.
        """
        if self.config.protocol == Protocol.SIMPLE:
            resp = self._simple_send("r", simple_member)
            wire = int(self._simple_extract(resp))
        else:
            wire = self._modbus_read_regs(modbus_reg, 1)[0]
        return wire / (10 ** radix)

    def _write_scaled(self, simple_member: int, modbus_reg: int, radix: int, value: float) -> None:
        """
        Scale a physical value and write it via whichever protocol is active.

        The float is multiplied by 10**radix and rounded to the nearest integer.
        """
        wire = round(value * (10 ** radix))
        if self.config.protocol == Protocol.SIMPLE:
            self._simple_send("w", simple_member, str(wire))
        else:
            self._modbus_write_single(modbus_reg, wire)

    def _require_simple(self, method_name: str) -> None:
        """Raise DPM86XXProtocolError if MODBUS RTU is active."""
        if self.config.protocol != Protocol.SIMPLE:
            raise DPM86XXProtocolError(
                f"{method_name}() is only available when using the Simple protocol"
            )

    # ==================================================================
    # ─── PUBLIC READ API ───────────────────────────────────────────────
    # ==================================================================

    def read_max_voltage(self) -> float:
        """
        (Simple r00) Read the factory-rated maximum output voltage of this model.

        Returns
        -------
        float – voltage in Volts (e.g. 60.0 for a DPM-8650)

        Notes
        -----
        Not available via MODBUS RTU.
        """
        self._require_simple("read_max_voltage")
        resp = self._simple_send("r", 0)
        return int(self._simple_extract(resp)) / 100.0

    def read_max_current(self) -> float:
        """
        (Simple r01) Read the factory-rated maximum output current of this model.

        Returns
        -------
        float – current in Amperes (e.g. 5.0 for DPM-8605, 50.0 for DPM-8650)

        Notes
        -----
        Not available via MODBUS RTU.
        """
        self._require_simple("read_max_current")
        resp = self._simple_send("r", 1)
        return int(self._simple_extract(resp)) / 1000.0

    def read_voltage_setting(self) -> float:
        """
        (r10 / reg 0x0000) Read the programmed voltage setpoint.

        Returns
        -------
        float – voltage in Volts, resolution 0.01 V
        """
        return self._read_scaled(10, 0x0000, self._RADIX_V)

    def read_current_setting(self) -> float:
        """
        (r11 / reg 0x0001) Read the programmed current (limit) setpoint.

        Returns
        -------
        float – current in Amperes, resolution 0.001 A
        """
        return self._read_scaled(11, 0x0001, self._RADIX_A)

    def read_output_enabled(self) -> bool:
        """
        (r12 / reg 0x0002) Return True if the output is currently switched ON.
        """
        if self.config.protocol == Protocol.SIMPLE:
            resp = self._simple_send("r", 12)
            return bool(int(self._simple_extract(resp)))
        else:
            return bool(self._modbus_read_regs(0x0002, 1)[0])

    def read_output_voltage(self) -> float:
        """
        (r30 / reg 0x1001) Read the measured (actual) output voltage.

        Returns
        -------
        float – voltage in Volts, resolution 0.01 V
        """
        return self._read_scaled(30, 0x1001, self._RADIX_V)

    def read_output_current(self) -> float:
        """
        (r31 / reg 0x1002) Read the measured (actual) output current.

        Returns
        -------
        float – current in Amperes, resolution 0.001 A
        """
        return self._read_scaled(31, 0x1002, self._RADIX_A)

    def read_output_power(self) -> float:
        """
        Convenience – return the product of measured voltage and current.

        Returns
        -------
        float – power in Watts (derived, not a register on the device)
        """
        return round(self.read_output_voltage() * self.read_output_current(), 4)

    def read_regulation_mode(self) -> RegulationMode:
        """
        (r32 / reg 0x1000) Return the active regulation mode.

        Returns
        -------
        ``RegulationMode.CV``  – constant-voltage mode
        ``RegulationMode.CC``  – constant-current mode
        ``RegulationMode.OFF`` – output is off / no regulation (MODBUS only)
        """
        if self.config.protocol == Protocol.SIMPLE:
            resp = self._simple_send("r", 32)
            raw  = int(self._simple_extract(resp))
            # Simple protocol: 0 = CV, 1 = CC
            return RegulationMode.CC if raw == 1 else RegulationMode.CV
        else:
            # MODBUS CCCV register 0x1000: 0 = no output, 1 = CV, 2 = CC
            raw = self._modbus_read_regs(0x1000, 1)[0]
            
            resp = RegulationMode.fromInt(raw)
            if resp is not None:
                return resp
            
            raise DPM86XXError(f"Unknown regulation mode value {raw} in register 0x1000")

    def read_temperature(self) -> int:
        """
        (r33 / reg 0x1003) Read the device's internal temperature sensor.

        Returns
        -------
        int – temperature in °C, resolution 1 °C
        """
        if self.config.protocol == Protocol.SIMPLE:
            # Protocol example shows empty operand: :01r33=,
            resp = self._simple_send("r", 33, "")
            return int(self._simple_extract(resp))
        else:
            return int(self._modbus_read_regs(0x1003, 1)[0])

    # ==================================================================
    # ─── PUBLIC WRITE API ──────────────────────────────────────────────
    # ==================================================================

    def set_voltage(self, volts: float) -> None:
        """
        (w10 / reg 0x0000) Set the voltage setpoint.

        Parameters
        ----------
        volts : target voltage in Volts, resolution 0.01 V
        """
        if volts < 0:
            raise DPM86XXValueError(f"Voltage must be ≥ 0, got {volts}")
        self._write_scaled(10, 0x0000, self._RADIX_V, volts)

    def set_current(self, amps: float) -> None:
        """
        (w11 / reg 0x0001) Set the current limit setpoint.

        Parameters
        ----------
        amps : target current in Amperes, resolution 0.001 A
        """
        if amps < 0:
            raise DPM86XXValueError(f"Current must be ≥ 0, got {amps}")
        self._write_scaled(11, 0x0001, self._RADIX_A, amps)

    def set_output(self, enabled: bool) -> None:
        """
        (w12 / reg 0x0002) Switch the output ON or OFF.

        Parameters
        ----------
        enabled : True to turn the output ON, False to turn it OFF
        """
        if self.config.protocol == Protocol.SIMPLE:
            self._simple_send("w", 12, "1" if enabled else "0")
        else:
            self._modbus_write_single(0x0002, 1 if enabled else 0)

    def set_voltage_and_current(self, volts: float, amps: float) -> None:
        """
        (w20 / 0x10 multi-write) Set voltage and current in a single transaction.

        More efficient than calling :meth:`set_voltage` and :meth:`set_current`
        separately because it requires only one serial round-trip.

        Parameters
        ----------
        volts : voltage setpoint in Volts
        amps  : current setpoint in Amperes
        """
        if volts < 0:
            raise DPM86XXValueError(f"Voltage must be ≥ 0, got {volts}")
        if amps < 0:
            raise DPM86XXValueError(f"Current must be ≥ 0, got {amps}")

        v_wire = round(volts * (10 ** self._RADIX_V))
        i_wire = round(amps  * (10 ** self._RADIX_A))

        if self.config.protocol == Protocol.SIMPLE:
            # Format:  :01w20=<vvvv>,<iiiii>,\r\n
            self._simple_send("w", 20, f"{v_wire},{i_wire}")
        else:
            # Write registers 0x0000 (voltage) and 0x0001 (current) atomically
            self._modbus_write_multiple(0x0000, [v_wire, i_wire])

    def set_power_on_default(self, output_on: bool) -> None:
        """
        (Simple w13) Configure whether the output is ON or OFF after power-up.

        Parameters
        ----------
        output_on : True → output defaults ON; False → output defaults OFF

        Notes
        -----
        Simple protocol only.  The confirmation code ``1313`` is appended
        automatically.
        """
        self._require_simple("set_power_on_default")
        val = "1" if output_on else "0"
        self._simple_send("w", 13, f"{val},1313")

    def set_fast_discharge(self, enabled: bool) -> None:
        """
        (Simple w14) Enable or disable the fast-discharge switch.

        Parameters
        ----------
        enabled : True to enable fast discharge, False to disable

        Notes
        -----
        Simple protocol only.  The confirmation code ``1414`` is appended
        automatically.
        """
        self._require_simple("set_fast_discharge")
        val = "1" if enabled else "0"
        self._simple_send("w", 14, f"{val},1414")

    def set_communication_protocol(self, protocol: Protocol) -> None:
        """
        (Simple w15) Switch the device between Simple and MODBUS RTU protocol.

        .. warning::
           After calling this the device will stop responding on the current
           protocol.  You must re-open the connection with the matching
           ``DPM86XXConfig.protocol`` setting.

        Parameters
        ----------
        protocol : :class:`Protocol.SIMPLE` or :class:`Protocol.MODBUS_RTU`

        Notes
        -----
        Only callable while the Simple protocol is active.  The confirmation
        code ``1515`` is appended automatically.
        """
        self._require_simple("set_communication_protocol")
        self._simple_send("w", 15, f"{int(protocol)},1515")

    def set_baud_rate(self, baud: BaudRate) -> None:
        """
        (Simple w16) Persist a new baud rate in the device's non-volatile memory.

        .. warning::
           The new baud rate takes effect immediately.  Reopen the serial port
           with the matching baud rate before sending further commands.

        Parameters
        ----------
        baud : one of the :class:`BaudRate` enum members

        Notes
        -----
        Simple protocol only.  The wire encoding divides the baud rate by 100
        and zero-pads to four digits (e.g. 9600 → "0096").  The confirmation
        code ``1616`` is appended automatically.
        """
        self._require_simple("set_baud_rate")
        # Protocol encodes as baud/100, zero-padded to 4 digits
        code = int(baud) // 100
        self._simple_send("w", 16, f"{code:04d},1616")

    def set_slave_address(self, new_address: int) -> None:
        """
        (Simple w17) Change the device's slave address.

        .. warning::
           All subsequent commands must use the new address.  Update
           ``DPM86XXConfig.address`` accordingly before the next call.

        Parameters
        ----------
        new_address : integer in [1, 99]

        Notes
        -----
        Simple protocol only.  The confirmation code ``1717`` is appended
        automatically.
        """
        self._require_simple("set_slave_address")
        if not 1 <= new_address <= 99:
            raise DPM86XXValueError(f"Slave address must be 1–99, got {new_address}")
        self._simple_send("w", 17, f"{new_address:02d},1717")

    def save_settings(self, slot: "int | SaveSlot") -> None:
        """
        (Simple w21) Save the current V/I setpoints to non-volatile memory.

        Parameters
        ----------
        slot :
            Memory slot to write.  Use :class:`SaveSlot` members for
            clarity:

            * ``M0``–``M9``      → slots 0–9
            * ``UPPER_LIMIT``    → save as upper V/I limit preset  (slot 10)
            * ``LOWER_LIMIT``    → save as lower V/I limit preset  (slot 11)
            * ``CANCEL_LIMIT``   → cancel all V/I limits            (slot 12)

        Notes
        -----
        Simple protocol only.
        """
        self._require_simple("save_settings")
        s = int(slot)
        if not 0 <= s <= 12:
            raise DPM86XXValueError(f"Save slot must be 0–12, got {s}")
        self._simple_send("w", 21, str(s))

    def recall_settings(self, slot: "int | SaveSlot") -> None:
        """
        (Simple w22) Recall previously saved V/I setpoints from non-volatile memory.

        Parameters
        ----------
        slot : integer 0–9 or a :class:`SaveSlot` member ``M0``–``M9``

        Notes
        -----
        Simple protocol only.
        """
        self._require_simple("recall_settings")
        s = int(slot)
        if not 0 <= s <= 9:
            raise DPM86XXValueError(f"Recall slot must be 0–9, got {s}")
        self._simple_send("w", 22, str(s))

    # ==================================================================
    # ─── READ-ALL + CONSOLE VISUALISATION ─────────────────────────────
    # ==================================================================

    def read_all(self) -> DPM86XXState:
        """
        Read every available parameter in one pass and return a :class:`DPM86XXState`.

        When the MODBUS RTU protocol is active, registers are batched into two
        multi-register reads to minimize round-trips:

        * Batch 1 – regs 0x0000–0x0002: voltage setpoint, current setpoint,
          output switch state.
        * Batch 2 – regs 0x1000–0x1003: regulation mode, measured voltage,
          measured current, temperature.

        Returns
        -------
        DPM86XXState
            Fully populated state snapshot.  :attr:`DPM86XXState.timestamp`
            is set to the current wall-clock time.
        """
        s = DPM86XXState()

        if self.config.protocol == Protocol.MODBUS_RTU:
            # ── Batch 1: setpoints + output switch (regs 0x0000–0x0002)
            regs_set = self._modbus_read_regs(0x0000, 3)
            s.voltage_set    = regs_set[0] / 100.0
            s.current_set    = regs_set[1] / 1000.0
            s.output_enabled = bool(regs_set[2])

            # ── Batch 2: measured values + status (regs 0x1000–0x1003)
            regs_meas = self._modbus_read_regs(0x1000, 4)
            raw_mode  = regs_meas[0]            # CCCV: 0 = no output, 1 = CV, 2 = CC
            s.voltage_measured = regs_meas[1] / 100.0
            s.current_measured = regs_meas[2] / 1000.0
            s.temperature      = float(regs_meas[3])
            s.regulation_mode  = RegulationMode.fromInt(raw_mode)
            # max_voltage / max_current not available via MODBUS; remain None

        else:   # Simple protocol – one request per register
            try:
                s.max_voltage = self.read_max_voltage()
            except DPM86XXError:
                pass
            try:
                s.max_current = self.read_max_current()
            except DPM86XXError:
                pass

            s.voltage_set      = self.read_voltage_setting()
            s.current_set      = self.read_current_setting()
            s.output_enabled   = self.read_output_enabled()
            s.voltage_measured = self.read_output_voltage()
            s.current_measured = self.read_output_current()
            s.regulation_mode  = self.read_regulation_mode()
            s.temperature      = self.read_temperature()

        if s.voltage_measured is not None and s.current_measured is not None:
            s.power_measured = round(s.voltage_measured * s.current_measured, 4)

        s.timestamp = time.time()
        return s

    def display_status(self) -> None:
        """
        Read all available parameters and render a formatted status panel on stdout.

        Calls :meth:`read_all` internally.  ANSI colour codes are used when the
        terminal appears to support them; plain ASCII box-drawing is used otherwise.
        """
        _render_status_panel(self.read_all(), self.config)


# ---------------------------------------------------------------------------
# Console rendering (standalone so it can be called with pre-fetched data)
# ---------------------------------------------------------------------------

def _ansi_supported() -> bool:
    """Return True when stdout is a tty that supports ANSI escape codes."""
    if os.name == "nt":
        return os.environ.get("TERM") is not None or "WT_SESSION" in os.environ
    return hasattr(os, "get_terminal_size") and os.isatty(1)

global USE_ANSI_COLORING
USE_ANSI_COLORING = _ansi_supported()

# ANSI colour shortcuts
_RST  = "\033[0m"
_BOLD = "\033[1m"
_STRK = "\033[9m"
_GRN  = "\033[32m"
_RED  = "\033[31m"
_YEL  = "\033[33m"
_CYN  = "\033[36m"
_DIM  = "\033[2m"

def green(t):  return color(t, _GRN)
def red(t):    return color(t, _RED)
def yel(t):    return color(t, _YEL)
def cyn(t):    return color(t, _CYN)
def dim(t):    return color(t, _DIM)
def bold(t):   return color(t, _BOLD)
def strike(t): return color(t, _STRK)

def color(text: str, code: str) -> str:
    global USE_ANSI_COLORING
    return f"{code}{text}{_RST}" if USE_ANSI_COLORING else text


def _fv(val: Optional[float], decimals: int = 2, unit: str = "V") -> str:
    """Format an optional float with unit; returns ``'---'`` for ``None``."""
    return f"{val:.{decimals}f} {unit}" if val is not None else "---"


_color_mode_lookup: dict[Optional[RegulationMode], tuple[str, Callable[[str], str]]] = {
    None: ("---", dim),
    RegulationMode.CV: ("CV", green),
    RegulationMode.CC: ("CC", yel),
    RegulationMode.OFF: ("OFF", dim),
}
_color_output_lookup: dict[Optional[bool], tuple[str, Callable[[str], str]]] = {
    True:  ("ON", green),
    False: ("OFF", red),
    None:  ("---", dim),
}


# ---------------------------------------------------------------------------
# DPM86XXDevice – high-level device object
# ---------------------------------------------------------------------------

class DPM86XXDevice:
    """
    High-level object representing one physical JT-DPM86XX power supply.

    Unlike :class:`DPM86XX` (which is a raw serial API), ``DPM86XXDevice``
    adds:

    * A human-readable **name** label.
    * A persistent **state cache** (:attr:`state`) of type
      :class:`DPM86XXState` that survives port open/close cycles.
    * :meth:`update_state` – reads all registers and updates the cache.
    * :meth:`is_connected` – probes the device without side-effects.
    * :meth:`display_state` – renders the *cached* state panel; no I/O.

    The class is a context manager and delegates all write/read primitives
    to the wrapped :class:`DPM86XX` instance via :attr:`api`.

    Typical usage::

        device = DPM86XXDevice(cfg, name="Bench PSU")
        with device:
            device.update_state()
            device.display_state()

    Or without a context manager (auto open/close per operation)::

        device = DPM86XXDevice(cfg, name="Bench PSU")
        state  = device.update_state()   # opens, reads, closes automatically
        device.display_state()           # uses cached state – no I/O
    """
    config: DPM86XXConfig
    name  : str
    state : DPM86XXState
    api   : DPM86XX
    errors: list[DPM86XXError]

    def __init__(self, config: DPM86XXConfig, name: str = ""):
        """
        Parameters
        ----------
        config :
            Connection configuration.  :attr:`DPM86XXConfig.address`
            is the primary identifier for this device on the bus.
        name :
            Human-readable label (e.g. ``"Bench PSU"``).  Defaults to
            ``"PSU@addr<N>"`` when omitted.
        """
        self.config = config
        self.name   = name.strip() or f"PSU@addr{config.address:02d}"
        self.state  = DPM86XXState()
        self.api    = DPM86XX(config)
        self.errors = []

    # ── Identity ───────────────────────────────────────────────────────────

    @property
    def address(self) -> int:
        """Slave address as configured in :attr:`config`."""
        return self.config.address

    def __repr__(self) -> str:
        conn = "open" if self.api.is_open else "closed"
        return (
            f"<DPM86XXDevice name={self.name!r} addr={self.address} "
            f"port={self.config.port!r} {conn}>"
        )

    # ── Connection management ──────────────────────────────────────────────

    def open(self) -> "DPM86XXDevice":
        """
        Open the serial port.

        Returns *self* for one-liner chaining::

            device.open().update_state()
        """
        self.api.open()
        return self

    def close(self) -> None:
        """Close the serial port."""
        self.api.close()

    def __enter__(self) -> "DPM86XXDevice":
        return self.open()

    def __exit__(self, *_) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        """``True`` when the underlying serial port is currently open."""
        return self.api.is_open

    # ── Connection probe ───────────────────────────────────────────────────

    def is_connected(self) -> bool:
        """
        Probe the device and return ``True`` if it responds.

        A lightweight read of the voltage setpoint register is used as the
        probe (available on both protocols).

        If the serial port is **already open**, the check runs on the existing
        connection.  Otherwise the port is opened and closed automatically for
        the duration of the probe, leaving the connection state unchanged.

        Returns
        -------
        bool
            ``True``  – device responded within :attr:`DPM86XXConfig.timeout`.
            ``False`` – no response, serial error, or port could not be opened.
        """
        was_open = self.api.is_open
        try:
            if not was_open:
                self.api.open()
            self.api.read_voltage_setting()
            return True
        except (DPM86XXError, serial.SerialException, OSError):
            return False
        finally:
            if not was_open:
                try:
                    self.api.close()
                except Exception:
                    pass

    # ── State management ───────────────────────────────────────────────────

    def update_state(self) -> DPM86XXState:
        """
        Read all available registers and update :attr:`state`.

        If the serial port is already open the existing connection is used.
        Otherwise the port is opened before reading and closed afterwards,
        leaving the connection state unchanged.

        Returns
        -------
        DPM86XXState
            The freshly populated state (also stored in :attr:`state`).

        Raises
        ------
        DPM86XXError
            If communication fails (e.g. no response, CRC error).
        serial.SerialException
            If the serial port cannot be opened.
        """
        was_open = self.api.is_open
        try:
            if not was_open:
                self.api.open()
            self.state = self.api.read_all()
            return self.state
        finally:
            if not was_open:
                self.api.close()

    # ── Error tracking ─────────────────────────────────────────────────────
    
    def has_errors(self) -> bool:
        """Return True if any errors have been recorded in :attr:`errors`."""
        return len(self.errors) > 0
    
    def clear_errors(self) -> None:
        """Clear the :attr:`errors` list."""
        self.errors.clear()

    # ── Console output ──────────────────────────────────────────────────────

    def display_state(self) -> None:
        """
        Print a full-detail status panel of the **cached** :attr:`state`.

        This method does **not** communicate with the device.  Call
        :meth:`update_state` first if you need fresh values.
        """
        _render_status_panel(self.state, self.config, device_name=self.name)


# ---------------------------------------------------------------------------
# display_device_list – compact multi-device summary table
# ---------------------------------------------------------------------------

def _render_status_panel(
    state: DPM86XXState,
    cfg: DPM86XXConfig,
    device_name: str = "",
    W: int = 52
) -> None:
    """
    Print a formatted full-detail status panel for one device.

    Parameters
    ----------
    state :
        State snapshot to render.  ``None`` fields are shown as ``---``.
    cfg :
        Connection configuration (used for header information only).
    device_name :
        Optional human-readable device label shown in the title bar.
    W :
        Inner width of the box (excluding the vertical borders).  Default 52.
    """
    proto = "Simple" if cfg.protocol == Protocol.SIMPLE else "MODBUS RTU"
    LINE  = "─" * W
    SEP   = f"├{LINE}┤"

    def row(label: str, value_str: str, col_fn=None) -> str:
        """One labelled data row; *value_str* is measured for alignment."""
        label_col = 25
        value_col = W - label_col - 2
        colored   = col_fn(value_str) if col_fn else value_str
        padding   = value_col - len(value_str)
        return f"│ {label:<{label_col}s}{' ' * max(0, padding)}{colored} │"

    # ── Output state & regulation mode decorations ────────────────────────
    out_str, out_fn = _color_output_lookup.get(state.output_enabled, ("---", dim))
    mode_str, mode_fn = _color_mode_lookup.get(state.regulation_mode, ("---", dim))

    # ── Title bar ─────────────────────────────────────────────────────────
    name_part = f"{device_name}  ·  " if device_name else ""
    title     = f"  {name_part}JT-DPM86XX  ·  {proto}  ·  Addr {cfg.address}"
    print()
    print(f"╭{LINE}╮")
    print(f"│{_BOLD if USE_ANSI_COLORING else ''}{title:^{W}}{_RST if USE_ANSI_COLORING else ''}│")
    print(SEP)

    # ── Last-update timestamp ─────────────────────────────────────────────
    if state.timestamp is not None:
        import datetime
        ts_str = datetime.datetime.fromtimestamp(state.timestamp).strftime("%H:%M:%S")
        age    = state.age()
        age_str = f"{age:.1f} s ago" if age is not None else ""
        print(row("State captured at", f"{ts_str}  ({age_str})", dim))
        print(SEP)

    # ── Model limits (Simple protocol only) ───────────────────────────────
    if state.max_voltage is not None or state.max_current is not None:
        if state.max_voltage is not None:
            print(row("Model maximum voltage", _fv(state.max_voltage, 2, "V"), cyn))
        if state.max_current is not None:
            print(row("Model maximum current", _fv(state.max_current, 3, "A"), cyn))
        print(SEP)

    # ── Setpoints ─────────────────────────────────────────────────────────
    print(row("Voltage setpoint",     _fv(state.voltage_set,  2, "V")))
    print(row("Current setpoint",     _fv(state.current_set,  3, "A")))
    print(SEP)

    # ── Measured values ───────────────────────────────────────────────────
    print(row("Measured voltage",     _fv(state.voltage_measured, 2, "V")))
    print(row("Measured current",     _fv(state.current_measured, 3, "A")))
    print(row("Measured power (V×I)", _fv(state.power_measured,   4, "W")))
    print(SEP)

    # ── Status ────────────────────────────────────────────────────────────
    print(row("Output state",         out_str,                      out_fn))
    print(row("Regulation mode",      mode_str,                    mode_fn))
    print(row("Device temperature",   _fv(state.temperature, 0, "°C")))
    print(f"╰{LINE}╯")
    print()


def display_device_list(devices: List[DPM86XXDevice]) -> None:
    """
    Print a compact summary table of multiple :class:`DPM86XXDevice` objects.

    Uses the **cached** :attr:`DPM86XXDevice.state` of each device — no serial
    communication is performed.  Call :meth:`DPM86XXDevice.update_state` on
    each device beforehand to ensure fresh values are shown.

    The *Status* column indicates the freshness of each device's cached state:

    * ``● LIVE   `` – data is less than th devices stale time old.
    * ``○ STALE  `` – data is older than the devices stale time but was fetched at least once.
    * ``✗ NO DATA`` – :meth:`update_state` has never been called for this device.
    * ``✗ ERROR  `` – iff errors have accumulated.
    * ``✗ DISCONN`` – the device was unreachable when :meth:`update_state` was called.

    Parameters
    ----------
    devices :
        List of :class:`DPM86XXDevice` instances to display.  An empty list
        prints a short notice and returns immediately.
    """
    if not devices:
        print("(device list is empty)")
        return

    # ── Column inner-widths (content only, no padding) ───────────────────
    # Each cell is rendered as  │<sp>content<sp>  for visual clarity.
    C_ADDR   = 2    # " 1" … "99"
    C_NAME   = 18   # truncated device name
    C_STAT   = 9    # "● LIVE  " / "○ STALE " / "✗ NO DATA" / "✗ ERROR  " / "✗ DISCONN"
    C_VSET   = 7    # "12.34 V"
    C_ISET   = 8    # " 1.500 A"
    C_VOUT   = 7    # "12.31 V"
    C_IOUT   = 8    # " 0.748 A"
    C_MODE   = 4    # "CV" / "CC" / "--"
    C_OUT    = 3    # " ON" / "OFF"
    C_TEMP   = 4    # "35°C" / " ---"

    # Build widths list and header labels in lock-step
    _COL_W = [C_ADDR, C_NAME, C_STAT, C_VSET, C_ISET, C_VOUT, C_IOUT, C_MODE, C_OUT, C_TEMP]
    _HDR   = ["Adr",  "Name",  "Status",  "V_set", " I_set", "V_out", " I_out", "Mode", "Out", "Temp"]

    # Total table width: 2 outside borders + columns (incl. internal spaces) + column spacers
    _TW = 1 + sum(1 + w + 1 for w in _COL_W) + (len(_COL_W) - 1) + 1
    
    # ── Cell formatter helpers ────────────────────────────────────────────
    def _pad(s: str, w: int) -> str:
        """Left-pad *s* to width *w* using raw len (no ANSI in *s*)."""
        return s[:w].ljust(w)

    def _rpad(s: str, w: int) -> str:
        return s[:w].rjust(w)

    def _fmt_v(v: Optional[float]) -> str:
        return f"{v:2.2f} V" if v is not None else "--.-- V"

    def _fmt_a(v: Optional[float]) -> str:
        return f"{v:2.3f} A" if v is not None else "--.--- A"

    def _fmt_mode(v: Optional[str]) -> str:
        if v is None:  return "--"
        return f"{v:<2}"[:3]

    def _fmt_out(v: Optional[bool]) -> str:
        if v is None:   return "---"
        return " ON" if v else "OFF"

    def _fmt_temp(v: Optional[int]) -> str:
        return f"{v:2d}°C" if v is not None else "--°C"

    def _status_cell(dev: DPM86XXDevice) -> tuple[str, object]:
        """Return (raw_text, colour_fn) for the status column."""
        s = dev.state
        if not dev.is_connected():
            return ("✗ DISCONN", red)
        if dev.has_errors():
            return ("✗ ERROR  ", red)
        if not s.is_valid():
            return ("✗ NO DATA", dim)
        if s.is_stale(dev.config.stale_time):
            return ("○ STALE ", yel)
        return ("● LIVE  ", green)

    # ── Separator row builder ─────────────────────────────────────────────
    def _hsep(left="├", mid="┼", right="┤") -> str:
        parts = [f"{'─' * (w + 2)}" for w in _COL_W]
        return left + mid.join(parts) + right

    # ── Single data row builder ───────────────────────────────────────────
    def _data_row(cells_raw: list, colour_fns: list) -> str:
        """
        Build one │-delimited row.  *cells_raw* are plain strings (used for
        width measurement); *colour_fns* are applied for display.
        """
        parts = []
        for raw, fn, w in zip(cells_raw, colour_fns, _COL_W):
            displayed = fn(raw) if fn else raw
            # Padding is computed from raw length, applied after coloring
            pad = " " * max(0, w - len(raw))
            parts.append(f" {displayed}{pad} ")
        return "│" + "│".join(parts) + "│"

    # ── Print the table ───────────────────────────────────────────────────
    proto_label = "Simple" if devices[0].config.protocol == Protocol.SIMPLE else "MODBUS RTU"
    title = f" JT-DPM86XX Device List  ·  {len(devices)} device{'s' if len(devices) != 1 else ''}  ·  {proto_label} "

    print()
    print("╭" + "─" * (_TW - 2) + "╮")
    print("│" + bold(title.center(_TW - 2)) + "│")
    print(_hsep("├", "┬", "┤"))

    # Header row
    hdr_cells = [_rpad(h, w) for h, w in zip(_HDR, _COL_W)]
    hdr_fns   = [bold] * len(_HDR)
    print(_data_row(hdr_cells, hdr_fns))
    print(_hsep("├", "┼", "┤"))

    mode_fn_lookup = {
        None: dim,
        RegulationMode.CV: green,
        RegulationMode.CC: yel,
        RegulationMode.OFF: dim,
    }
    
    # Data rows
    for dev in devices:
        s = dev.state
        stat_raw, stat_fn = _status_cell(dev)

        mode_str = s.regulation_mode.name if s.regulation_mode is not None else None
        mode_fn = mode_fn_lookup.get(s.regulation_mode, dim)
        
        cells_raw = [
            _rpad(str(dev.address), C_ADDR),
            _pad(dev.name,          C_NAME),
            _pad(stat_raw,          C_STAT),
            _fmt_v(s.voltage_set)        .rjust(C_VSET),
            _fmt_a(s.current_set)        .rjust(C_ISET),
            _fmt_v(s.voltage_measured)   .rjust(C_VOUT),
            _fmt_a(s.current_measured)   .rjust(C_IOUT),
            _fmt_mode(mode_str)          .center(C_MODE),
            _fmt_out(s.output_enabled)   .rjust(C_OUT),
            _fmt_temp(s.temperature)     .rjust(C_TEMP),
        ]

        # Output-ON/OFF gets its own colour; regulation mode too
        out_fn  = (green if s.output_enabled is True
                   else (red if s.output_enabled is False else dim))

        colour_fns = [
            None,      # addr – no colour
            None,      # name
            stat_fn,   # status
            None,      # V_set
            None,      # I_set
            None,      # V_out
            None,      # I_out
            mode_fn,   # mode
            out_fn,    # out
            None,      # temp
        ]
        
        if not dev.is_connected():
            dim_red = lambda t: red(dim(strike(t)))
            colour_fns = [
                None,       # addr – no colour
                None,       # name
                stat_fn,    # status
                dim_red,    # V_set
                dim_red,    # I_set
                dim_red,    # V_out
                dim_red,    # I_out
                dim_red,    # mode
                dim_red,    # out
                dim_red,    # temp
            ]

        print(_data_row(cells_raw, colour_fns))

    print(_hsep("╰", "┴", "╯"))
    print()


# ---------------------------------------------------------------------------
# Convenience: stand-alone live monitor
# ---------------------------------------------------------------------------

def live_monitor(
    config: DPM86XXConfig,
    interval: float = 1.0,
    iterations: int = 0,
    name: str = "",
) -> None:
    """
    Repeatedly poll a device and print its full status panel.

    Internally creates a :class:`DPM86XXDevice`, keeps the connection open
    for the duration, and calls :meth:`~DPM86XXDevice.update_state` +
    :meth:`~DPM86XXDevice.display_state` on each iteration.

    Parameters
    ----------
    config :
        Connection configuration.
    interval :
        Seconds between refreshes.
    iterations :
        How many polls to perform (``0`` = infinite; stop with Ctrl-C).
    name :
        Optional device label passed to :class:`DPM86XXDevice`.
    """
    device = DPM86XXDevice(config, name=name)
    with device:
        count = 0
        try:
            while True:
                device.update_state()
                device.display_state()
                count += 1
                if iterations and count >= iterations:
                    break
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")


# ---------------------------------------------------------------------------
# Minimal self-test / demo  (runs when executed directly)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import datetime

    print("DPM86XX API – self-test / demo")
    print("=" * 60)

    # ── CRC unit tests (reference values from protocol document §6) ───────
    tests = [
        # Read regs 0x0000–0x0001 from slave 0x01 → expected CRC 0x0BC4
        (bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x02]), 0x0BC4, "0x03 read frame"),
        # Write 24.00V to reg 0x0000 via 0x06 → expected CRC 0xB28F
        (bytes([0x01, 0x06, 0x00, 0x00, 0x09, 0x60]), 0xB28F, "0x06 write frame"),
    ]
    all_pass = True
    for payload, expected, label in tests:
        calc = _crc16_modbus(payload)
        ok   = calc == expected
        all_pass = all_pass and ok
        status = "PASS ✓" if ok else "FAIL ✗"
        print(f"  CRC-16 [{label}]:  {calc:#06x} == {expected:#06x}  {status}")

    # ── DPM86XXState smoke test ───────────────────────────────────────────
    print()
    s = DPM86XXState()
    assert not s.is_valid(),  "fresh state should be invalid"
    assert s.age()  is None,  "fresh state age should be None"
    assert s.is_stale(),      "fresh state should be stale"
    s.timestamp = time.time() - 3.0
    assert s.is_valid(),           "state with timestamp should be valid"
    assert not s.is_stale(5.0),    "3s old state should not be stale (max=5s)"
    assert s.is_stale(2.0),        "3s old state should be stale (max=2s)"
    print("  DPM86XXState logic:  PASS ✓")

    # ── Render a mock status panel ────────────────────────────────────────
    mock = DPM86XXState(
        voltage_set      = 12.34,
        current_set      =  1.500,
        output_enabled   = True,
        voltage_measured = 12.31,
        current_measured =  0.748,
        power_measured   =  9.208,
        regulation_mode  = RegulationMode.CV,
        temperature      = 35,
        max_voltage      = 60.0,
        max_current      =  5.0,
        timestamp        = time.time(),
    )
    cfg_mock = DPM86XXConfig(
        port="COM1", baud_rate=BaudRate.B9600, address=1, protocol=Protocol.SIMPLE
    )
    print()
    _render_status_panel(mock, cfg_mock, device_name="Bench PSU")

    # ── Render a mock device list ─────────────────────────────────────────
    dev1 = DPM86XXDevice(
        DPM86XXConfig(port="COM1", address=1, protocol=Protocol.SIMPLE), "Bench PSU"
    )
    dev1.state = mock

    dev2 = DPM86XXDevice(
        DPM86XXConfig(port="COM1", address=2, protocol=Protocol.SIMPLE), "HV Supply"
    )
    dev2.state = DPM86XXState(
        voltage_set=48.0, 
        current_set=3.0,
        output_enabled=True,
        voltage_measured=47.92, 
        current_measured=2.991,
        power_measured=143.4,
        regulation_mode=RegulationMode.CC,
        temperature=42,
        timestamp=time.time() - 8.0,   # stale
    )

    dev3 = DPM86XXDevice(
        DPM86XXConfig(port="COM1", address=3, protocol=Protocol.SIMPLE), "Spare Unit"
    )
    # dev3.state deliberately left empty (never updated)

    display_device_list([dev1, dev2, dev3])

    # ── Usage hints ──────────────────────────────────────────────────────
    print("Quick-start snippets:")
    print()
    print("  # Low-level API")
    print("  from dpm86xx import DPM86XX, DPM86XXConfig, BaudRate")
    print("  cfg = DPM86XXConfig(port='/dev/ttyUSB0', baud_rate=BaudRate.B9600, address=1)")
    print("  with DPM86XX(cfg) as psu:")
    print("      psu.set_voltage(12.0); psu.set_current(2.0); psu.set_output(True)")
    print("      psu.display_status()")
    print()
    print("  # Device objects with state caching")
    print("  from dpm86xx import DPM86XXDevice, DPM86XXConfig, display_device_list")
    print("  dev = DPM86XXDevice(cfg, name='Bench PSU')")
    print("  dev.update_state()        # reads all registers")
    print("  dev.display_state()       # full panel from cache")
    print("  print(dev.is_connected()) # live connectivity probe")
    print("  display_device_list([dev])")