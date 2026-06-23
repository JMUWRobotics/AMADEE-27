"""
dpm86xx.py  –  Python API for the Joy-IT JT-DPM86XX Programmable Lab Power Supply
====================================================================================
Supports both communication protocols documented in the official protocol sheet:

  • Simple (ASCII) Protocol  – human-readable command/response lines over RS-232/RS-485
  • MODBUS RTU Protocol      – binary framing via RS-485

Author  : generated from JT-DPM86XX_Communication-protocol_2025-07-22.pdf
Requires: pyserial  (`pip install pyserial`)

Quick-start example
-------------------
    from dpm86xx import DPM86XX, DPM86XXConfig, BaudRate, Protocol

    cfg = DPM86XXConfig(port="/dev/ttyUSB0", baud_rate=BaudRate.B9600, address=1)
    with DPM86XX(cfg) as psu:
        psu.set_voltage(12.34)
        psu.set_current(1.500)
        psu.set_output(True)
        psu.display_status()        # pretty-print everything to the console
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

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
    """
    port             : str      = "COM1"
    baud_rate        : BaudRate = BaudRate.B9600
    address          : int      = 1
    protocol         : Protocol = Protocol.SIMPLE
    timeout          : float    = 1.0
    write_delay      : float    = 0.05


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

class RegulationMode(IntEnum):
    """Internal representation of the CCCV register (0x1000)."""
    OFF = 0
    CV  = 1
    CC  = 2

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
            
            for mode in RegulationMode:
                if mode.value == raw:
                    return mode
            raise DPM86XXError(f"Unknown regulation mode value {raw} in register 0x1000")

    def read_temperature(self) -> float:
        """
        (r33 / reg 0x1003) Read the device's internal temperature sensor.

        Returns
        -------
        float – temperature in °C, resolution 1 °C
        """
        if self.config.protocol == Protocol.SIMPLE:
            # Protocol example shows empty operand: :01r33=,
            resp = self._simple_send("r", 33, "")
            return float(self._simple_extract(resp))
        else:
            return float(self._modbus_read_regs(0x1003, 1)[0])

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

    def read_all(self) -> dict:
        """
        Read every available parameter in one pass and return a flat dictionary.

        When the MODBUS RTU protocol is active, several read operations are
        batched into multi-register reads to reduce round-trips.

        Returns
        -------
        dict with keys:

        ================  =========  ============================================
        Key               Type       Description
        ================  =========  ============================================
        voltage_set       float      Programmed voltage setpoint [V]
        current_set       float      Programmed current setpoint [A]
        output_enabled    bool       True if output is switched ON
        voltage_measured  float      Measured output voltage [V]
        current_measured  float      Measured output current [A]
        power_measured    float      Derived output power (V×I) [W]
        regulation_mode   str        ``"CV"``, ``"CC"``, or ``"OFF"``
        temperature       float      Device temperature [°C]
        max_voltage       float|None Model maximum voltage [V] (Simple only)
        max_current       float|None Model maximum current [A] (Simple only)
        ================  =========  ============================================
        """
        data: dict = {}

        if self.config.protocol == Protocol.MODBUS_RTU:
            # ── Batch read 1: setpoints + output switch (regs 0x0000–0x0002, 3 words)
            regs_set = self._modbus_read_regs(0x0000, 3)
            data["voltage_set"]     = regs_set[0] / 100.0
            data["current_set"]     = regs_set[1] / 1000.0
            data["output_enabled"]  = bool(regs_set[2])

            # ── Batch read 2: measured values (regs 0x1000–0x1003, 4 words)
            regs_meas = self._modbus_read_regs(0x1000, 4)
            raw_mode  = regs_meas[0]   # CCCV: 0=off, 1=CV, 2=CC
            data["voltage_measured"] = regs_meas[1] / 100.0
            data["current_measured"] = regs_meas[2] / 1000.0
            data["temperature"]      = float(regs_meas[3])
            data["regulation_mode"]  = {0: "OFF", 1: "CV", 2: "CC"}.get(raw_mode, f"?({raw_mode})")
            data["max_voltage"]      = None
            data["max_current"]      = None

        else:   # Simple protocol – one read per register
            try:
                data["max_voltage"] = self.read_max_voltage()
            except DPM86XXError:
                data["max_voltage"] = None
            try:
                data["max_current"] = self.read_max_current()
            except DPM86XXError:
                data["max_current"] = None

            data["voltage_set"]      = self.read_voltage_setting()
            data["current_set"]      = self.read_current_setting()
            data["output_enabled"]   = self.read_output_enabled()
            data["voltage_measured"] = self.read_output_voltage()
            data["current_measured"] = self.read_output_current()
            data["regulation_mode"]  = self.read_regulation_mode()
            data["temperature"]      = self.read_temperature()

        data["power_measured"] = round(
            data["voltage_measured"] * data["current_measured"], 4
        )
        return data

    def display_status(self) -> None:
        """
        Read all available parameters and render a formatted status panel on stdout.

        Calls :meth:`read_all` internally.  ANSI colour codes are used when the
        terminal appears to support them; plain ASCII box-drawing is used otherwise.
        """
        data   = self.read_all()
        _render_status_panel(data, self.config)


# ---------------------------------------------------------------------------
# Console rendering (standalone so it can be called with pre-fetched data)
# ---------------------------------------------------------------------------

def _ansi_supported() -> bool:
    """Return True when stdout is a tty that supports ANSI escape codes."""
    if os.name == "nt":
        return os.environ.get("TERM") is not None or "WT_SESSION" in os.environ
    return hasattr(os, "get_terminal_size") and os.isatty(1)


# ANSI colour shortcuts
_RST  = "\033[0m"
_BOLD = "\033[1m"
_GRN  = "\033[32m"
_RED  = "\033[31m"
_YEL  = "\033[33m"
_CYN  = "\033[36m"
_DIM  = "\033[2m"


def _colour(text: str, code: str, use_colour: bool) -> str:
    return f"{code}{text}{_RST}" if use_colour else text


def _render_status_panel(data: dict, cfg: DPM86XXConfig) -> None:
    """Print a formatted status panel for the given data dictionary."""
    use_col  = _ansi_supported()
    proto    = "Simple" if cfg.protocol == Protocol.SIMPLE else "MODBUS RTU"
    W        = 52          # inner box width
    LINE     = "─" * W

    # Colour helpers
    def bold(t):  return _colour(t, _BOLD, use_col)
    def green(t): return _colour(t, _GRN,  use_col)
    def red(t):   return _colour(t, _RED,  use_col)
    def yel(t):   return _colour(t, _YEL,  use_col)
    def cyn(t):   return _colour(t, _CYN,  use_col)
    def dim(t):   return _colour(t, _DIM,  use_col)

    def row(label: str, value_str: str, val_colour_fn=None) -> str:
        """Format a label/value pair inside a box row (pure ASCII widths)."""
        label_col = 30
        value_col = W - label_col - 1
        coloured  = val_colour_fn(value_str) if val_colour_fn else value_str
        padding   = value_col - len(value_str)   # use raw length for alignment
        return f"│ {label:<{label_col}}{' ' * max(0, padding)}{coloured} │"

    def sep(char="─"):
        return f"├{char * W}┤"

    # ── Output state & regulation mode decorations ──────────────────────
    if data["output_enabled"]:
        out_str   = "ON"
        out_fn    = green
    else:
        out_str   = "OFF"
        out_fn    = red

    mode = data["regulation_mode"]
    mode_fn = yel if mode == "CC" else (green if mode == "CV" else dim)

    # ── Title bar ────────────────────────────────────────────────────────
    title = f"  JT-DPM86XX  ·  {proto}  ·  Address {cfg.address}"
    print()
    print(f"╭{LINE}╮")
    print(f"│{bold(title):^{W}}│" if not use_col else f"│{_BOLD}{title:^{W}}{_RST}│")
    print(f"├{LINE}┤")

    # ── Model limits (Simple protocol only) ─────────────────────────────
    if data["max_voltage"] is not None or data["max_current"] is not None:
        if data["max_voltage"] is not None:
            print(row("Model maximum voltage", f"{data['max_voltage']:.2f} V", cyn))
        if data["max_current"] is not None:
            print(row("Model maximum current", f"{data['max_current']:.3f} A", cyn))
        print(sep())

    # ── Setpoints ────────────────────────────────────────────────────────
    print(row("Voltage setpoint",       f"{data['voltage_set']:.2f} V"))
    print(row("Current setpoint",       f"{data['current_set']:.3f} A"))
    print(sep())

    # ── Measured values ──────────────────────────────────────────────────
    print(row("Measured voltage",       f"{data['voltage_measured']:.2f} V"))
    print(row("Measured current",       f"{data['current_measured']:.3f} A"))
    print(row("Measured power (V×I)",   f"{data['power_measured']:.4f} W"))
    print(sep())

    # ── Status ───────────────────────────────────────────────────────────
    print(row("Output state",           out_str,  out_fn))
    print(row("Regulation mode",        mode,     mode_fn))
    print(row("Device temperature",     f"{data['temperature']:.0f} °C"))
    print(f"╰{LINE}╯")
    print()


# ---------------------------------------------------------------------------
# Convenience: stand-alone live monitor
# ---------------------------------------------------------------------------

def live_monitor(config: DPM86XXConfig, interval: float = 1.0, iterations: int = 0) -> None:
    """
    Open a connection and repeatedly call :meth:`DPM86XX.display_status`.

    Parameters
    ----------
    config     : connection configuration
    interval   : seconds between refreshes
    iterations : how many times to poll (0 = infinite, Ctrl-C to stop)
    """
    with DPM86XX(config) as psu:
        count = 0
        try:
            while True:
                psu.display_status()
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
    import sys

    print("DPM86XX API – self-test / demo")
    print("=" * 52)

    # ── CRC unit test ────────────────────────────────────────────────────
    # Reference values from Section 6 of the protocol document:
    #   Read regs 0x0000 & 0x0001 from slave 0x01 → frame 01 03 00 00 00 02 CRC
    ref_payload = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x02])
    calc        = _crc16_modbus(ref_payload)
    expected    = 0x0BC4   # C4 0B in little-endian → 0x0BC4
    ok          = calc == expected
    print(f"CRC-16 self-test:  calculated={calc:#06x}  expected={expected:#06x}  {'PASS ✓' if ok else 'FAIL ✗'}")

    # ── Verify CRC of the example set-voltage frame (Section 6) ─────────
    # Host sets 24.00V: 01 06 00 00 09 60 → CRC should be 0xB28F (8F B2 LE)
    ref2     = bytes([0x01, 0x06, 0x00, 0x00, 0x09, 0x60])
    calc2    = _crc16_modbus(ref2)
    expected2 = 0xB28F
    ok2      = calc2 == expected2
    print(f"CRC-16 self-test2: calculated={calc2:#06x}  expected={expected2:#06x}  {'PASS ✓' if ok2 else 'FAIL ✗'}")

    print()
    print("To use the API:")
    print("  from dpm86xx import DPM86XX, DPM86XXConfig, BaudRate, Protocol, SaveSlot")
    print()
    print("  cfg = DPM86XXConfig(port='/dev/ttyUSB0', baud_rate=BaudRate.B9600, address=1)")
    print("  with DPM86XX(cfg) as psu:")
    print("      psu.set_voltage(12.0)")
    print("      psu.set_current(2.0)")
    print("      psu.set_output(True)")
    print("      psu.display_status()")
    print()
    print("For live monitoring:")
    print("  from dpm86xx import live_monitor, DPM86XXConfig")
    print("  live_monitor(cfg, interval=2.0)")
