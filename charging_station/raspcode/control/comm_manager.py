"""Anbindung des WLAN-Protokolls (comm_server.py) an die Adapter der Station.

Aufgabenteilung
---------------
* **Normalfall = WLAN.** Der Rover schickt seine Sollwerte in der Lade-Anfrage (target_values), die Station
  reserviert einen Adapter, und mit dem ersten Heartbeat ("Rover steckt am Adapter") startet der Ladevorgang
  direkt aus diesen Werten. Heartbeat-Watchdog und Ende-Meldungen laufen ebenfalls über das Protokoll.
* **NFC = Notfall.** Hat ein Rover kein WLAN mehr (z. B. Akku komplett leer), startet die RFID-Karte weiterhin
  einen Ladevorgang wie bisher - ohne Sitzung, ohne Watchdog. Abschaltbar mit ``comm.nfc_fallback: false``.
* Ein vom WLAN reservierter Adapter liest keine Karte (eine Karte kann ihn nicht übernehmen), solange die
  Sitzung läuft. Danach wird erst wieder eine Karte gelesen, wenn die alte einmal weg war.

    Anfrage      -> freien Adapter reservieren (Slot), Werte und Leistung prüfen    -> ACCEPT / REJECT
    1. Heartbeat -> Anfrage an den Adapter geben (Laden startet), ACK
    Heartbeats   -> Watchdog; bleiben sie 15 s aus: Adapter sofort aus
    Ende         -> Rover meldet COMPLETE, oder die Station sendet STOP (Ladeende, Fehler ...)

Alle Methoden laufen im asyncio-Event-Loop der Station (derselbe wie die StateMachine).
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field, fields
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple, Union

from control.adapter_manager import AdapterManager, AdapterState, RequestedValues
from drivers.comm_protocol import DEFAULT_PORT, HEARTBEAT_INTERVAL_S, HEARTBEAT_TIMEOUT_S
from drivers.comm_server import Backend, CommServer, Rejection, Reservation, ServerConfig
from control.energy_manager import EnergyManager, allocate_max_min, allocate_proportional
from control.system_state import State

log = logging.getLogger("comm.manager")

_ACTIVE_STATES = (AdapterState.REQUESTING, AdapterState.SET_VALUES, AdapterState.CHARGING)
_RUNNING_STATES = (AdapterState.SET_VALUES, AdapterState.CHARGING)
_STATIONS_THAT_CAN_CHARGE = (State.IDLE, State.CHARGING)

# Stationszustand -> (Ablehnungsgrund, retry_after_s oder None für den Standardwert)
_REJECT_BY_STATE: Dict[State, Tuple[str, Optional[float]]] = {
    State.BOOT: ("station_booting", 10.0),
    State.FAULT: ("station_fault", None),
    State.DEBUG: ("station_debug", None),
    State.LOW_BATTERY: ("low_battery", None),
    State.LOW_SOLAR_IDLE: ("low_solar", None),
    State.LOW_SOLAR_CHARGING: ("low_solar", None),
}

# Zustände, in denen die Station laufende Sitzungen beendet (Grund -> an den Rover)
_STOP_BY_STATE: Dict[State, str] = {
    State.FAULT: "station_fault",
    State.DEBUG: "station_debug",
}


# --------------------------------------------------------------------------
# Konfiguration (Abschnitt ``comm:`` in system.yaml)
# --------------------------------------------------------------------------
@dataclass
class CommConfig:
    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = DEFAULT_PORT
    heartbeat_interval_s: float = HEARTBEAT_INTERVAL_S
    heartbeat_timeout_s: float = HEARTBEAT_TIMEOUT_S
    request_timeout_s: float = 10.0
    reservation_timeout_s: float = 300.0
    stop_ack_timeout_s: float = 5.0
    reject_retry_after_s: float = 30.0
    check_interval_s: float = 0.5
    min_charge_power_w: float = 20.0   # weniger verfügbare Leistung pro Rover lohnt sich nicht -> "insufficient_power"
    start_timeout_s: float = 300.0     # freigegeben, aber der Adapter lädt nach so langer Zeit noch nicht (keine Leistung ...)
    nfc_fallback: bool = True          # RFID-Karte startet im Notfall einen Ladevorgang ohne WLAN
    default_cutoff_a: Optional[float] = None   # Abschaltstrom, wenn der Rover keinen schickt; None = Pflichtangabe

    @classmethod
    def from_dict(cls, data: Optional[Mapping[str, Any]]) -> "CommConfig":
        data = dict(data or {})
        allowed = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - allowed)
        if unknown:
            # Tippfehler in einer Sicherheitskonfiguration dürfen nicht stillschweigend ignoriert werden.
            raise ValueError(f"unknown key(s) in 'comm' section: {unknown}; allowed: {sorted(allowed)}")
        cfg = cls(**data)
        cfg.validate()
        return cfg

    def validate(self) -> None:
        for name in ("enabled", "nfc_fallback"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"comm.{name} must be true/false, got {getattr(self, name)!r}")
        if not isinstance(self.host, str) or not self.host:
            raise ValueError(f"comm.host must be a non-empty string, got {self.host!r}")
        if isinstance(self.port, bool) or not isinstance(self.port, int) or not 0 <= self.port <= 65535:
            raise ValueError(f"comm.port must be an integer in 0..65535, got {self.port!r}")
        for name in ("heartbeat_interval_s", "heartbeat_timeout_s", "request_timeout_s",
                     "reservation_timeout_s", "stop_ack_timeout_s", "reject_retry_after_s",
                     "check_interval_s", "min_charge_power_w", "start_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value <= 0:
                raise ValueError(f"comm.{name} must be a positive number, got {value!r}")
        if self.default_cutoff_a is not None:
            value = self.default_cutoff_a
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value < 0:
                raise ValueError(f"comm.default_cutoff_a must be null or a number >= 0, got {value!r}")
        if self.heartbeat_timeout_s <= self.heartbeat_interval_s:
            raise ValueError("comm.heartbeat_timeout_s must be larger than comm.heartbeat_interval_s")

    def server_config(self) -> ServerConfig:
        return ServerConfig(
            host=self.host,
            port=self.port,
            heartbeat_interval=float(self.heartbeat_interval_s),
            heartbeat_timeout=float(self.heartbeat_timeout_s),
            request_timeout=float(self.request_timeout_s),
            reservation_timeout=float(self.reservation_timeout_s),
            stop_ack_timeout=float(self.stop_ack_timeout_s),
            reject_retry_after=float(self.reject_retry_after_s),
            check_interval=float(self.check_interval_s),
        )


# --------------------------------------------------------------------------
# Ein reservierter Adapter
# --------------------------------------------------------------------------
@dataclass(eq=False)
class _Slot:
    adapter: AdapterManager
    rover_id: str
    values: RequestedValues               # Sollwerte aus der Anfrage des Rovers
    authorized: bool = False              # Anfrage ist an den Adapter gegeben, Laden läuft/darf laufen
    waiting_since: Optional[float] = None  # seit wann freigegeben, aber der Adapter lädt noch nicht
    last_status: Dict[str, Any] = field(default_factory=dict)

    @property
    def demand_w(self) -> float:
        return self.values.power_w


# --------------------------------------------------------------------------
# Das Backend
# --------------------------------------------------------------------------
class CommManager(Backend):
    def __init__(self, energy: EnergyManager, config: CommConfig,
                 get_state: Callable[[], State]) -> None:
        self.energy = energy
        self.cfg = config
        self._get_state = get_state
        self._by_addr: Dict[int, _Slot] = {}      # DPM-Adresse -> reservierter Slot
        self.server: Optional[CommServer] = None

    # ---------------------------------------------------------------- Lebenszyklus
    async def start(self) -> None:
        """Gate einsetzen und Server starten. Scheitert der Start, bleibt das Gate für WLAN-Adapter zu."""
        self.energy.adapter_gate = self.adapter_gate
        for adapter in self.energy.adapters:
            adapter.rfid_enabled = self.cfg.nfc_fallback
        if not self.cfg.nfc_fallback:
            log.warning("comm.nfc_fallback ist aus: RFID-Karten starten KEINE Ladevorgänge mehr.")
        self.server = CommServer(self.cfg.server_config(), self)
        await self.server.start()

    async def stop(self) -> None:
        """Alle Sitzungen beenden (Ports stromlos) und den Server schließen."""
        if self.server is not None:
            await self.server.stop()

    @property
    def port(self) -> int:
        return self.server.port if self.server is not None else self.cfg.port

    def stop_session(self, rover_id: str, reason: str = "station_stop") -> None:
        """Ende durch die Station für einen Rover (z. B. Not-Aus). Aus jedem Thread aufrufbar."""
        self._require_server().stop_session(rover_id, reason)

    def stop_all(self, reason: str = "station_stop") -> None:
        """Wie stop_session, für alle Rover."""
        self._require_server().stop_all(reason)

    def _require_server(self) -> CommServer:
        if self.server is None:
            raise RuntimeError("CommManager.start() wurde nicht aufgerufen")
        return self.server

    # ---------------------------------------------------------------- Gate
    def adapter_gate(self, adapter: AdapterManager) -> bool:
        """Darf dieser Adapter laden?

        * Adapter mit WLAN-Sitzung: nur, solange die Sitzung freigegeben ist.
        * Adapter ohne Sitzung: ja - das ist der NFC-Notfallweg (ist er abgeschaltet, liest der Adapter
          keine Karte und hat nie eine Anfrage).
        """
        slot = self._by_addr.get(adapter.dpm_address)
        return True if slot is None else slot.authorized

    # ---------------------------------------------------------------- Backend: reserve
    async def reserve(self, rover_id: str, target_values: Dict[str, Any]) -> Union[Reservation, Rejection]:
        # Achtung: zwischen Auswahl des Adapters und dem Eintragen in _by_addr darf KEIN await stehen,
        # sonst könnten zwei gleichzeitige Anfragen denselben Adapter bekommen.
        state = self._get_state()
        if state in _REJECT_BY_STATE:
            reason, retry = _REJECT_BY_STATE[state]
            log.info("Anfrage von %s abgelehnt: Station im Zustand %s", rover_id, state.name)
            return Rejection(reason, retry)

        try:
            values = self._parse_request(target_values)
        except ValueError as exc:
            log.info("Anfrage von %s abgelehnt: %s", rover_id, exc)
            return Rejection("invalid_target_values")

        fitting = [a for a in self.energy.adapters
                   if values.voltage_v <= a.dpm_max_voltage and values.current_a <= a.dpm_max_current]
        if not fitting:
            log.info("Anfrage von %s abgelehnt: kein Adapter kann %s liefern", rover_id, values)
            return Rejection("unsupported_target_values")
        try:
            values.validate(fitting[0].dpm_max_voltage, fitting[0].dpm_max_current)   # u. a. Abschaltstrom < Ladestrom
        except ValueError as exc:
            log.info("Anfrage von %s abgelehnt: %s", rover_id, exc)
            return Rejection("invalid_target_values")

        free = [a for a in fitting
                if a.state is AdapterState.IDLE and a.dpm_address not in self._by_addr]
        if not free:
            log.info("Anfrage von %s abgelehnt: kein freier Adapter", rover_id)
            return Rejection("no_free_slot")
        # Kleinste ausreichende Spannungsklasse zuerst: die 60-V-Adapter bleiben für 60-V-Rover frei.
        adapter = min(free, key=lambda a: (a.dpm_max_voltage, a.dpm_address))

        rejection = self._check_power(values.power_w)
        if rejection is not None:
            log.info("Anfrage von %s abgelehnt: %s (Bedarf %.0f W)", rover_id, rejection.reason, values.power_w)
            return rejection

        slot = _Slot(adapter=adapter, rover_id=rover_id, values=values)
        self._by_addr[adapter.dpm_address] = slot
        adapter.rfid_enabled = False       # ab jetzt gehört der Adapter dieser Sitzung; eine Karte kann ihn nicht übernehmen
        
        return Reservation(rover_id=rover_id, slot_id=adapter.dpm_address,
                           target_values=dict(target_values), context=slot)

    def _parse_request(self, target_values: Mapping[str, Any]) -> RequestedValues:
        """voltage_v und current_a sind Pflicht, cutoff_a (oder comm.default_cutoff_a) ebenfalls, battery_type optional.
        Unbekannte Schlüssel werden ignoriert."""
        def number(key: str, default: Optional[float] = None, minimum_exclusive: bool = True) -> float:
            if key not in target_values:
                if default is None:
                    raise ValueError(f"target_values.{key} fehlt")
                return default
            value = target_values[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                    or (value <= 0 if minimum_exclusive else value < 0):
                raise ValueError(f"target_values.{key} muss eine {'positive' if minimum_exclusive else 'nicht negative'} "
                                 f"Zahl sein, erhalten: {value!r}")
            return float(value)
        rover_id = target_values.get("rover_id")
        if not isinstance(rover_id, str) or not rover_id:
            raise ValueError(f"target_values.rover_id muss eine nicht-leere Zeichenkette sein, erhalten: {rover_id!r}")
        voltage = number("voltage_v")
        current = number("current_a")
        cutoff = number("cutoff_a", self.cfg.default_cutoff_a, minimum_exclusive=False)
        battery_type = target_values.get("battery_type", 0)
        if isinstance(battery_type, bool) or not isinstance(battery_type, int) or not 0 <= battery_type <= 255:
            raise ValueError(f"target_values.battery_type muss eine ganze Zahl 0..255 sein, erhalten: {battery_type!r}")
        return RequestedValues(robot_id=rover_id, tag_id=0, voltage_v=voltage, current_a=current,
                               cutoff_a=cutoff, battery_type=battery_type)

    def _fair(self, demands: List[float], budget: float) -> List[float]:
        allocator = allocate_proportional if self.energy.allocation_policies == "proportional" else allocate_max_min
        return allocator(demands, max(0.0, budget))

    def _existing_demands(self) -> List[float]:
        """Leistungsbedarf aller Adapter, die Leistung bekommen oder bald bekommen (WLAN und NFC-Notfall)."""
        demands: List[float] = []
        for adapter in self.energy.adapters:
            slot = self._by_addr.get(adapter.dpm_address)
            if slot is not None and adapter.state is AdapterState.IDLE:
                demands.append(slot.demand_w)                       # reserviert, Rover noch unterwegs
            elif adapter.state in _ACTIVE_STATES:
                demands.append(adapter.requested_power_w or (slot.demand_w if slot else 0.0))
        return demands

    def _check_power(self, demand_w: float) -> Optional[Rejection]:
        """Reicht die verfügbare Leistung (Netzteil-Limit bzw. MPPT) für einen weiteren Rover?"""
        limit = self.energy.power_limit_reached()
        if limit is None:                       # unbegrenzte Versorgung
            return None
        allocation = self._fair(self._existing_demands() + [demand_w], limit)
        share = allocation[-1]
        needed = min(self.cfg.min_charge_power_w, demand_w)
        if share + 1e-9 < needed:
            return Rejection("insufficient_power")
        return None

    # ---------------------------------------------------------------- Backend: authorize / deactivate / release
    async def authorize(self, reservation: Reservation) -> None:
        """Erster Heartbeat: die Sollwerte des Rovers gehen an den Adapter, das Laden startet (nach Leistungsverteilung)."""
        slot: _Slot = reservation.context
        adapter = slot.adapter
        state = self._get_state()
        if state in _STOP_BY_STATE or state is State.BOOT:
            raise RuntimeError(f"station state {state.name}")
        if adapter.output_voltage_v < adapter._set_voltage:
            await adapter.start_request(slot.values, "comm")     # AdapterError, wenn er nicht IDLE ist (ERROR, OFF ...)
        else:
            raise RuntimeError(f"{adapter.name} is not connected to the rover")
        slot.authorized = True                               # (kein await dazwischen)
        log.info("%s: Laden für Rover %s gestartet (%s)", adapter.name, slot.rover_id, slot.values)

    async def deactivate(self, reservation: Reservation, reason: str) -> bool:
        """Adapter stromlos schalten. True, wenn danach sicher aus.

        Reihenfolge ist Absicht: erst das Gate schließen (der EnergyManager startet den Adapter dann nicht
        wieder), dann Pause + erzwungenes Abschalten in einem Zug unter der Sperre des Adapters. Ein gerade
        laufender step() wird abgewartet und kann den Ausgang danach nicht wieder einschalten.
        """
        slot: _Slot = reservation.context
        adapter = slot.adapter
        was_authorized = slot.authorized
        slot.authorized = False
        if not (was_authorized or adapter.state in _RUNNING_STATES):
            return True
        try:
            await adapter.force_off(f"comm: {reason}")   # unter der Adapter-Sperre: kein step() läuft dazwischen
        except Exception:  # noqa: BLE001
            log.critical("%s: Ausgang konnte nicht abgeschaltet werden (%s)", adapter.name, reason, exc_info=True)
            return False
        log.info("%s: Ausgang aus (%s)", adapter.name, reason)
        return True

    async def release(self, reservation: Reservation) -> None:
        """Sitzung vorbei: Adapter zurück auf IDLE, Slot frei. Der Adapter liest danach wieder Karten (Notfallweg)."""
        slot: _Slot = reservation.context
        adapter = slot.adapter
        slot.authorized = False
        try:
            await adapter.release_request()
        except Exception:  # noqa: BLE001
            log.critical("%s: Freigabe der Anfrage fehlgeschlagen", adapter.name, exc_info=True)
        if self._by_addr.get(adapter.dpm_address) is slot:
            del self._by_addr[adapter.dpm_address]
            adapter.rfid_enabled = self.cfg.nfc_fallback

    # ---------------------------------------------------------------- Backend: check / ack_info / on_status
    def check(self, reservation: Reservation) -> Optional[str]:
        """Alle check_interval_s aufgerufen. Ein Text beendet die Sitzung durch die Station."""
        slot: _Slot = reservation.context
        adapter = slot.adapter

        station = self._get_state()
        stop = _STOP_BY_STATE.get(station)
        if stop is not None:
            return stop
        if adapter.state is AdapterState.ERROR:
            return "adapter_fault"
        if adapter.state is AdapterState.OFF:
            return "adapter_off"
        if slot.authorized and adapter.state is AdapterState.CUTOFF:
            return "charge_complete"

        # Freigegeben, aber der Adapter lädt lange nicht (z. B. keine Leistung): Slot nicht ewig blockieren.
        # Warten wegen niedrigem Akku / wenig Solar zählt nicht.
        if (slot.authorized and adapter.state in (AdapterState.IDLE, AdapterState.REQUESTING)
                and station in _STATIONS_THAT_CAN_CHARGE):
            now = time.monotonic()
            if slot.waiting_since is None:
                slot.waiting_since = now
            elif now - slot.waiting_since > self.cfg.start_timeout_s:
                return "charge_not_started"
        else:
            slot.waiting_since = None
        return None

    def ack_info(self, reservation: Reservation) -> Dict[str, Any]:
        slot: _Slot = reservation.context
        return {"adapter_state": slot.adapter.state.name}

    def on_status(self, reservation: Reservation, status: Dict[str, Any]) -> None:
        slot: _Slot = reservation.context
        slot.last_status = status
