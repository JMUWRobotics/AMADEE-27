"""Protokoll-Client des Rovers (ohne ROS-Abhängigkeit).

Spricht das Ladeprotokoll aus sequence_diagram.md mit der Ladestation. Der Client
kennt weder Navigation noch Akku: er wird von außen gesteuert (der ROS-Node in
charging_comm_node.py ruft ihn auf) und meldet jede Zustandsänderung per Callback.

Ablauf aus Sicht des Aufrufers:

    client.request_charge({"voltage_v": 29.4, "current_a": 5.0, "cutoff_a": 0.5})   # -> REQUESTING
        Station antwortet:
        ACCEPTED (Slot zugewiesen)  oder  REJECTED (Ablehnung / Warten)
    ... Navigation zum Ladeadapter (macht jemand anderes) ...
    client.docked()                 # Heartbeat-Schleife startet
        erstes ACK der Station      -> CHARGING (Port ist unter Strom)
    client.set_live_status({...})   # wird in jedem Heartbeat mitgeschickt
    client.finish()                 # reguläres Ende -> FINISHING -> FINISHED
        oder die Station beendet:   -> STOPPED_BY_STATION (ACK wird automatisch gesendet)

Threads: Jede Sitzung läuft in einem eigenen Hintergrund-Thread. Alle Socket-
Zugriffe passieren ausschließlich dort, die öffentlichen Methoden legen nur
Kommandos in eine Queue. Der on_event-Callback wird im Hintergrund-Thread
aufgerufen und muss deshalb schnell und thread-sicher sein.
"""
from __future__ import annotations

import enum
import logging
import queue
import select
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .protocol import (
    ACK,
    CHARGE_ACCEPT,
    CHARGE_COMPLETE,
    CHARGE_REJECT,
    CHARGE_REQUEST,
    CHARGE_STOP,
    DEFAULT_PORT,
    ERROR,
    HEARTBEAT,
    HEARTBEAT_INTERVAL_S,
    HEARTBEAT_TIMEOUT_S,
    LineBuffer,
    Message,
    ProtocolError,
)

log = logging.getLogger("rover_charging.client")

POLL_S = 0.05   # so oft werden Kommandos und Timer geprüft


class ChargingState(str, enum.Enum):
    IDLE = "IDLE"                          # nichts aktiv
    REQUESTING = "REQUESTING"              # Anfrage gesendet, warte auf ACCEPT/REJECT
    ACCEPTED = "ACCEPTED"                  # Slot zugewiesen -> Rover soll zum Ladeadapter fahren
    CHARGING = "CHARGING"                  # Station hat bestätigt, Port ist unter Strom
    FINISHING = "FINISHING"                # CHARGE_COMPLETE gesendet, warte auf ACK
    FINISHED = "FINISHED"                  # Ende bestätigt, Port ist stromlos
    REJECTED = "REJECTED"                  # Station hat abgelehnt (info: reason, retry_after_s)
    STOPPED_BY_STATION = "STOPPED_BY_STATION"  # Not-Aus / Ende durch Station -> Kontakt lösen
    CONNECTION_LOST = "CONNECTION_LOST"    # Verbindung/ACKs weg
    ERROR = "ERROR"                        # Station nicht erreichbar, Protokollfehler ...


# Nach diesen Zuständen endet die Sitzung (der Sitzungs-Thread beendet sich danach)
_SESSION_END_STATES = (
    ChargingState.IDLE, ChargingState.FINISHED, ChargingState.REJECTED, ChargingState.STOPPED_BY_STATION,
    ChargingState.CONNECTION_LOST, ChargingState.ERROR,
)

EventCallback = Callable[[ChargingState, Dict[str, Any]], None]


class _ConnectionClosed(Exception):
    """Die Station hat die Verbindung geschlossen."""


class _Shutdown(Exception):
    """close() wurde aufgerufen."""


class _Link:
    """Eine TCP-Verbindung mit Zeilen-Framing. Nur aus dem Sitzungs-Thread benutzen."""

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self._buf = LineBuffer()
        self._seq = 0
        self._unread: List[Message] = []

    def send(self, msg_type: str, payload: Optional[Dict[str, Any]] = None) -> int:
        self._seq += 1
        self.sock.sendall(Message(msg_type, self._seq, payload or {}).encode())
        return self._seq

    def push_back(self, msgs: List[Message]) -> None:
        """Bereits empfangene, aber noch nicht verarbeitete Nachrichten zurücklegen."""
        self._unread = list(msgs) + self._unread

    def poll(self, timeout: float) -> List[Message]:
        """Wartet bis zu timeout auf Daten; liefert die vollständig empfangenen Nachrichten."""
        if self._unread:
            out, self._unread = self._unread, []
            return out
        ready, _, _ = select.select([self.sock], [], [], max(0.0, timeout))
        if not ready:
            return []
        data = self.sock.recv(4096)
        if not data:
            raise _ConnectionClosed("connection_closed")
        out: List[Message] = []
        try:
            lines = self._buf.feed(data)
        except ProtocolError as exc:
            raise _ConnectionClosed(str(exc)) from exc
        for line in lines:
            try:
                out.append(Message.decode(line))
            except ProtocolError as exc:
                log.warning("Ungültige Nachricht der Station verworfen: %s", exc)
        return out

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def _positive(value: Any, default: float) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0:
        return float(value)
    return default


class RoverChargingClient:
    def __init__(
        self,
        rover_id: str,
        station_host: str,
        station_port: int = DEFAULT_PORT,
        on_event: Optional[EventCallback] = None,
        connect_timeout: float = 5.0,
        response_timeout: float = 10.0,
        ack_timeout: float = HEARTBEAT_TIMEOUT_S,
        heartbeat_interval: float = HEARTBEAT_INTERVAL_S,
        send_timeout: float = 5.0,
    ) -> None:
        """
        response_timeout   so lange auf ACCEPT/REJECT warten
        ack_timeout        ohne ACK der Station so lange -> CONNECTION_LOST
        heartbeat_interval Voreinstellung; die Station nennt im ACCEPT ihre eigenen Werte
                           (heartbeat_interval / heartbeat_timeout), die dann gelten
        """
        self.rover_id = rover_id
        self.host = station_host
        self.port = station_port
        self.on_event = on_event
        self.connect_timeout = connect_timeout
        self.response_timeout = response_timeout
        self.ack_timeout = ack_timeout
        self.heartbeat_interval = heartbeat_interval
        self.send_timeout = send_timeout

        self._lock = threading.Lock()
        self._state = ChargingState.IDLE
        self._active = False
        self._thread: Optional[threading.Thread] = None
        self._cmds: "queue.Queue[str]" = queue.Queue()
        self._live_status: Dict[str, Any] = {}
        self._live_status_time: Optional[float] = None

    # -------------------------------------------------------------- öffentlich
    @property
    def state(self) -> ChargingState:
        return self._state

    @property
    def active(self) -> bool:
        """True, solange eine Sitzung läuft (von request_charge() bis zum letzten Ereignis)."""
        return self._active

    def request_charge(self, target_values: Optional[Dict[str, Any]] = None) -> None:
        """Startet eine Sitzung: Verbindung aufbauen und CHARGE_REQUEST senden.

        Wirft RuntimeError, wenn bereits eine Sitzung läuft. Das Ergebnis kommt als Event.
        """
        with self._lock:
            if self._active:
                raise RuntimeError(f"Es läuft bereits eine Sitzung (Zustand {self._state.value})")
            self._active = True
            self._cmds = cmds = queue.Queue()
            thread = threading.Thread(
                target=self._run, args=(dict(target_values or {}), cmds),
                name=f"charging-{self.rover_id}", daemon=True,
            )
            self._thread = thread
        thread.start()

    def docked(self) -> bool:
        """Rover steckt am Ladeadapter: Heartbeat-Schleife starten (nur nach ACCEPTED)."""
        return self._command("docked")

    def finish(self) -> bool:
        """Reguläres Ende durch den Rover: CHARGE_COMPLETE senden."""
        return self._command("finish")

    def set_live_status(self, status: Dict[str, Any]) -> None:
        """Aktueller Live-Status; wird mit jedem Heartbeat an die Station geschickt."""
        with self._lock:
            self._live_status = dict(status)
            self._live_status_time = time.monotonic()

    def close(self, timeout: float = 2.0) -> None:
        """Sitzung hart beenden (Verbindung schließen) und auf den Thread warten.

        Die Station merkt das am geschlossenen Socket: war der Port an, schaltet sie
        sofort ab. Für ein sauberes Ende vorher finish() aufrufen.
        """
        with self._lock:
            if self._active:
                self._cmds.put("shutdown")
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)

    # ------------------------------------------------------------------ intern
    def _command(self, name: str) -> bool:
        with self._lock:
            if not self._active:
                return False
            self._cmds.put(name)
            return True

    def _emit(self, state: ChargingState, **info: Any) -> None:
        with self._lock:
            self._state = state
            if state in _SESSION_END_STATES:
                # Die Sitzung ist mit diesem Ereignis zu Ende: ein sofort folgendes request_charge() muss
                # klappen. (Der Thread räumt danach nur noch seine Verbindung auf.)
                self._active = False
        log.info("Zustand: %s %s", state.value, info if info else "")
        if self.on_event is not None:
            try:
                self.on_event(state, info)
            except Exception:  # noqa: BLE001
                log.exception("on_event-Callback ist abgestürzt")

    def _heartbeat_payload(self) -> Dict[str, Any]:
        with self._lock:
            age = None if self._live_status_time is None else round(time.monotonic() - self._live_status_time, 3)
            return {"status": dict(self._live_status), "status_age_s": age}

    # ------------------------------------------------------------ Sitzungs-Thread
    def _run(self, target_values: Dict[str, Any], cmds: "queue.Queue[str]") -> None:
        link: Optional[_Link] = None
        try:
            self._emit(ChargingState.REQUESTING, target_values=target_values)
            try:
                sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
            except OSError as exc:
                self._emit(ChargingState.ERROR, reason="connect_failed", message=str(exc))
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.send_timeout)
            link = _Link(sock)
            self._converse(link, cmds, target_values)
        except _Shutdown:
            self._emit(ChargingState.IDLE, reason="closed")
        except (_ConnectionClosed, OSError) as exc:
            reason = str(exc) or exc.__class__.__name__
            if self._state == ChargingState.REQUESTING:
                self._emit(ChargingState.ERROR, reason="connection_closed", message=reason)
            else:
                self._emit(ChargingState.CONNECTION_LOST, reason=reason)
        except Exception as exc:  # noqa: BLE001
            log.exception("Unerwarteter Fehler in der Lade-Sitzung")
            self._emit(ChargingState.ERROR, reason="internal_error", message=repr(exc))
        finally:
            if link is not None:
                link.close()
            with self._lock:
                if self._thread is threading.current_thread():   # nicht die Sitzung eines Nachfolgers beenden
                    self._active = False

    @staticmethod
    def _drain(cmds: "queue.Queue[str]") -> List[str]:
        out: List[str] = []
        while True:
            try:
                out.append(cmds.get_nowait())
            except queue.Empty:
                return out

    def _converse(self, link: _Link, cmds: "queue.Queue[str]", target_values: Dict[str, Any]) -> None:
        # ---- 1. Anfrage-Phase: Rover -> CS: Lade-Anfrage (ID, Soll-Werte)
        link.send(CHARGE_REQUEST, {"rover_id": self.rover_id, "target_values": target_values})
        reply = self._await_reply(link, cmds)
        if reply is None:
            return   # Timeout: ERROR wurde schon gemeldet
        if reply.type == CHARGE_REJECT:
            self._emit(ChargingState.REJECTED, **reply.payload)   # "Ablehnung / Warten": Rover bricht ab
            return
        if reply.type == ERROR:
            self._emit(ChargingState.ERROR, reason=str(reply.payload.get("code", "station_error")),
                       message=str(reply.payload.get("message", "")))
            return
        if reply.type != CHARGE_ACCEPT:
            self._emit(ChargingState.ERROR, reason="unexpected_reply", message=f"Antwort {reply.type}")
            return

        # ---- Slot-Zuweisung erhalten; Zeiten gibt die Station vor
        accept = reply.payload
        hb_interval = _positive(accept.get("heartbeat_interval"), self.heartbeat_interval)
        ack_timeout = _positive(accept.get("heartbeat_timeout"), self.ack_timeout)
        self._emit(ChargingState.ACCEPTED, **accept)

        # ---- 2. Lade-Phase (Heartbeat) und 3. Beendigungs-Phase
        heartbeating = False
        next_heartbeat = 0.0
        last_ack = 0.0
        complete_seq: Optional[int] = None
        finish_deadline = 0.0

        while True:
            now = time.monotonic()

            for cmd in self._drain(cmds):
                if cmd == "shutdown":
                    raise _Shutdown()
                if cmd == "docked":
                    if self._state == ChargingState.ACCEPTED and not heartbeating:
                        heartbeating = True
                        next_heartbeat = now      # erster Heartbeat sofort: Station schaltet den Port ein
                        last_ack = now
                    else:
                        log.warning("docked() ignoriert (Zustand %s)", self._state.value)
                elif cmd == "finish":
                    if complete_seq is None:
                        # Reguläres Ende durch Rover (Rover gibt den Ladeslot frei)
                        complete_seq = link.send(CHARGE_COMPLETE)
                        heartbeating = False
                        finish_deadline = now + ack_timeout
                        self._emit(ChargingState.FINISHING)

            if heartbeating and now >= next_heartbeat:
                link.send(HEARTBEAT, self._heartbeat_payload())   # Rover -> CS: Heartbeat / Live-Status
                next_heartbeat = now + hb_interval

            for msg in link.poll(POLL_S):
                if msg.type == ACK:
                    ack_seq = msg.payload.get("ack_seq")
                    if complete_seq is not None and ack_seq == complete_seq:
                        self._emit(ChargingState.FINISHED, **msg.payload)   # ACK & Port deaktiviert
                        return
                    if heartbeating:
                        last_ack = time.monotonic()
                        if self._state == ChargingState.ACCEPTED:
                            self._emit(ChargingState.CHARGING)   # erstes ACK: Port ist an
                elif msg.type == CHARGE_STOP:
                    # Not-Aus / Ende durch Station: "Trennt Verbindung", dann ACK
                    self._emit(ChargingState.STOPPED_BY_STATION,
                               reason=str(msg.payload.get("reason", "unknown")))
                    try:
                        link.send(ACK, {"ack_seq": msg.seq})
                    except OSError as exc:
                        log.warning("ACK auf CHARGE_STOP nicht sendbar: %s", exc)
                    return
                elif msg.type == ERROR:
                    log.warning("Station meldet Fehler: %s", msg.payload)
                else:
                    log.warning("Unerwartete Nachricht %s ignoriert", msg.type)

            now = time.monotonic()
            if heartbeating and now - last_ack > ack_timeout:
                self._emit(ChargingState.CONNECTION_LOST, reason="ack_timeout")
                return
            if complete_seq is not None and now > finish_deadline:
                self._emit(ChargingState.CONNECTION_LOST, reason="complete_not_acknowledged")
                return

    def _await_reply(self, link: _Link, cmds: "queue.Queue[str]") -> Optional[Message]:
        """Wartet auf die erste Antwort der Station (ACCEPT / REJECT / ERROR)."""
        deadline = time.monotonic() + self.response_timeout
        while True:
            for cmd in self._drain(cmds):
                if cmd == "shutdown":
                    raise _Shutdown()
                log.warning("%s() vor der Slot-Zuweisung ignoriert", cmd)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._emit(ChargingState.ERROR, reason="response_timeout",
                           message="keine Antwort der Station auf die Anfrage")
                return None
            msgs = link.poll(min(POLL_S, remaining))
            for i, msg in enumerate(msgs):
                if msg.type in (CHARGE_ACCEPT, CHARGE_REJECT, ERROR):
                    link.push_back(msgs[i + 1:])   # nichts verlieren, was direkt danach kam
                    return msg
                log.warning("Unerwartete Nachricht %s vor der Antwort ignoriert", msg.type)
