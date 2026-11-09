"""WLAN-Protokoll-Server der Ladestation (asyncio, TCP).

Setzt das Protokoll aus sequence_diagram.md um, kennt aber weder Adapter noch DPM:
Alles, was die Hardware betrifft, steckt hinter der Schnittstelle ``Backend``
(Umsetzung für diese Station: control/comm_manager.py).

Ablauf je Rover (eine TCP-Verbindung = eine Sitzung):

  1. CHARGE_REQUEST -> backend.reserve():  Slot vergeben?
                       -> CHARGE_ACCEPT (Slot-Zuweisung)  oder  CHARGE_REJECT
  2. Erster HEARTBEAT ("Rover steckt am Adapter") -> backend.authorize():
     Laden wird freigegeben. Jeder HEARTBEAT wird mit ACK beantwortet.
     Kommt länger als heartbeat_timeout (15 s) keiner mehr -> backend.deactivate()
     (Notabschaltung, Strom auf 0).
  3. Ende
     a) durch Rover:   CHARGE_COMPLETE -> backend.deactivate() -> ACK
     b) durch Station: CHARGE_STOP     -> Rover trennt, sendet ACK -> backend.deactivate()
                       (kommt kein ACK, wird nach stop_ack_timeout trotzdem abgeschaltet)
        Auslöser: backend.check() liefert einen Grund (z. B. Ladeende, Adapterfehler),
        oder stop_session()/stop_all() von außen.
  Danach (immer): backend.release() gibt den Slot wieder frei.

Alle Sitzungen laufen als asyncio-Tasks im selben Event-Loop wie der Rest der
Station. Alle Backend-Methoden werden im Event-Loop aufgerufen und dürfen ``await``en.
"""
from __future__ import annotations

import asyncio
import contextlib
import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

from drivers.comm_protocol import (
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

log = logging.getLogger("comm.server")

SEND_TIMEOUT_S = 5.0


# --------------------------------------------------------------------------
# Konfiguration
# --------------------------------------------------------------------------
@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = DEFAULT_PORT
    heartbeat_interval: float = HEARTBEAT_INTERVAL_S   # wird dem Rover im ACCEPT mitgeteilt
    heartbeat_timeout: float = HEARTBEAT_TIMEOUT_S     # danach Notabschaltung
    request_timeout: float = 10.0       # nach dem Verbinden muss die Anfrage kommen
    reservation_timeout: float = 300.0  # Slot zugewiesen, aber Rover meldet sich nie am Adapter
    stop_ack_timeout: float = 5.0       # Wartezeit auf ACK nach CHARGE_STOP
    reject_retry_after: float = 30.0    # Hinweis an den Rover bei Ablehnung
    check_interval: float = 0.5         # so oft fragt der Server backend.check()

    def __post_init__(self) -> None:
        if self.heartbeat_timeout <= self.heartbeat_interval:
            raise ValueError("heartbeat_timeout muss größer als heartbeat_interval sein")
        if self.check_interval <= 0:
            raise ValueError("check_interval muss positiv sein")


# --------------------------------------------------------------------------
# Schnittstelle zur Hardware
# --------------------------------------------------------------------------
@dataclass
class Reservation:
    """Ergebnis von Backend.reserve(): welcher Slot gehört dem Rover."""
    rover_id: str
    slot_id: int
    target_values: Dict[str, Any] = field(default_factory=dict)
    accept_extra: Dict[str, Any] = field(default_factory=dict)   # zusätzliche Felder im CHARGE_ACCEPT
    context: Any = None                                          # frei für das Backend


@dataclass
class Rejection:
    """Ergebnis von Backend.reserve(), wenn nicht geladen werden kann."""
    reason: str
    retry_after_s: Optional[float] = None


class Backend:
    """Was der Server von der Hardware-Seite braucht."""

    async def reserve(self, rover_id: str, target_values: Dict[str, Any]) -> Union[Reservation, Rejection]:
        """Anfrage prüfen (freie Slots, Leistung ...) und einen Slot zuweisen oder ablehnen."""
        raise NotImplementedError

    async def authorize(self, reservation: Reservation) -> None:
        """Laden freigeben (erster Heartbeat). Eine Exception beendet die Sitzung mit CHARGE_STOP."""
        raise NotImplementedError

    async def deactivate(self, reservation: Reservation, reason: str) -> bool:
        """Port stromlos schalten. True = sicher aus. Muss mehrfach aufrufbar sein."""
        raise NotImplementedError

    async def release(self, reservation: Reservation) -> None:
        """Slot wieder freigeben (wird nach deactivate() immer aufgerufen)."""
        raise NotImplementedError

    def check(self, reservation: Reservation) -> Optional[str]:
        """Periodische Prüfung. Ein Text beendet die Sitzung durch die Station (Grund an den Rover)."""
        return None

    def ack_info(self, reservation: Reservation) -> Dict[str, Any]:
        """Zusätzliche Felder für das ACK auf einen Heartbeat (z. B. Adapterzustand)."""
        return {}

    def on_status(self, reservation: Reservation, status: Dict[str, Any]) -> None:
        """Live-Status des Rovers aus dem Heartbeat."""


# --------------------------------------------------------------------------
# Eine Sitzung = eine Rover-Verbindung
# --------------------------------------------------------------------------
# Einträge der Inbox:  ("msg", Message) | ("bad", Text) | ("eof", Grund) | ("stop", Grund)
_Item = Tuple[str, Any]


class _Session:
    def __init__(self, server: "CommServer", reader: asyncio.StreamReader,
                 writer: asyncio.StreamWriter) -> None:
        self.server = server
        self.backend = server.backend
        self.cfg = server.config
        self.reader = reader
        self.writer = writer
        self.peer = writer.get_extra_info("peername")
        self.inbox: "asyncio.Queue[_Item]" = asyncio.Queue()
        self.rover_id: Optional[str] = None
        self.res: Optional[Reservation] = None
        self.state = "CONNECTED"      # CONNECTED -> RESERVED -> CHARGING -> STOPPING/ENDED
        self.last_status: Dict[str, Any] = {}
        self.port_active = False      # True ab dem Moment, in dem authorize() aufgerufen wurde
        self._seq = 0
        self._registered = False

    # ----------------------------------------------------------- Hilfsfunktionen
    @property
    def tag(self) -> str:
        return self.rover_id or str(self.peer)

    @property
    def slot_id(self) -> Optional[int]:
        return self.res.slot_id if self.res else None

    async def _send(self, msg_type: str, payload: Optional[Dict[str, Any]] = None) -> Optional[int]:
        """Sendet eine Nachricht. Gibt deren seq zurück, None wenn das Senden scheiterte."""
        if self.writer.is_closing():
            return None
        self._seq += 1
        try:
            self.writer.write(Message(msg_type, self._seq, payload or {}).encode())
            await asyncio.wait_for(self.writer.drain(), SEND_TIMEOUT_S)
        except (ConnectionError, OSError, asyncio.TimeoutError) as exc:
            log.warning("[%s] Senden von %s fehlgeschlagen: %s", self.tag, msg_type, exc)
            return None
        return self._seq

    async def _error(self, code: str, message: str) -> None:
        await self._send(ERROR, {"code": code, "message": message})

    async def _next(self, timeout: float) -> Optional[_Item]:
        """Nächstes Ereignis der Sitzung; None, wenn timeout vorher abläuft."""
        try:
            return await asyncio.wait_for(self.inbox.get(), max(0.0, timeout))
        except asyncio.TimeoutError:
            return None

    async def _deactivate(self, reason: str) -> bool:
        """Port stromlos schalten. True, wenn danach sicher aus (oder nie an)."""
        if not self.port_active or self.res is None:
            return True
        try:
            ok = await self.backend.deactivate(self.res, reason)
        except Exception:  # noqa: BLE001 – Hardwarefehler dürfen die Station nicht stoppen
            log.critical("[%s] Port %s konnte NICHT stromlos geschaltet werden!",
                         self.tag, self.slot_id, exc_info=True)
            return False
        if not ok:
            log.critical("[%s] Port %s meldet: nicht sicher stromlos!", self.tag, self.slot_id)
            return False
        self.port_active = False
        return True

    async def _emergency_off(self, reason: str) -> None:
        log.warning("[%s] NOTABSCHALTUNG Port %s: %s", self.tag, self.slot_id, reason)
        self.state = "ENDED"
        await self._deactivate(f"notabschaltung: {reason}")

    # ------------------------------------------------------------ Empfang (Task)
    async def _pump(self) -> None:
        """Liest vom Socket, zerlegt in Nachrichten und legt sie in die Inbox."""
        buf = LineBuffer()
        try:
            while True:
                data = await self.reader.read(4096)
                if not data:
                    self.inbox.put_nowait(("eof", "connection_closed"))
                    return
                try:
                    lines = buf.feed(data)
                except ProtocolError as exc:
                    self.inbox.put_nowait(("eof", str(exc)))
                    return
                for line in lines:
                    try:
                        self.inbox.put_nowait(("msg", Message.decode(line)))
                    except ProtocolError as exc:
                        self.inbox.put_nowait(("bad", str(exc)))
        except (ConnectionError, OSError) as exc:
            self.inbox.put_nowait(("eof", f"connection_error: {exc}"))

    # ------------------------------------------------------------------ Ablauf
    async def run(self) -> None:
        log.debug("Verbindung von %s", self.peer)
        pump = asyncio.create_task(self._pump())
        try:
            if await self._handshake():
                await self._active()
        except asyncio.CancelledError:
            log.warning("[%s] Sitzung abgebrochen (Station fährt herunter)", self.tag)
            raise
        except Exception:  # noqa: BLE001
            log.exception("[%s] Unerwarteter Fehler – Port wird abgeschaltet", self.tag)
        finally:
            await self._cleanup(pump)

    async def _cleanup(self, pump: "asyncio.Task[None]") -> None:
        pump.cancel()
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await pump
        # Sicherheitsnetz: egal wie die Sitzung endet, der Port ist danach aus.
        if not await self._deactivate("sitzung beendet"):
            await self._deactivate("sitzung beendet (zweiter Versuch)")
        if self.res is not None:
            try:
                await self.backend.release(self.res)
            except Exception:  # noqa: BLE001
                log.exception("[%s] release() ist abgestürzt", self.tag)
        if self._registered and self.server.sessions.get(self.rover_id) is self:
            del self.server.sessions[self.rover_id]
        self.writer.close()
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await asyncio.wait_for(self.writer.wait_closed(), 2.0)
        log.info("[%s] Sitzung beendet", self.tag)

    # ------------------------------------------- 1. Anfrage-Phase (Diagramm)
    async def _handshake(self) -> bool:
        """Wartet auf CHARGE_REQUEST und antwortet mit ACCEPT/REJECT. True = Slot vergeben."""
        item = await self._next(self.cfg.request_timeout)
        if item is None:
            await self._error("request_timeout", "keine CHARGE_REQUEST innerhalb der Wartezeit")
            return False
        kind, value = item
        if kind in ("eof", "stop"):
            return False
        if kind == "bad":
            await self._error("bad_message", value)
            return False
        msg: Message = value
        if msg.type != CHARGE_REQUEST:
            await self._error("unexpected_message", f"erwartet CHARGE_REQUEST, erhalten {msg.type}")
            return False
        rover_id = msg.payload.get("rover_id")
        target_values = msg.payload.get("target_values", {})
        if not isinstance(rover_id, str) or not rover_id.strip() or not isinstance(target_values, dict):
            await self._error("invalid_request", "rover_id (Text) und target_values (Objekt) erforderlich")
            return False
        self.rover_id = rover_id.strip()

        # Doppelte Rover-ID: der Eintrag wird VOR dem await reserviert, damit zwei gleichzeitige
        # Anfragen derselben ID nicht beide durchkommen.
        if self.rover_id in self.server.sessions:
            await self._reject("already_active", None)
            return False
        self.server.sessions[self.rover_id] = self
        self._registered = True

        # "Prüft freie Slots & Solarleistung"
        try:
            result = await self.backend.reserve(self.rover_id, target_values)
        except Exception:  # noqa: BLE001
            log.exception("[%s] reserve() ist abgestürzt", self.tag)
            result = Rejection("backend_error")
        if isinstance(result, Rejection):
            await self._reject(result.reason, result.retry_after_s)
            return False

        self.res = result
        self.state = "RESERVED"
        log.info("[%s] Slot %s zugewiesen (Soll-Werte: %s)", self.tag, self.slot_id, target_values)
        payload = {**result.accept_extra,
                   "slot_id": result.slot_id,
                   "heartbeat_interval": self.cfg.heartbeat_interval,
                   "heartbeat_timeout": self.cfg.heartbeat_timeout}
        return await self._send(CHARGE_ACCEPT, payload) is not None

    async def _reject(self, reason: str, retry_after: Optional[float]) -> None:
        log.info("[%s] Anfrage abgelehnt: %s", self.tag, reason)
        retry = self.cfg.reject_retry_after if retry_after is None else retry_after
        await self._send(CHARGE_REJECT, {"reason": reason, "retry_after_s": retry})

    # ------------------------------- 2. Lade-Phase und 3. Beendigung (Diagramm)
    async def _active(self) -> None:
        """Solange der Slot vergeben ist: Heartbeats, Watchdog, periodische Prüfung, Ende."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self.cfg.reservation_timeout   # bis zum ersten Heartbeat
        next_check = loop.time() + self.cfg.check_interval
        while True:
            now = loop.time()
            item = await self._next(min(deadline, next_check) - now)
            now = loop.time()

            if now >= next_check:   # zeitgesteuert, damit auch viele Nachrichten die Prüfung nicht aushungern
                next_check = now + self.cfg.check_interval
                reason = self._safe_check()
                if reason is not None:
                    await self._station_stop(reason)
                    return

            if item is None:
                if now >= deadline - 1e-3:
                    if self.state == "RESERVED":
                        log.warning("[%s] Rover hat den Adapter nie erreicht (Reservierung abgelaufen)", self.tag)
                        await self._station_stop("reservation_timeout")
                    else:
                        await self._emergency_off("heartbeat_timeout")   # Diagramm: Timer abgelaufen
                    return
                continue   # nur der periodische Wecker

            kind, value = item
            if kind == "eof":
                if self.state == "CHARGING":
                    await self._emergency_off(f"verbindung verloren ({value})")
                else:
                    log.info("[%s] Verbindung vor Ladebeginn geschlossen (%s)", self.tag, value)
                return
            if kind == "stop":
                await self._station_stop(value)
                return
            if kind == "bad":
                await self._error("bad_message", value)   # Watchdog läuft unverändert weiter
                continue

            msg: Message = value
            if msg.type == HEARTBEAT:
                if not await self._on_heartbeat(msg):
                    return
                deadline = loop.time() + self.cfg.heartbeat_timeout
            elif msg.type == CHARGE_COMPLETE:
                await self._on_complete(msg)
                return
            elif msg.type == ERROR:
                log.warning("[%s] Rover meldet Fehler: %s", self.tag, msg.payload)
            else:
                await self._error("unexpected_message", f"{msg.type} ist hier nicht erlaubt")

    def _safe_check(self) -> Optional[str]:
        try:
            return self.backend.check(self.res)
        except Exception:  # noqa: BLE001
            log.exception("[%s] check() ist abgestürzt", self.tag)
            return "backend_error"

    async def _on_heartbeat(self, msg: Message) -> bool:
        """False, wenn die Sitzung danach beendet ist."""
        status = msg.payload.get("status")
        self.last_status = status if isinstance(status, dict) else {}
        if self.state == "RESERVED":
            # Erster Heartbeat = Rover steckt am Adapter -> Laden freigeben
            self.port_active = True   # vor authorize(): auch ein halb gelungenes Freigeben wird später abgeschaltet
            try:
                await self.backend.authorize(self.res)
            except Exception:  # noqa: BLE001
                log.exception("[%s] Port %s konnte nicht freigegeben werden", self.tag, self.slot_id)
                await self._station_stop("port_enable_failed")
                return False
            self.state = "CHARGING"
            log.info("[%s] Laden freigegeben auf Slot %s", self.tag, self.slot_id)
        try:
            extra = self.backend.ack_info(self.res)
        except Exception:  # noqa: BLE001
            log.exception("[%s] ack_info() ist abgestürzt", self.tag)
            extra = {}
        if await self._send(ACK, {**extra, "ack_seq": msg.seq}) is None:
            await self._emergency_off("ACK nicht sendbar")
            return False
        try:
            self.backend.on_status(self.res, dict(self.last_status))
        except Exception:  # noqa: BLE001
            log.exception("[%s] on_status() ist abgestürzt", self.tag)
        return True

    async def _on_complete(self, msg: Message) -> None:
        """Reguläres Ende durch Rover: erst Port aus, dann ACK."""
        ok = await self._deactivate("charge_complete")
        self.state = "ENDED"
        await self._send(ACK, {"ack_seq": msg.seq, "port": "deactivated" if ok else "error"})
        log.info("[%s] Ladevorgang regulär beendet", self.tag)

    async def _station_stop(self, reason: str) -> None:
        """Ende durch Station: CHARGE_STOP, auf ACK des Rovers warten, dann Port aus."""
        self.state = "STOPPING"
        log.warning("[%s] Station beendet den Ladevorgang: %s", self.tag, reason)
        stop_seq = await self._send(CHARGE_STOP, {"reason": reason})
        if stop_seq is not None:
            await self._wait_for_ack(stop_seq)
        await self._deactivate(f"station_stop: {reason}")
        self.state = "ENDED"

    async def _wait_for_ack(self, seq: int) -> bool:
        loop = asyncio.get_running_loop()
        end = loop.time() + self.cfg.stop_ack_timeout
        while True:
            item = await self._next(end - loop.time())
            if item is None:
                log.warning("[%s] Kein ACK auf CHARGE_STOP – schalte trotzdem ab", self.tag)
                return False
            kind, value = item
            if kind == "eof":
                return False   # Rover ist weg: Port kann ohnehin sofort aus
            if kind == "msg" and value.type == ACK and value.payload.get("ack_seq") == seq:
                return True
            # alles andere (z. B. noch eintreffende Heartbeats) wird ignoriert


# --------------------------------------------------------------------------
# Der Server
# --------------------------------------------------------------------------
class CommServer:
    def __init__(self, config: ServerConfig, backend: Backend) -> None:
        self.config = config
        self.backend = backend
        self.sessions: Dict[str, _Session] = {}
        self.port = config.port          # nach start() der tatsächlich benutzte Port
        self._server: Optional[asyncio.AbstractServer] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._tasks: "set[asyncio.Task[None]]" = set()

    # ----------------------------------------------------------- Start / Stopp
    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._server = await asyncio.start_server(self._on_connect, self.config.host, self.config.port)
        self.port = self._server.sockets[0].getsockname()[1]
        log.info("Kommunikation lauscht auf %s:%d (Heartbeat %.0f s / Timeout %.0f s)",
                 self.config.host, self.port, self.config.heartbeat_interval, self.config.heartbeat_timeout)

    async def stop(self) -> None:
        """Herunterfahren: neue Verbindungen sperren, alle Sitzungen beenden (Ports stromlos)."""
        if self._server is not None:
            self._server.close()
        tasks = list(self._tasks)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        if self._server is not None:
            with contextlib.suppress(asyncio.TimeoutError, Exception):
                await asyncio.wait_for(self._server.wait_closed(), 2.0)
        self._server = None
        log.info("Kommunikation beendet")

    async def _on_connect(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        task = asyncio.current_task()
        if task is not None:
            self._tasks.add(task)
        try:
            await _Session(self, reader, writer).run()
        finally:
            if task is not None:
                self._tasks.discard(task)

    # ------------------------------------------ Steuerung von außen (thread-sicher)
    def stop_session(self, rover_id: str, reason: str = "station_stop") -> None:
        """Ende durch die Station (Not-Aus): schickt CHARGE_STOP an den Rover. Aus jedem Thread aufrufbar."""
        self._call_in_loop(self._deliver_stop, rover_id, reason)

    def stop_all(self, reason: str = "station_stop") -> None:
        """Wie stop_session, aber für alle laufenden Sitzungen."""
        self._call_in_loop(self._deliver_stop_all, reason)

    def _call_in_loop(self, fn: Callable[..., None], *args: Any) -> None:
        if self._loop is None:
            raise RuntimeError("Server läuft nicht (start() noch nicht aufgerufen)")
        self._loop.call_soon_threadsafe(fn, *args)

    def _deliver_stop(self, rover_id: str, reason: str) -> None:
        session = self.sessions.get(rover_id)
        if session is None:
            log.warning("stop_session: kein aktiver Rover '%s'", rover_id)
            return
        session.inbox.put_nowait(("stop", reason))

    def _deliver_stop_all(self, reason: str) -> None:
        for session in list(self.sessions.values()):
            session.inbox.put_nowait(("stop", reason))

    def snapshot(self) -> List[Dict[str, Any]]:
        """Übersicht der laufenden Sitzungen. Nur im Event-Loop-Thread aufrufen."""
        return [
            {"rover_id": s.rover_id, "slot_id": s.slot_id, "state": s.state,
             "port_active": s.port_active, "status": dict(s.last_status)}
            for s in self.sessions.values()
        ]
