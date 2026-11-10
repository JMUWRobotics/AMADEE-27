"""Gemeinsames Protokoll Rover <-> Ladestation (Version 1).

WICHTIG: Diese Datei liegt identisch an zwei Stellen:
  - charging_station/protocol.py
  - rover_ws/src/rover_charging_comm/rover_charging_comm/protocol.py
Änderungen immer in beiden Kopien machen (tests/test_protocol.py prüft das).

Transport
---------
TCP über WLAN, eine Verbindung pro Ladevorgang. Jede Nachricht ist genau eine
Zeile JSON (UTF-8, am Ende "\\n"):

    {"v": 1, "type": "HEARTBEAT", "seq": 7, "payload": {...}}

  v        Protokollversion
  type     Nachrichtentyp (siehe unten)
  seq      fortlaufende Nummer je Absender und Verbindung (ab 1)
  payload  Objekt mit den Nutzdaten des jeweiligen Typs

Nachrichten (Nummern entsprechen dem Sequenzdiagramm)
-----------------------------------------------------
  Rover -> Station
    CHARGE_REQUEST   rover_id, target_values{}                       (1)
    HEARTBEAT        status{}, status_age_s                          (Laden)
    CHARGE_COMPLETE  -                                               (Ende durch Rover)

  Station -> Rover
    CHARGE_ACCEPT    slot_id, heartbeat_interval, heartbeat_timeout  (Slot-Zuweisung)
    CHARGE_REJECT    reason, retry_after_s                           (Ablehnung / Warten)
    ACK              ack_seq, ...                                    (Antwort auf HEARTBEAT,
                                                                      CHARGE_COMPLETE; Rover -> Station
                                                                      auch Antwort auf CHARGE_STOP)
    CHARGE_STOP      reason                                          (Ende durch Station / Not-Aus)

  Beide Richtungen
    ERROR            code, message                                   (unzulässige/ungültige Nachricht)

Der Inhalt von target_values (Soll-Werte) und status (Live-Status) wird hier
bewusst NICHT festgelegt: beide Seiten reichen die Objekte unverändert durch.
Zusätzliche Felder in payload werden ignoriert, das Protokoll bleibt so erweiterbar.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List

PROTOCOL_VERSION = 1
DEFAULT_PORT = 9000
MAX_LINE_BYTES = 64 * 1024

HEARTBEAT_INTERVAL_S = 5.0   # Diagramm: Heartbeat alle 5 s
HEARTBEAT_TIMEOUT_S = 15.0   # Diagramm: Notabschaltung nach > 15 s ohne Heartbeat

# Rover -> Station
CHARGE_REQUEST = "CHARGE_REQUEST"
HEARTBEAT = "HEARTBEAT"
CHARGE_COMPLETE = "CHARGE_COMPLETE"

# Station -> Rover
CHARGE_ACCEPT = "CHARGE_ACCEPT"
CHARGE_REJECT = "CHARGE_REJECT"
CHARGE_STOP = "CHARGE_STOP"

# Beide Richtungen
ACK = "ACK"
ERROR = "ERROR"

ALL_TYPES = frozenset({
    CHARGE_REQUEST, HEARTBEAT, CHARGE_COMPLETE,
    CHARGE_ACCEPT, CHARGE_REJECT, CHARGE_STOP,
    ACK, ERROR,
})


class ProtocolError(ValueError):
    """Eingehende Daten sind kein gültiges Protokollpaket."""


@dataclass
class Message:
    type: str
    seq: int
    payload: Dict[str, Any] = field(default_factory=dict)

    def encode(self) -> bytes:
        """Eine Protokollzeile (inkl. abschließendem Zeilenumbruch)."""
        obj = {"v": PROTOCOL_VERSION, "type": self.type, "seq": self.seq, "payload": self.payload}
        return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"

    @classmethod
    def decode(cls, line: bytes) -> "Message":
        """Liest eine Zeile; wirft ProtocolError, wenn sie nicht zum Protokoll passt."""
        try:
            obj = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            raise ProtocolError(f"ungültiges JSON: {exc}") from exc
        if not isinstance(obj, dict):
            raise ProtocolError("Nachricht ist kein JSON-Objekt")
        if obj.get("v") != PROTOCOL_VERSION:
            raise ProtocolError(f"Protokollversion {obj.get('v')!r} nicht unterstützt")
        mtype = obj.get("type")
        if mtype not in ALL_TYPES:
            raise ProtocolError(f"unbekannter Nachrichtentyp {mtype!r}")
        seq = obj.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise ProtocolError("'seq' fehlt oder ist keine ganze Zahl")
        payload = obj.get("payload", {})
        if not isinstance(payload, dict):
            raise ProtocolError("'payload' ist kein Objekt")
        return cls(mtype, seq, payload)


class LineBuffer:
    """Setzt einen TCP-Bytestrom wieder zu Zeilen zusammen.

    TCP kennt keine Nachrichtengrenzen: ein recv() kann Teile einer Zeile oder
    mehrere Zeilen liefern. feed() gibt nur vollständige Zeilen zurück.
    """

    def __init__(self, max_line_bytes: int = MAX_LINE_BYTES) -> None:
        self._buf = bytearray()
        self._max = max_line_bytes

    def feed(self, data: bytes) -> List[bytes]:
        self._buf.extend(data)
        lines: List[bytes] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            if len(line) > self._max:
                raise ProtocolError("Zeile zu lang")
            if line.strip():
                lines.append(line)
        if len(self._buf) > self._max:
            raise ProtocolError("Zeile zu lang")
        return lines
