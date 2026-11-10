"""ROS-2-Node der Roverseite: Kommunikation mit der Ladestation.

Dieser Node macht ausschließlich die Kommunikation nach sequence_diagram.md.
Navigation, Akku-Überwachung und das Festlegen der Soll-Werte machen andere
Nodes; sie sprechen über die unten stehenden Topics mit diesem Node.

Alle Namen sind relativ, liegen also im Namespace des Nodes (z. B. /rover_01/charging/...).
Strukturierte Daten werden als JSON (Text) übertragen. Es gibt drei gleichwertige Wege, den Node zu bedienen:

  1. Topics    (Fire-and-forget, siehe unten)
  2. Services  (wie Topics, aber mit sofortiger Antwort, ob der Befehl angenommen wurde)
  3. Action    charging/session: eine komplette Sitzung mit Feedback und Ergebnis (für Behavior Trees)

  Eingang (andere Nodes -> dieser Node)
    charging/request      std_msgs/String  JSON-Objekt mit den Soll-Werten, z. B. {"voltage_v": 29.4, "current_a": 5.0, "cutoff_a": 0.5}
                                            -> sendet die Lade-Anfrage an die Station
    charging/docked       std_msgs/Empty   Rover steckt am Ladeadapter -> Heartbeat-Schleife startet
    charging/finish       std_msgs/Empty   Rover beendet das Laden (reguläres Ende, CHARGE_COMPLETE)
    charging/live_status  std_msgs/String  JSON-Objekt mit dem Live-Status (Spannung, Strom, ...);
                                            wird mit jedem Heartbeat an die Station geschickt

  Ausgang (dieser Node -> andere Nodes)
    charging/state        std_msgs/String  Zustandsname (transient local), siehe ChargingState:
                                            IDLE REQUESTING ACCEPTED CHARGING FINISHING FINISHED
                                            REJECTED STOPPED_BY_STATION CONNECTION_LOST ERROR
    charging/event        std_msgs/String  JSON {"rover_id", "state", "info"}; info enthält Details,
                                            z. B. bei ACCEPTED die Slot-Zuweisung, bei REJECTED
                                            den Grund, bei STOPPED_BY_STATION den Stopp-Grund

  Services (Paket rover_charging_interfaces bzw. std_srvs)
    charging/request_charge   rover_charging_interfaces/srv/RequestCharge  wie charging/request, mit Antwort
    charging/confirm_docked   std_srvs/srv/Trigger   wie charging/docked (nur nach ACCEPTED möglich)
    charging/finish_charge    std_srvs/srv/Trigger   wie charging/finish (nur in ACCEPTED/CHARGING möglich;
                                                     in ACCEPTED gibt es den Slot ohne Laden wieder frei)
    Die Antwort sagt nur, ob der Befehl angenommen wurde. Ergebnisse der Station kommen als Zustand/Feedback.

  Action
    charging/session          rover_charging_interfaces/action/ChargeSession
      Ziel:      Soll-Werte (JSON) -> Anfrage wird gesendet
      Feedback:  jeder Zustandswechsel (state, slot_id, info_json)
      Ergebnis:  SUCCEEDED bei FINISHED oder Stopp durch die Station mit Grund aus success_stop_reasons
                 (Standard: charge_complete = Akku voll), sonst ABORTED mit reason; cancel beendet sauber
      Angedockt wird weiterhin über confirm_docked / charging/docked gemeldet.
      Es läuft immer nur eine Sitzung: solange sie läuft, werden weitere Ziele und request_charge abgelehnt.

Was die anderen Nodes auf die Zustände tun sollten:
  ACCEPTED            -> zum Ladeadapter fahren, danach charging/docked senden
  REJECTED            -> nicht laden; info.retry_after_s sagt, wann ein neuer Versuch sinnvoll ist
  STOPPED_BY_STATION  -> Ladekontakt lösen (Not-Aus/Ende durch Station)
  CONNECTION_LOST     -> Ladekontakt lösen, Station schaltet den Port ab
  FINISHED            -> Port ist stromlos, Ladeadapter darf verlassen werden
"""
from __future__ import annotations

import json
import queue
import socket
import threading
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from rover_charging_interfaces.action import ChargeSession
from rover_charging_interfaces.srv import RequestCharge

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # ältere rclpy-Versionen
    class ExternalShutdownException(Exception):  # type: ignore[no-redef]
        pass

from .protocol import DEFAULT_PORT
from .rover_client import ChargingState, RoverChargingClient


# Zustände, mit denen eine Sitzung für die Action zu Ende ist (IDLE kommt nur nach close() des Clients)
_ENDING_STATES = (
    ChargingState.FINISHED, ChargingState.REJECTED, ChargingState.STOPPED_BY_STATION,
    ChargingState.CONNECTION_LOST, ChargingState.ERROR, ChargingState.IDLE,
)


class _GoalSession:
    """Eine von der Action gestartete Sitzung: sammelt die Ereignisse und wartet auf das Ende."""

    def __init__(self) -> None:
        self.future = Future()                    # wird beim Ende der Sitzung gesetzt
        self.goal_handle: Any = None              # erst gesetzt, wenn der Execute-Callback läuft
        self.slot_id = -1
        self.last: Optional[Tuple[ChargingState, Dict[str, Any]]] = None
        self.ending: Optional[Tuple[ChargingState, Dict[str, Any]]] = None

    def add(self, state: ChargingState, info: Dict[str, Any]) -> None:
        self.last = (state, info)
        slot = info.get("slot_id")
        if isinstance(slot, int) and not isinstance(slot, bool):
            self.slot_id = slot
        if state in _ENDING_STATES:
            self.ending = (state, info)


class ChargingCommNode(Node):
    def __init__(self) -> None:
        super().__init__("charging_comm")

        self.declare_parameter("rover_id", "")                       # leer -> Namespace, sonst Hostname
        self.declare_parameter("station_host", "raspberrypi.local")
        self.declare_parameter("station_port", DEFAULT_PORT)
        self.declare_parameter("connect_timeout", 5.0)
        self.declare_parameter("response_timeout", 10.0)
        # Voreinstellungen; die Station nennt im ACCEPT ihre eigenen Zeiten, die dann gelten:
        self.declare_parameter("heartbeat_interval", 5.0)
        self.declare_parameter("ack_timeout", 15.0)
        # Stopp-Gründe der Station, die für die Action als Erfolg zählen (Akku voll):
        self.declare_parameter("success_stop_reasons", ["charge_complete"])

        rover_id = str(self.get_parameter("rover_id").value) or self._default_rover_id()
        host = str(self.get_parameter("station_host").value)
        port = int(self.get_parameter("station_port").value)

        self._events: "queue.Queue[Tuple[ChargingState, Dict[str, Any]]]" = queue.Queue()
        self._client = RoverChargingClient(
            rover_id=rover_id,
            station_host=host,
            station_port=port,
            on_event=lambda state, info: self._events.put((state, info)),   # läuft im Client-Thread
            connect_timeout=float(self.get_parameter("connect_timeout").value),
            response_timeout=float(self.get_parameter("response_timeout").value),
            heartbeat_interval=float(self.get_parameter("heartbeat_interval").value),
            ack_timeout=float(self.get_parameter("ack_timeout").value),
        )
        self._rover_id = rover_id
        self._success_stop_reasons: List[str] = [str(r) for r in self.get_parameter("success_stop_reasons").value]
        self._lock = threading.RLock()                 # schützt _goal (Executor kann mehrere Threads haben)
        self._goal: Optional[_GoalSession] = None

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(String, "charging/state", latched)
        self._event_pub = self.create_publisher(String, "charging/event", 10)
        self.create_subscription(String, "charging/request", self._on_request, 10)
        self.create_subscription(Empty, "charging/docked", self._on_docked, 10)
        self.create_subscription(Empty, "charging/finish", self._on_finish, 10)
        self.create_subscription(String, "charging/live_status", self._on_live_status, 10)

        self.create_service(RequestCharge, "charging/request_charge", self._srv_request_charge)
        self.create_service(Trigger, "charging/confirm_docked", self._srv_confirm_docked)
        self.create_service(Trigger, "charging/finish_charge", self._srv_finish_charge)

        # Eigene Callback-Gruppe: der Execute-Callback läuft so lange wie die Sitzung und darf die
        # Topics, Services und den Timer des Nodes nicht blockieren.
        self._action_server = ActionServer(
            self, ChargeSession, "charging/session",
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        # Ereignisse des Client-Threads werden hier im ROS-Thread veröffentlicht
        self.create_timer(0.05, self._drain_events)

        self._state_pub.publish(String(data=ChargingState.IDLE.value))
        self.get_logger().info(f"Rover '{rover_id}' bereit, Station {host}:{port}")

    # ----------------------------------------------------------------- Hilfen
    def _default_rover_id(self) -> str:
        namespace = self.get_namespace().strip("/").replace("/", "_")
        return namespace or socket.gethostname()

    @staticmethod
    def _parse_json(text: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        """-> (Objekt, None) oder (None, Fehlertext). Leerer Text = leeres Objekt."""
        if not text.strip():
            return {}, None
        try:
            value = json.loads(text)
        except ValueError as exc:
            return None, f"kein gültiges JSON ({exc})"
        if not isinstance(value, dict):
            return None, f"JSON-Objekt erwartet, erhalten {type(value).__name__}"
        return value, None

    def _parse_json_object(self, text: str, what: str) -> Optional[Dict[str, Any]]:
        value, error = self._parse_json(text)
        if error is not None:
            self.get_logger().error(f"{what}: {error}")
        return value

    # ------------------------------------------------------------ Befehle (gemeinsam für Topic/Service/Action)
    def _cmd_request(self, target_values: Dict[str, Any]) -> Tuple[bool, str]:
        with self._lock:
            if self._goal is not None:
                return False, "Es läuft bereits eine Sitzung der Action charging/session"
            try:
                self._client.request_charge(target_values)
            except RuntimeError as exc:
                return False, str(exc)
        return True, "Anfrage gesendet"

    def _cmd_docked(self) -> Tuple[bool, str]:
        state = self._client.state
        if not self._client.active:
            return False, "keine aktive Sitzung"
        if state is not ChargingState.ACCEPTED:
            return False, f"Andocken kann nur nach ACCEPTED gemeldet werden (Zustand {state.value})"
        if not self._client.docked():
            return False, "keine aktive Sitzung"
        return True, "Heartbeat-Schleife gestartet"

    def _cmd_finish(self) -> Tuple[bool, str]:
        state = self._client.state
        if not self._client.active:
            return False, "keine aktive Sitzung"
        if state not in (ChargingState.ACCEPTED, ChargingState.CHARGING):
            return False, f"Beenden ist nur in ACCEPTED oder CHARGING möglich (Zustand {state.value})"
        if not self._client.finish():
            return False, "keine aktive Sitzung"
        return True, "Ende wird an die Station gemeldet"

    # --------------------------------------------------------------- Eingänge (Topics)
    def _on_request(self, msg: String) -> None:
        target_values = self._parse_json_object(msg.data, "charging/request")
        if target_values is None:
            return
        ok, message = self._cmd_request(target_values)
        if not ok:
            self.get_logger().warning(f"Anfrage ignoriert: {message}")

    def _on_docked(self, _msg: Empty) -> None:
        ok, message = self._cmd_docked()
        if not ok:
            self.get_logger().warning(f"charging/docked ignoriert: {message}")

    def _on_finish(self, _msg: Empty) -> None:
        ok, message = self._cmd_finish()
        if not ok:
            self.get_logger().warning(f"charging/finish ignoriert: {message}")

    def _on_live_status(self, msg: String) -> None:
        status = self._parse_json_object(msg.data, "charging/live_status")
        if status is not None:
            self._client.set_live_status(status)

    # --------------------------------------------------------------- Services
    def _srv_request_charge(self, request: Any, response: Any) -> Any:
        values, error = self._parse_json(request.target_values_json)
        if error is not None:
            self.get_logger().error(f"charging/request_charge: {error}")
            response.success, response.message = False, error
        else:
            response.success, response.message = self._cmd_request(values)
            if not response.success:
                self.get_logger().warning(f"Anfrage ignoriert: {response.message}")
        response.state = self._client.state.value
        return response

    def _srv_confirm_docked(self, _request: Any, response: Any) -> Any:
        response.success, response.message = self._cmd_docked()
        if not response.success:
            self.get_logger().warning(f"charging/confirm_docked abgelehnt: {response.message}")
        return response

    def _srv_finish_charge(self, _request: Any, response: Any) -> Any:
        response.success, response.message = self._cmd_finish()
        if not response.success:
            self.get_logger().warning(f"charging/finish_charge abgelehnt: {response.message}")
        return response

    # ------------------------------------------------------------------ Action
    def _on_goal(self, goal_request: Any) -> Any:
        values, error = self._parse_json(goal_request.target_values_json)
        if error is not None:
            self.get_logger().error(f"charging/session: Ziel abgelehnt, {error}")
            return GoalResponse.REJECT
        with self._lock:
            if self._goal is not None or self._client.active:
                self.get_logger().warning("charging/session: Ziel abgelehnt, es läuft bereits eine Sitzung")
                return GoalResponse.REJECT
            self._drain_events()          # Reste einer früheren Sitzung dürfen dieses Ziel nicht beenden
            goal = _GoalSession()
            self._goal = goal
            try:
                self._client.request_charge(values)
            except RuntimeError as exc:   # nur bei einem Rennen mit einem anderen Aufrufer
                self._goal = None
                self.get_logger().warning(f"charging/session: Ziel abgelehnt, {exc}")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle: Any) -> Any:
        """Abbruch: vor der Slot-Zuweisung hart schließen, danach sauber beenden (CHARGE_COMPLETE)."""
        state = self._client.state
        if self._client.active:
            if state in (ChargingState.ACCEPTED, ChargingState.CHARGING):
                self._client.finish()
            elif state is ChargingState.REQUESTING:
                self._client.close(timeout=0.2)
            # FINISHING: das Ende läuft schon
        return CancelResponse.ACCEPT

    async def _execute_goal(self, goal_handle: Any) -> Any:
        with self._lock:
            goal = self._goal
        result = ChargeSession.Result()
        result.slot_id = -1
        if goal is None:      # darf nicht vorkommen (Ziel wurde zwischen Annahme und Start verworfen)
            result.state, result.reason, result.info_json = "ERROR", "no_session", "{}"
            goal_handle.abort()
            return result

        goal.goal_handle = goal_handle
        self._publish_feedback(goal)              # bisherigen Stand nachliefern (z. B. REQUESTING)
        await goal.future                         # wird in _drain_events beim Ende der Sitzung gesetzt

        state, info = goal.ending
        reason = str(info.get("reason", ""))
        success = state is ChargingState.FINISHED or (
            state is ChargingState.STOPPED_BY_STATION and reason in self._success_stop_reasons)
        result.success = success
        result.state = state.value
        result.reason = reason
        result.slot_id = goal.slot_id
        result.info_json = json.dumps(info, default=str, ensure_ascii=False)

        with self._lock:
            if self._goal is goal:
                self._goal = None
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _publish_feedback(self, goal: _GoalSession) -> None:
        if goal.goal_handle is None or goal.last is None:
            return
        state, info = goal.last
        feedback = ChargeSession.Feedback()
        feedback.state = state.value
        feedback.slot_id = goal.slot_id
        feedback.info_json = json.dumps(info, default=str, ensure_ascii=False)
        goal.goal_handle.publish_feedback(feedback)

    # ---------------------------------------------------------------- Ausgänge
    def _drain_events(self) -> None:
        while True:
            try:
                state, info = self._events.get_nowait()
            except queue.Empty:
                return
            self._state_pub.publish(String(data=state.value))
            self._event_pub.publish(String(data=json.dumps(
                {"rover_id": self._rover_id, "state": state.value, "info": info},
                default=str, ensure_ascii=False)))
            with self._lock:
                goal = self._goal
                if goal is not None:
                    goal.add(state, info)
                    self._publish_feedback(goal)
                    if goal.ending is not None and not goal.future.done():
                        goal.future.set_result(None)

    def destroy_node(self) -> bool:
        with self._lock:
            goal = self._goal
            if goal is not None and not goal.future.done():   # laufende Action nicht hängen lassen
                goal.ending = (ChargingState.IDLE, {"reason": "node_shutdown"})
                goal.future.set_result(None)
        self._client.close()   # hart schließen; die Station schaltet den Port bei Bedarf ab
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargingCommNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
