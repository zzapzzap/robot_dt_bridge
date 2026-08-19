"""Production Robot <-> process PLC <-> Jetson ROS 2 gateway.

The process PLC remains the owner of the robot/CC-Link contract.  This node
owns exactly one MELSEC MC TCP session and exposes PLC pose data plus explicit
command-register readback as ROS topics.  Writes are event driven and available
only from the explicit ROS services below; startup, reconnect, Unity topics,
and status polling never write PLC devices.

``controller_ack`` in a service response means only that the MC batch-write
received a successful PLC response.  It is not a robot-controller ACK and is
not safety rated.  ``confirmed`` is reserved for independent mapped robot
actual feedback; command-word readback is reported separately.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from robot_bridge_msgs.msg import (
    RobotControlState,
    RobotMemory,
    RobotPose,
    RobotStatus,
)
from robot_bridge_msgs.srv import (
    GetRobotStatus,
    RequestStart,
    RequestStop,
    SetHold,
    SetSpeedPercent,
    TriggerRobotAction,
)

from .config_loader import PlcBridgeConfig
from .mc_client import McClient, McConfig, McError, bit_of, words_to_dword


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class _Snapshot:
    """Last complete read of one robot's PLC register map."""

    sequence: int = 0
    contact_wall_s: float = 0.0
    contact_monotonic_s: Optional[float] = None
    connection_state: int = RobotStatus.CONNECTION_CONNECTING
    operation_state: Optional[int] = None
    raw_axes: List[int] = field(default_factory=list)
    degrees: List[float] = field(default_factory=list)
    clamped: bool = False
    run: Optional[bool] = None
    hold: Optional[bool] = None
    emergency_stop: Optional[bool] = None
    fault_reset: Optional[bool] = None
    device_home: Optional[bool] = None
    robot_home: Optional[bool] = None
    standby: Optional[bool] = None
    control_word_raw: int = 0
    speed_down_1_raw: int = 0
    speed_down_2_raw: int = 0
    speed_down_3_raw: int = 0
    speed_down_1: Optional[bool] = None
    speed_down_2: Optional[bool] = None
    speed_down_3: Optional[bool] = None
    requested_speed_percent: Optional[float] = None
    read_latency_ms: float = 0.0
    status_message: str = "PLC feedback has not been read yet"


@dataclass
class _Runtime:
    instance: object
    snapshot: _Snapshot = field(default_factory=_Snapshot)
    lock: threading.RLock = field(default_factory=threading.RLock)
    pub_memory: object = None
    pub_pose: object = None
    pub_pose_unity: object = None
    pub_joint: object = None
    pub_status: object = None
    pub_control: object = None


def _value(obj, name: str, default=None):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _connection_config(value) -> McConfig:
    if isinstance(value, McConfig):
        return value
    if isinstance(value, dict):
        return McConfig.from_dict(value)
    fields = McConfig.__dataclass_fields__  # noqa: SLF001
    return McConfig.from_dict({
        name: getattr(value, name)
        for name in fields
        if hasattr(value, name)
    })


class PlcGatewayNode(Node):
    """One process-PLC session serving every enabled register-map instance."""

    def __init__(self) -> None:
        super().__init__("plc_gateway_node")

        self.declare_parameter("profile", "sim")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("allow_field_control_writes", False)
        profile = str(self.get_parameter("profile").value)
        config_dir = str(self.get_parameter("config_dir").value or "") or None
        self.profile = profile
        self.allow_field_control_writes = bool(
            self.get_parameter("allow_field_control_writes").value
        )

        self.cfg = PlcBridgeConfig.load(config_dir, profile=profile)
        instances = list(self.cfg.enabled_instances())
        if not instances:
            raise ValueError("PLC configuration has no enabled robot instance")

        self.poll_hz = float(self.cfg.poll_hz)
        self.status_publish_hz = float(self.cfg.status_publish_hz)
        self.stale_timeout_s = float(self.cfg.stale_timeout_ms) / 1000.0
        self.verify_timeout_s = float(self.cfg.verify_timeout_s)
        self.allowed_speeds = tuple(
            sorted({int(value) for value in self.cfg.allowed_speed_percent})
        )
        if self.poll_hz <= 0.0 or self.status_publish_hz <= 0.0:
            raise ValueError("PLC poll/status publish rates must be positive")
        if self.stale_timeout_s <= 0.0 or self.verify_timeout_s <= 0.0:
            raise ValueError("PLC stale/verification timeouts must be positive")

        self.client = McClient(
            _connection_config(self.cfg.connection), self.get_logger()
        )
        self._transport_lock = threading.RLock()
        self._stop_generation_lock = threading.Lock()
        self._stop_generation = 0
        self._next_connect_monotonic_s = 0.0
        self._shutting_down = threading.Event()
        self.context.on_shutdown(self._shutting_down.set)

        self._poll_callbacks = MutuallyExclusiveCallbackGroup()
        self._status_callbacks = MutuallyExclusiveCallbackGroup()
        self._motion_callbacks = MutuallyExclusiveCallbackGroup()
        # A stop increments its generation before waiting for the shared MC
        # transport, so a queued start/speed callback cannot pass it.
        self._stop_callbacks = ReentrantCallbackGroup()

        self.runtimes: Dict[str, _Runtime] = {}
        self._services: List[object] = []
        for instance in instances:
            runtime = self._create_runtime(instance)
            robot_id = self._robot_id(instance)
            if robot_id in self.runtimes:
                raise ValueError(f"duplicate PLC robot id: {robot_id}")
            self.runtimes[robot_id] = runtime
            self._create_services(robot_id)

        # These timers only read or publish cached data.  There is deliberately
        # no command-rewrite/watchdog timer.
        self.create_timer(
            1.0 / self.poll_hz,
            self.poll_plc,
            callback_group=self._poll_callbacks,
        )
        self.create_timer(
            1.0 / self.status_publish_hz,
            self.publish_statuses,
            callback_group=self._status_callbacks,
        )

        endpoint = self.cfg.connection
        policy = (
            "commands enabled"
            if self.cfg.commissioned and self.cfg.commands_enabled
            else "read only / uncommissioned"
        )
        self.get_logger().info(
            "PLC gateway %s:%s profile=%s robots=%s poll=%gHz status=%gHz "
            "[%s]"
            % (
                _value(endpoint, "host", "?"),
                _value(endpoint, "port", "?"),
                profile,
                ",".join(self.runtimes),
                self.poll_hz,
                self.status_publish_hz,
                policy,
            )
        )
        self.get_logger().warn(
            "PLC command services are supervisory process requests; they do "
            "not replace the robot safety PLC, guards, or hard-wired E-stop"
        )

    # --------------------------------------------------------------- ROS setup
    def _create_runtime(self, instance) -> _Runtime:
        robot_id = self._robot_id(instance)
        topics = dict(_value(instance, "topics", {}) or {})
        namespace = f"/robot/{robot_id}"
        runtime = _Runtime(instance=instance)
        runtime.pub_memory = self.create_publisher(
            RobotMemory,
            topics.get("memory", namespace + "/memory"),
            SENSOR_QOS,
        )
        runtime.pub_pose = self.create_publisher(
            RobotPose,
            topics.get("pose", namespace + "/pose"),
            SENSOR_QOS,
        )
        runtime.pub_pose_unity = self.create_publisher(
            RobotPose,
            topics.get("unity_pose_raw", namespace + "/cmd_degs_raw"),
            SENSOR_QOS,
        )
        runtime.pub_joint = self.create_publisher(
            JointState,
            topics.get("joint_states", namespace + "/joint_states"),
            SENSOR_QOS,
        )
        runtime.pub_status = self.create_publisher(
            RobotStatus,
            topics.get("status", namespace + "/status"),
            SENSOR_QOS,
        )
        runtime.pub_control = self.create_publisher(
            RobotControlState,
            namespace + "/control_state",
            SENSOR_QOS,
        )
        return runtime

    def _create_services(self, robot_id: str) -> None:
        namespace = f"/robot/{robot_id}"
        self._services.extend([
            self.create_service(
                GetRobotStatus,
                namespace + "/get_status",
                lambda request, response, rid=robot_id:
                self.on_get_status(rid, request, response),
                callback_group=self._status_callbacks,
            ),
            self.create_service(
                RequestStop,
                namespace + "/request_stop",
                lambda request, response, rid=robot_id:
                self.on_request_stop(rid, request, response),
                callback_group=self._stop_callbacks,
            ),
            self.create_service(
                RequestStart,
                namespace + "/request_start",
                lambda request, response, rid=robot_id:
                self.on_request_start(rid, request, response),
                callback_group=self._motion_callbacks,
            ),
            self.create_service(
                SetSpeedPercent,
                namespace + "/set_speed_percent",
                lambda request, response, rid=robot_id:
                self.on_set_speed_percent(rid, request, response),
                callback_group=self._motion_callbacks,
            ),
            self.create_service(
                SetHold,
                namespace + "/set_hold",
                lambda request, response, rid=robot_id:
                self.on_set_hold(rid, request, response),
                callback_group=self._motion_callbacks,
            ),
            self.create_service(
                TriggerRobotAction,
                namespace + "/trigger_action",
                lambda request, response, rid=robot_id:
                self.on_trigger_action(rid, request, response),
                callback_group=self._motion_callbacks,
            ),
        ])

    @staticmethod
    def _robot_id(instance) -> str:
        for key in ("robot_id", "id", "instance_id"):
            value = str(_value(instance, key, "") or "").strip()
            if value:
                return value
        raise ValueError("PLC instance has no robot id")

    # ---------------------------------------------------------- MC read session
    def poll_plc(self) -> None:
        """Timer callback: a failed connect schedules retry without sleeping."""
        if self._is_running():
            self._refresh_all()

    def _refresh_all(self) -> bool:
        if not self._is_running():
            return False
        observations = {}
        started = time.perf_counter()
        try:
            with self._transport_lock:
                if not self._is_running() or not self._ensure_connected_locked():
                    return False
                read_cache = {}
                for robot_id, runtime in self.runtimes.items():
                    observations[robot_id] = self._read_instance_locked(
                        runtime.instance, read_cache
                    )
        except (OSError, McError) as exc:
            self._transport_failed(exc)
            return False
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            self._mark_all_unavailable(f"invalid PLC register map: {exc}")
            self.get_logger().error(
                f"PLC register-map error: {exc}",
                throttle_duration_sec=5.0,
            )
            return False

        latency_ms = (time.perf_counter() - started) * 1000.0
        contact_wall_s = time.time()
        contact_monotonic_s = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        for robot_id, observation in observations.items():
            runtime = self.runtimes[robot_id]
            with runtime.lock:
                previous_request = runtime.snapshot.requested_speed_percent
                runtime.snapshot = _Snapshot(
                    sequence=runtime.snapshot.sequence + 1,
                    contact_wall_s=contact_wall_s,
                    contact_monotonic_s=contact_monotonic_s,
                    connection_state=RobotStatus.CONNECTION_CONNECTED,
                    requested_speed_percent=previous_request,
                    read_latency_ms=latency_ms,
                    status_message="PLC pose and command registers read",
                    **observation,
                )
                snapshot = replace(runtime.snapshot)
            self._publish_sample(runtime, snapshot, stamp)
        return True

    def _ensure_connected_locked(self) -> bool:
        if self.client.connected:
            return True
        now = time.monotonic()
        if now < self._next_connect_monotonic_s:
            return False
        try:
            self.client.connect()
        except OSError as exc:
            self._transport_failed_locked(exc)
            return False
        # Connecting only opens a transport.  Never replay or synthesize a
        # command here.
        self._next_connect_monotonic_s = 0.0
        return True

    def _read_instance_locked(self, instance, read_cache: dict) -> dict:
        read_head = str(instance.read_head)
        read_words = int(instance.read_words)
        block_key = (read_head, read_words)
        if block_key not in read_cache:
            read_cache[block_key] = self.client.read_words(
                read_head, read_words
            )
        words = read_cache[block_key]

        status_word_name = str(instance.status_word)
        status_key = (status_word_name, 1)
        if status_key not in read_cache:
            read_cache[status_key] = self.client.read_words(
                status_word_name, 1
            )
        status_word = int(read_cache[status_key][0])

        raw = []
        for axis in instance.axes:
            offset = int(axis.offset)
            if str(axis.type).lower() == "dword":
                raw.append(words_to_dword(words[offset], words[offset + 1]))
            else:
                raw.append(int(words[offset]))
        degrees, clamped = instance.to_degrees(raw)

        status_bits = dict(instance.status_bits)
        estop_bit = status_bits.get(
            "emergency_stop", status_bits.get("estop")
        )
        run_offset = instance.run_feedback_offset
        speed_offsets = dict(instance.speed_feedback_offsets)
        speed_1_raw = self._mapped_raw_word(
            words, speed_offsets.get("speed_down_1")
        )
        speed_2_raw = self._mapped_raw_word(
            words, speed_offsets.get("speed_down_2")
        )
        speed_3_raw = self._mapped_raw_word(
            words, speed_offsets.get("speed_down_3")
        )
        return {
            "operation_state": int(words[instance.operation_state_offset]),
            "raw_axes": list(raw),
            "degrees": list(degrees),
            "clamped": bool(clamped),
            "run": (
                bool(words[int(run_offset)])
                if run_offset is not None else None
            ),
            "hold": self._mapped_bit(status_word, status_bits.get("hold")),
            "emergency_stop": self._mapped_bit(status_word, estop_bit),
            "fault_reset": self._mapped_bit(
                status_word, status_bits.get("fault_reset")
            ),
            "device_home": self._mapped_bit(
                status_word, status_bits.get("device_home")
            ),
            "robot_home": self._mapped_bit(
                status_word, status_bits.get("robot_home")
            ),
            "standby": self._mapped_bit(
                status_word, status_bits.get("standby")
            ),
            "control_word_raw": status_word,
            "speed_down_1_raw": int(speed_1_raw or 0),
            "speed_down_2_raw": int(speed_2_raw or 0),
            "speed_down_3_raw": int(speed_3_raw or 0),
            "speed_down_1": bool(speed_1_raw),
            "speed_down_2": bool(speed_2_raw),
            "speed_down_3": bool(speed_3_raw),
        }

    @staticmethod
    def _mapped_bit(word: int, offset) -> Optional[bool]:
        return bit_of(word, int(offset)) if offset is not None else None

    @staticmethod
    def _mapped_raw_word(
        words: Sequence[int], offset
    ) -> Optional[int]:
        return int(words[int(offset)]) if offset is not None else None

    def _transport_failed(self, error: Exception) -> None:
        with self._transport_lock:
            self._transport_failed_locked(error)

    def _transport_failed_locked(self, error: Exception) -> None:
        try:
            self.client.note_failure()
            delay = max(0.0, float(self.client.backoff_delay()))
        except AttributeError:
            self.client.close()
            delay = 1.0
        self._next_connect_monotonic_s = time.monotonic() + delay
        self._mark_all_unavailable(str(error))
        self.get_logger().warn(
            f"PLC transport unavailable: {error}; retry in {delay:g}s",
            throttle_duration_sec=3.0,
        )

    def _mark_all_unavailable(self, message: str) -> None:
        stamp = self.get_clock().now().to_msg()
        for runtime in self.runtimes.values():
            with runtime.lock:
                runtime.snapshot.connection_state = (
                    RobotStatus.CONNECTION_DEGRADED
                    if runtime.snapshot.sequence
                    else RobotStatus.CONNECTION_DISCONNECTED
                )
                runtime.snapshot.status_message = message
                sequence = runtime.snapshot.sequence
            # RobotMemory has no tri-state fields.  Publish all flags false and
            # link_ok=false; RobotStatus is the authoritative UNKNOWN model.
            memory = RobotMemory()
            memory.header.stamp = stamp
            memory.header.frame_id = self._robot_id(runtime.instance)
            memory.link_ok = False
            memory.seq = sequence
            runtime.pub_memory.publish(memory)

    # --------------------------------------------------------------- publishers
    def _publish_sample(self, runtime, snapshot: _Snapshot, stamp) -> None:
        robot_id = self._robot_id(runtime.instance)

        memory = RobotMemory()
        memory.header.stamp = stamp
        memory.header.frame_id = robot_id
        memory.run = bool(snapshot.run)
        memory.hold = bool(snapshot.hold)
        memory.emergency_stop = bool(snapshot.emergency_stop)
        memory.speed_down_1 = bool(snapshot.speed_down_1)
        memory.speed_down_2 = bool(snapshot.speed_down_2)
        memory.speed_down_3 = bool(snapshot.speed_down_3)
        memory.operation_state = int(snapshot.operation_state or 0)
        if len(snapshot.raw_axes) != 6:
            raise ValueError(f"{robot_id}: expected six PLC axes")
        (
            memory.s_axis,
            memory.h_axis,
            memory.v_axis,
            memory.r2_axis,
            memory.b_axis,
            memory.r1_axis,
        ) = snapshot.raw_axes
        memory.link_ok = True
        memory.seq = snapshot.sequence
        memory.read_latency_ms = float(snapshot.read_latency_ms)
        runtime.pub_memory.publish(memory)

        pose = RobotPose()
        pose.header.stamp = stamp
        pose.header.frame_id = robot_id
        pose.robot_id = robot_id
        pose.axis_names = list(runtime.instance.axis_names)
        pose.degrees = list(snapshot.degrees)
        pose.raw = list(snapshot.raw_axes)
        pose.calibrated = bool(runtime.instance.calibrated)
        pose.clamped = bool(snapshot.clamped)
        runtime.pub_pose.publish(pose)
        # Preserve the legacy adapter input as the same controller-coordinate
        # sample.  The PLC-aware Unity adapter owns its CAD conversion, while
        # direct-Hi6 and legacy routes remain identity transforms.
        runtime.pub_pose_unity.publish(pose)

        # RViz consumes CAD joint coordinates directly.
        visual_degrees, _visual_clamped = (
            runtime.instance.to_visual_degrees(snapshot.degrees)
        )

        joint = JointState()
        joint.header.stamp = stamp
        joint.header.frame_id = robot_id
        joint.name = [f"joint_{index}" for index in range(1, 7)]
        joint.position = [math.radians(value) for value in visual_degrees]
        runtime.pub_joint.publish(joint)

    def publish_statuses(self) -> None:
        if not self._is_running():
            return
        for robot_id, runtime in self.runtimes.items():
            runtime.pub_status.publish(self._build_status(robot_id))
            runtime.pub_control.publish(self._build_control_state(robot_id))

    def _build_control_state(self, robot_id: str) -> RobotControlState:
        """Return raw No.9--17 command-register readback.

        The message intentionally does not call these values robot feedback:
        reading back a command register only proves what is stored in the PLC.
        """
        runtime = self.runtimes[robot_id]
        with runtime.lock:
            snapshot = replace(runtime.snapshot)
        message = RobotControlState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = robot_id
        message.robot_id = robot_id
        message.sequence = snapshot.sequence
        age = self._snapshot_age(snapshot)
        message.fresh = self._snapshot_is_fresh(snapshot)
        message.age_sec = float(age if math.isfinite(age) else -1.0)
        message.slowdown_25_raw = int(snapshot.speed_down_1_raw)
        message.slowdown_50_raw = int(snapshot.speed_down_2_raw)
        message.slowdown_75_raw = int(snapshot.speed_down_3_raw)
        message.control_word_raw = int(snapshot.control_word_raw)

        direct = runtime.instance.direct_controls
        speed = dict(direct.speed)
        message.slowdown_25_active = (
            message.slowdown_25_raw
            == int(speed["speed_25"].active_value)
        )
        message.slowdown_50_active = (
            message.slowdown_50_raw
            == int(speed["speed_50"].active_value)
        )
        message.slowdown_75_active = (
            message.slowdown_75_raw
            == int(speed["speed_75"].active_value)
        )
        message.hold = bool(snapshot.hold)
        message.emergency_stop = bool(snapshot.emergency_stop)
        message.fault_reset = bool(snapshot.fault_reset)
        message.device_home = bool(snapshot.device_home)
        message.robot_home = bool(snapshot.robot_home)
        message.standby = bool(snapshot.standby)
        return message

    def _build_status(self, robot_id: str) -> RobotStatus:
        runtime = self.runtimes[robot_id]
        with runtime.lock:
            snapshot = replace(runtime.snapshot)
        message = RobotStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = robot_id
        message.robot_id = robot_id
        message.sequence = snapshot.sequence

        age = self._snapshot_age(snapshot)
        fresh = bool(
            snapshot.sequence
            and snapshot.connection_state == RobotStatus.CONNECTION_CONNECTED
            and math.isfinite(age)
            and age <= self.stale_timeout_s
        )
        message.fresh = fresh
        message.age_sec = float(age if math.isfinite(age) else -1.0)
        message.connection_state = snapshot.connection_state
        if (
            message.connection_state == RobotStatus.CONNECTION_CONNECTED
            and not fresh
        ):
            message.connection_state = RobotStatus.CONNECTION_DEGRADED
        if snapshot.contact_wall_s > 0.0:
            seconds = int(snapshot.contact_wall_s)
            message.last_controller_contact.sec = seconds
            message.last_controller_contact.nanosec = int(
                (snapshot.contact_wall_s - seconds) * 1_000_000_000
            )

        # The supplied PLC contract has no documented auto/manual/motor/fault
        # feedback.  Unknown is safer than inferring those states from D1000.
        message.operation_mode = RobotStatus.OPERATION_UNKNOWN
        message.motor_power_state = RobotStatus.MOTOR_UNKNOWN
        message.protective_stop_state = RobotStatus.SIGNAL_UNKNOWN
        message.fault_state = RobotStatus.SIGNAL_UNKNOWN
        message.requested_speed_valid = (
            snapshot.requested_speed_percent is not None
        )
        if snapshot.requested_speed_percent is not None:
            message.requested_speed_percent = float(
                snapshot.requested_speed_percent
            )

        if not fresh:
            message.execution_state = RobotStatus.EXECUTION_UNKNOWN
            message.emergency_stop_state = RobotStatus.SIGNAL_UNKNOWN
            message.actual_speed_valid = False
            message.status_message = (
                f"PLC feedback unavailable/stale: {snapshot.status_message}"
            )
            return message

        # D1014/D1016/D1018/D1020 and D1100 are the reviewed command-memory
        # allocation.  Echoing those words does not prove robot execution,
        # actual speed, or a safety-rated E-stop.  Raw values are published on
        # /control_state while the canonical actual fields remain UNKNOWN.
        message.emergency_stop_state = RobotStatus.SIGNAL_UNKNOWN
        message.execution_state = RobotStatus.EXECUTION_UNKNOWN
        message.actual_speed_valid = False
        message.status_message = (
            "fresh PLC pose and command-register readback; robot actual "
            "execution/speed/safety feedback is not mapped"
        )
        return message

    @staticmethod
    def _actual_speed(snapshot: _Snapshot) -> Optional[int]:
        flags = (
            snapshot.speed_down_1,
            snapshot.speed_down_2,
            snapshot.speed_down_3,
        )
        if any(value is None for value in flags):
            return None
        active = [index for index, value in enumerate(flags) if value]
        if len(active) > 1:
            return None
        return (75, 50, 25)[active[0]] if active else 100

    @staticmethod
    def _signal(value: Optional[bool]) -> int:
        if value is None:
            return RobotStatus.SIGNAL_UNKNOWN
        return (
            RobotStatus.SIGNAL_ACTIVE
            if value else RobotStatus.SIGNAL_INACTIVE
        )

    @staticmethod
    def _snapshot_age(snapshot: _Snapshot) -> float:
        if snapshot.contact_monotonic_s is None:
            return math.inf
        return max(0.0, time.monotonic() - snapshot.contact_monotonic_s)

    # ------------------------------------------------------------ read service
    def on_get_status(self, robot_id: str, request, response):
        controller_ack = False
        if request.force_controller_read:
            controller_ack = self._refresh_all()
        actual = self._build_status(robot_id)
        max_age = float(request.max_age_sec)
        if max_age <= 0.0:
            max_age = self.stale_timeout_s
        response.success = bool(
            actual.sequence
            and actual.age_sec >= 0.0
            and actual.age_sec <= max_age
            and actual.connection_state == RobotStatus.CONNECTION_CONNECTED
        )
        response.controller_ack = controller_ack
        response.from_cache = (
            not request.force_controller_read or not controller_ack
        )
        response.fresh = actual.fresh
        response.actual = actual
        response.error_code = "" if response.success else "STATUS_STALE"
        response.message = (
            "fresh actual PLC feedback"
            if response.success
            else "no fresh actual PLC feedback is available"
        )
        return response

    # ----------------------------------------------------------- write services
    def on_request_stop(self, robot_id: str, request, response):
        """Write the normal process-stop word (D2002 in the reviewed map)."""
        response.request_id = self._request_id(request.request_id)
        runtime = self.runtimes[robot_id]
        error = self._command_policy_error(runtime, "stop")
        if error:
            return self._reject(response, robot_id, *error)

        self._mark_stop_requested()
        response.accepted = True
        block = runtime.instance.motion_command
        values = self._one_hot_block(block, "stop")
        if not self._write_command(block.device, values):
            return self._write_failed(response, robot_id)
        response.controller_ack = True

        if runtime.instance.run_feedback_offset is None:
            return self._feedback_unavailable(response, robot_id)
        response.confirmed = self._verify(
            robot_id,
            lambda snapshot: snapshot.run is False,
            self._confirmation_timeout(request.confirmation_timeout_sec),
        )
        response.actual = self._build_status(robot_id)
        if response.confirmed:
            response.message = (
                "PLC accepted process-stop write and actual run feedback is OFF"
            )
        else:
            response.error_code = "CONFIRMATION_TIMEOUT"
            response.message = (
                "PLC accepted process-stop write, but actual run feedback did "
                "not become OFF; this is not an emergency stop"
            )
        self._audit("PROCESS_STOP", request, response)
        return response

    def on_request_start(self, robot_id: str, request, response):
        response.request_id = self._request_id(request.request_id)
        runtime = self.runtimes[robot_id]
        error = self._command_policy_error(runtime, "run", require_start=True)
        if error:
            return self._reject(response, robot_id, *error)

        stop_generation = self._current_stop_generation()
        if not self._fresh_feedback(robot_id, refresh=True):
            return self._reject(
                response, robot_id, "STATUS_STALE",
                "fresh PLC safety/status feedback is required before start",
            )
        with runtime.lock:
            snapshot = replace(runtime.snapshot)
        if snapshot.emergency_stop is not False:
            return self._reject(
                response, robot_id, "ESTOP_NOT_CLEAR",
                "PLC emergency-stop feedback is not confirmed clear",
            )
        if snapshot.hold is not False:
            return self._reject(
                response, robot_id, "HOLD_NOT_CLEAR",
                "PLC hold feedback is not confirmed clear",
            )

        response.accepted = True
        block = runtime.instance.motion_command
        values = self._one_hot_block(block, "run")
        if not self._write_command(
            block.device, values, stop_generation=stop_generation
        ):
            if self._stop_was_requested_after(stop_generation):
                return self._preempted(response, robot_id, "start")
            return self._write_failed(response, robot_id)
        response.controller_ack = True
        response.confirmed = self._verify(
            robot_id,
            lambda snapshot: snapshot.run is True,
            self._confirmation_timeout(request.confirmation_timeout_sec),
            stop_generation=stop_generation,
        )
        response.actual = self._build_status(robot_id)
        if response.confirmed:
            response.message = "PLC run write and actual run feedback confirmed"
        elif self._stop_was_requested_after(stop_generation):
            return self._preempted(response, robot_id, "start")
        else:
            response.error_code = "CONFIRMATION_TIMEOUT"
            response.message = (
                "PLC accepted run write, but actual run feedback was not confirmed"
            )
        self._audit("START", request, response)
        return response

    def on_set_speed_percent(self, robot_id: str, request, response):
        response.request_id = self._request_id(request.request_id)
        runtime = self.runtimes[robot_id]
        requested = float(request.speed_percent)
        matched = next(
            (
                value for value in self.allowed_speeds
                if abs(requested - value) < 1e-6
            ),
            None,
        )
        if matched is None:
            return self._reject(
                response,
                robot_id,
                "INVALID_SPEED",
                f"allowed PLC speed limits are {list(self.allowed_speeds)} percent",
            )
        # Every request writes all three non-contiguous registers so stale
        # slowdown selections are cleared.  Therefore all three addresses,
        # not just the selected one, must be allowlisted.
        control_names = ("speed_25", "speed_50", "speed_75")
        for control_name in control_names:
            error = self._direct_control_policy_error(runtime, control_name)
            if error:
                return self._reject(response, robot_id, *error)
        if not self._fresh_feedback(robot_id, refresh=True):
            return self._reject(
                response, robot_id, "STATUS_STALE",
                "fresh PLC feedback is required before changing speed limit",
            )

        direct = runtime.instance.direct_controls
        speed_controls = dict(direct.speed)
        writes = []
        for percent in (25, 50, 75):
            name = f"speed_{percent}"
            item = speed_controls[name]
            value = int(item.active_value) if matched == percent else 0
            writes.append((str(item.device), value))
        stop_generation = self._current_stop_generation()
        response.accepted = True
        # D1016/D1018/D1020 are non-contiguous WORD devices.  A random-word
        # write changes only those three addresses and cannot overwrite the
        # gap words D1017/D1019.
        if not self._write_random_command(
            writes, stop_generation=stop_generation
        ):
            if self._stop_was_requested_after(stop_generation):
                return self._preempted(response, robot_id, "speed")
            return self._write_failed(response, robot_id)
        response.controller_ack = True
        with runtime.lock:
            runtime.snapshot.requested_speed_percent = float(matched)

        register_readback = self._verify(
            robot_id,
            lambda snapshot: self._speed_registers_match(
                runtime.instance, snapshot, matched
            ),
            self._confirmation_timeout(request.confirmation_timeout_sec),
            stop_generation=stop_generation,
        )
        # The reviewed addresses are command/register memory, not independent
        # robot actual-speed feedback.  Keep ``confirmed`` false even when the
        # PLC register echo matches exactly.
        response.confirmed = False
        response.actual = self._build_status(robot_id)
        if register_readback:
            response.error_code = "ACTUAL_FEEDBACK_UNAVAILABLE"
            response.message = (
                f"PLC stored the {matched}% request and register readback "
                "matched; robot actual speed feedback is not mapped"
            )
        elif self._stop_was_requested_after(stop_generation):
            return self._preempted(response, robot_id, "speed")
        else:
            response.error_code = "REGISTER_READBACK_TIMEOUT"
            response.message = (
                "PLC acknowledged the speed request, but the command registers "
                "did not read back as requested"
            )
        self._audit("SPEED", request, response)
        return response

    def on_set_hold(self, robot_id: str, request, response):
        """Set or clear D1100.0 while preserving every other D1100 bit."""
        response.request_id = self._request_id(request.request_id)
        runtime = self.runtimes[robot_id]
        error = self._direct_control_policy_error(runtime, "hold")
        if error:
            return self._reject_control(response, robot_id, *error)
        if not bool(request.hold) and not self._fresh_feedback(
            robot_id, refresh=True
        ):
            return self._reject_control(
                response,
                robot_id,
                "STATUS_STALE",
                "fresh PLC readback is required before releasing Hold",
            )

        response.accepted = True
        ack, readback = self._write_control_bit(
            runtime, "hold", bool(request.hold)
        )
        response.controller_ack = ack
        response.register_readback = readback
        self._refresh_all()
        response.control_state = self._build_control_state(robot_id)
        if not ack:
            response.error_code = "PLC_WRITE_FAILED"
            response.message = "no successful MC write response was received"
        elif not readback:
            response.error_code = "REGISTER_READBACK_MISMATCH"
            response.message = (
                "PLC acknowledged Hold, but D1100.0 did not read back as requested"
            )
        else:
            response.message = (
                f"D1100.0 Hold register readback is {bool(request.hold)}; "
                "robot motion completion is not independently confirmed"
            )
        self._audit_control("HOLD", request, response)
        return response

    def on_trigger_action(self, robot_id: str, request, response):
        """Pulse one approved D1100 action bit, then always attempt to clear it."""
        response.request_id = self._request_id(request.request_id)
        action_names = {
            int(TriggerRobotAction.Request.ACTION_FAULT_RESET): "fault_reset",
            int(TriggerRobotAction.Request.ACTION_DEVICE_HOME): "device_home",
            int(TriggerRobotAction.Request.ACTION_ROBOT_HOME): "robot_home",
            int(TriggerRobotAction.Request.ACTION_STANDBY): "standby",
        }
        action_name = action_names.get(int(request.action))
        if action_name is None:
            return self._reject_control(
                response,
                robot_id,
                "INVALID_ACTION",
                "allowed actions are fault reset, device home, robot home, and standby",
            )
        runtime = self.runtimes[robot_id]
        error = self._direct_control_policy_error(runtime, action_name)
        if error:
            return self._reject_control(response, robot_id, *error)
        if not self._fresh_feedback(robot_id, refresh=True):
            return self._reject_control(
                response,
                robot_id,
                "STATUS_STALE",
                "fresh PLC command-register readback is required",
            )
        with runtime.lock:
            snapshot = replace(runtime.snapshot)
        if snapshot.emergency_stop is not False:
            return self._reject_control(
                response,
                robot_id,
                "STOP_BIT_NOT_CLEAR",
                "D1100.1 stop/E-stop command bit is not confirmed clear",
            )

        response.accepted = True
        set_ack = False
        set_readback = False
        clear_ack = False
        clear_readback = False
        try:
            set_ack, set_readback = self._write_control_bit(
                runtime, action_name, True
            )
            if set_ack:
                pulse_seconds = float(
                    runtime.instance.direct_controls.action_pulse_seconds
                )
                self._shutting_down.wait(pulse_seconds)
        finally:
            # Clearing uses a fresh read-modify-write so unrelated D1100 bits
            # changed by the PLC during the pulse are retained.
            if set_ack:
                clear_ack, clear_readback = self._write_control_bit(
                    runtime,
                    action_name,
                    False,
                    allow_during_shutdown=True,
                )

        response.controller_ack = bool(set_ack and clear_ack)
        response.register_readback = bool(set_readback and clear_readback)
        self._refresh_all()
        response.control_state = self._build_control_state(robot_id)
        if not set_ack:
            response.error_code = "PLC_WRITE_FAILED"
            response.message = "PLC did not acknowledge the action pulse assertion"
        elif not clear_ack:
            response.error_code = "PULSE_CLEAR_FAILED"
            response.message = (
                "PLC accepted the action assertion, but pulse clear was not acknowledged"
            )
        elif not response.register_readback:
            response.error_code = "REGISTER_READBACK_MISMATCH"
            response.message = (
                "action pulse write completed, but assert/clear register readback mismatched"
            )
        else:
            response.message = (
                f"{action_name} register pulse was asserted and cleared; "
                "robot action completion is not independently confirmed"
            )
        self._audit_control(action_name.upper(), request, response)
        return response

    def _direct_control_policy_error(
        self, runtime: _Runtime, control_name: str
    ) -> Optional[tuple[str, str]]:
        direct = runtime.instance.direct_controls
        if not bool(runtime.instance.command_map_verified):
            return (
                "COMMAND_MAP_UNVERIFIED",
                "No.9-17 PLC command-register map is not verified",
            )
        if control_name == "emergency_stop":
            return (
                "ESTOP_READ_ONLY",
                "D1100.1 is permanently read-only; use the physical safety system",
            )
        if control_name not in direct.writable_controls:
            return (
                "CONTROL_NOT_WRITABLE",
                f"PLC control {control_name!r} is not in the configured allowlist",
            )
        if self.profile == "field" and not self.allow_field_control_writes:
            return (
                "FIELD_WRITE_OPT_IN_REQUIRED",
                "field PLC writes require allow_field_control_writes:=true at launch",
            )
        return None

    @staticmethod
    def _speed_registers_match(instance, snapshot: _Snapshot, percent: int) -> bool:
        speed = dict(instance.direct_controls.speed)
        actual = {
            25: int(snapshot.speed_down_1_raw),
            50: int(snapshot.speed_down_2_raw),
            75: int(snapshot.speed_down_3_raw),
        }
        for candidate in (25, 50, 75):
            expected = (
                int(speed[f"speed_{candidate}"].active_value)
                if candidate == percent else 0
            )
            if actual[candidate] != expected:
                return False
        return True

    def _write_random_command(
        self,
        writes,
        *,
        stop_generation: Optional[int] = None,
    ) -> bool:
        try:
            with self._transport_lock:
                if not self._is_running():
                    return False
                if (
                    stop_generation is not None
                    and self._stop_was_requested_after(stop_generation)
                ):
                    return False
                if not self._ensure_connected_locked():
                    return False
                self.client.write_random_words(writes)
                return True
        except (OSError, McError, ValueError) as exc:
            self._transport_failed(exc)
            return False

    def _write_control_bit(
        self,
        runtime: _Runtime,
        bit_name: str,
        enabled: bool,
        *,
        allow_during_shutdown: bool = False,
    ) -> tuple[bool, bool]:
        """Masked D-word update with same-session register readback."""
        write_ack = False
        try:
            with self._transport_lock:
                if not allow_during_shutdown and not self._is_running():
                    return False, False
                if not self.client.connected:
                    if allow_during_shutdown:
                        return False, False
                    if not self._ensure_connected_locked():
                        return False, False
                if not self.client.connected:
                    return False, False
                direct = runtime.instance.direct_controls
                device = str(direct.control_word_device)
                bit = int(direct.control_bits[bit_name])
                mask = 1 << bit
                current = int(self.client.read_words(device, 1)[0])
                requested = (
                    current | mask if enabled else current & (~mask & 0xFFFF)
                )
                self.client.write_words(device, [requested])
                write_ack = True
                readback = int(self.client.read_words(device, 1)[0])
                return write_ack, bool(readback & mask) == bool(enabled)
        except (OSError, McError, KeyError, ValueError) as exc:
            self._transport_failed(exc)
            return write_ack, False

    def _command_policy_error(
        self, runtime: _Runtime, field_name: str, *, require_start: bool = False
    ) -> Optional[tuple[str, str]]:
        if not bool(self.cfg.commissioned):
            return (
                "UNCOMMISSIONED",
                "PLC profile/register map is not commissioned",
            )
        if not bool(self.cfg.commands_enabled):
            return "COMMANDS_DISABLED", "PLC command services are disabled"
        if require_start and not bool(self.cfg.start_enabled):
            return "START_DISABLED", "PLC remote start is separately disabled"
        if not bool(runtime.instance.map_verified):
            return "MAP_UNVERIFIED", "robot PLC register map is not verified"
        block = (
            runtime.instance.speed_command
            if field_name == "speed"
            else runtime.instance.motion_command
        )
        if block is None:
            return "COMMAND_UNMAPPED", "requested PLC command is not mapped"
        if field_name != "speed" and field_name not in dict(block.fields):
            return (
                "COMMAND_UNMAPPED",
                f"PLC command field {field_name!r} is not mapped",
            )
        return None

    @staticmethod
    def _one_hot_block(block, field_name: str) -> List[int]:
        values = [0] * int(block.words)
        fields = dict(block.fields)
        offset = int(fields[field_name])
        if offset < 0 or offset >= len(values):
            raise ValueError(f"command field {field_name} is outside block")
        values[offset] = 1
        return values

    def _write_command(
        self,
        device: str,
        values: Sequence[int],
        *,
        stop_generation: Optional[int] = None,
    ) -> bool:
        try:
            with self._transport_lock:
                if not self._is_running():
                    return False
                if (
                    stop_generation is not None
                    and self._stop_was_requested_after(stop_generation)
                ):
                    return False
                if not self._ensure_connected_locked():
                    return False
                self.client.write_words(str(device), list(values))
                return True
        except (OSError, McError) as exc:
            self._transport_failed(exc)
            return False

    def _verify(
        self,
        robot_id: str,
        predicate,
        timeout_s: float,
        *,
        stop_generation: Optional[int] = None,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._is_running() and time.monotonic() <= deadline:
            if (
                stop_generation is not None
                and self._stop_was_requested_after(stop_generation)
            ):
                return False
            if self._refresh_all():
                runtime = self.runtimes[robot_id]
                with runtime.lock:
                    snapshot = replace(runtime.snapshot)
                if self._snapshot_is_fresh(snapshot) and predicate(snapshot):
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._shutting_down.wait(min(0.05, remaining))
        return False

    def _fresh_feedback(self, robot_id: str, *, refresh: bool) -> bool:
        runtime = self.runtimes[robot_id]
        with runtime.lock:
            snapshot = replace(runtime.snapshot)
        if self._snapshot_is_fresh(snapshot):
            return True
        if refresh and self._refresh_all():
            with runtime.lock:
                snapshot = replace(runtime.snapshot)
            return self._snapshot_is_fresh(snapshot)
        return False

    def _snapshot_is_fresh(self, snapshot: _Snapshot) -> bool:
        return bool(
            snapshot.sequence
            and snapshot.connection_state == RobotStatus.CONNECTION_CONNECTED
            and self._snapshot_age(snapshot) <= self.stale_timeout_s
        )

    def _feedback_unavailable(self, response, robot_id: str):
        response.confirmed = False
        response.actual = self._build_status(robot_id)
        response.error_code = "FEEDBACK_UNAVAILABLE"
        response.message = (
            "PLC transport acknowledged the write, but commissioned actual "
            "feedback is not mapped"
        )
        return response

    def _write_failed(self, response, robot_id: str):
        response.controller_ack = False
        response.confirmed = False
        response.actual = self._build_status(robot_id)
        response.error_code = "PLC_WRITE_FAILED"
        response.message = (
            "request passed local policy, but no successful MC write response "
            "was received"
        )
        return response

    def _preempted(self, response, robot_id: str, command: str):
        response.confirmed = False
        response.actual = self._build_status(robot_id)
        response.error_code = "PREEMPTED_BY_STOP"
        response.message = f"{command} was superseded by a newer stop request"
        return response

    def _reject(self, response, robot_id: str, code: str, message: str):
        response.accepted = False
        response.controller_ack = False
        response.confirmed = False
        response.actual = self._build_status(robot_id)
        response.error_code = code
        response.message = message
        return response

    def _reject_control(
        self, response, robot_id: str, code: str, message: str
    ):
        response.accepted = False
        response.controller_ack = False
        response.register_readback = False
        response.control_state = self._build_control_state(robot_id)
        response.error_code = code
        response.message = message
        return response

    def _confirmation_timeout(self, requested: float) -> float:
        value = float(requested)
        return min(value if value > 0.0 else self.verify_timeout_s, 30.0)

    @staticmethod
    def _request_id(requested: str) -> str:
        return str(requested).strip() or str(uuid.uuid4())

    def _mark_stop_requested(self) -> int:
        with self._stop_generation_lock:
            self._stop_generation += 1
            return self._stop_generation

    def _current_stop_generation(self) -> int:
        with self._stop_generation_lock:
            return self._stop_generation

    def _stop_was_requested_after(self, generation: int) -> bool:
        return self._current_stop_generation() != generation

    def _audit(self, command: str, request, response) -> None:
        self.get_logger().info(
            "command=%s id=%s source=%s accepted=%s mc_ack=%s confirmed=%s "
            "reason=%s"
            % (
                command,
                response.request_id,
                request.source or "-",
                response.accepted,
                response.controller_ack,
                response.confirmed,
                request.reason or "-",
            )
        )

    def _audit_control(self, command: str, request, response) -> None:
        self.get_logger().info(
            "command=%s id=%s source=%s accepted=%s mc_ack=%s "
            "register_readback=%s reason=%s"
            % (
                command,
                response.request_id,
                request.source or "-",
                response.accepted,
                response.controller_ack,
                response.register_readback,
                request.reason or "-",
            )
        )

    def _is_running(self) -> bool:
        return (
            not self._shutting_down.is_set()
            and rclpy.ok(context=self.context)
        )

    def destroy_node(self) -> bool:
        self._shutting_down.set()
        with self._transport_lock:
            self.client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlcGatewayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
