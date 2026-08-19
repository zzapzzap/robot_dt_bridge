"""
ROS 2 adapter for one HD Hyundai Hi6 controller.

The node is deliberately separate from ``robot_memory_node``: it never speaks
MELSEC MC protocol and never writes anything during startup or reconnection.
Continuous controller readback is published as topics; start, normal stop and
playback-speed changes are explicit services with independent acknowledgement
and readback-confirmation fields.

This is a supervisory, non-safety-rated interface.  It does not replace the
robot safety controller, safety PLC, guards, or a hard-wired emergency stop.
"""

from __future__ import annotations

import math
import socket
import threading
import time
import uuid
from dataclasses import dataclass, replace
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from robot_bridge_msgs.msg import RobotPose, RobotStatus
from robot_bridge_msgs.srv import (
    GetRobotStatus,
    RequestStart,
    RequestStop,
    SetSpeedPercent,
)

from .config_loader import Hi6Config as Hi6BridgeConfig
from .hi6_client import (
    Hi6Client,
    Hi6Config as Hi6HttpConfig,
    Hi6Error,
)


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class _Snapshot:
    sequence: int = 0
    contact_wall_s: float = 0.0
    contact_monotonic_s: Optional[float] = None
    connection_state: int = RobotStatus.CONNECTION_CONNECTING
    mode_code: Optional[int] = None
    motor_state_code: Optional[int] = None
    is_playback: Optional[bool] = None
    is_remote_mode: Optional[bool] = None
    speed_percent: Optional[float] = None
    requested_speed_percent: Optional[float] = None
    emergency_stop: Optional[bool] = None
    last_error_id: Optional[int] = None
    status_message: str = "controller status has not been read yet"


class Hi6RobotNode(Node):
    """One isolated ROS process/session per robot controller."""

    def __init__(self) -> None:
        super().__init__("hi6_robot_node")

        self.declare_parameter("config_dir", "")
        self.declare_parameter("robot_id", "loading")
        config_dir = str(self.get_parameter("config_dir").value or "") or None
        robot_id = str(self.get_parameter("robot_id").value)

        bridge_config = Hi6BridgeConfig.load(config_dir)
        configured = bridge_config.robot(robot_id)
        network_hosts = bridge_config.network.get("hosts") or {}
        configured_source_address = (
            str(network_hosts.get("jetson") or "").strip()
            if isinstance(network_hosts, dict)
            else ""
        )

        self.declare_parameter("host", configured.host)
        self.declare_parameter("rest_port", configured.rest_port)
        self.declare_parameter("source_address", configured_source_address)
        self.declare_parameter("pose_hz", configured.pose_hz)
        self.declare_parameter("status_hz", configured.status_hz)
        self.declare_parameter(
            "status_publish_hz", configured.status_publish_hz
        )
        self.declare_parameter("stale_timeout_ms", configured.stale_timeout_ms)
        self.declare_parameter("verify_timeout_s", configured.verify_timeout_s)
        self.declare_parameter("allow_commands", configured.allow_commands)
        self.declare_parameter(
            "allow_speed_increase", configured.allow_speed_increase
        )
        self.declare_parameter("allow_start", configured.allow_start)
        self.declare_parameter(
            "allow_unverified_start", configured.allow_unverified_start
        )

        self.robot_id = robot_id
        self.host = str(self.get_parameter("host").value)
        self.rest_port = int(self.get_parameter("rest_port").value)
        self.source_address = (
            str(self.get_parameter("source_address").value or "").strip()
            or None
        )
        self.pose_hz = float(self.get_parameter("pose_hz").value)
        self.status_hz = float(self.get_parameter("status_hz").value)
        self.status_publish_hz = float(
            self.get_parameter("status_publish_hz").value
        )
        self.stale_timeout_s = (
            float(self.get_parameter("stale_timeout_ms").value) / 1000.0
        )
        self.verify_timeout_s = float(self.get_parameter("verify_timeout_s").value)
        self.allow_commands = bool(self.get_parameter("allow_commands").value)
        self.allow_speed_increase = bool(
            self.get_parameter("allow_speed_increase").value
        )
        self.allow_start = bool(self.get_parameter("allow_start").value)
        self.allow_unverified_start = bool(
            self.get_parameter("allow_unverified_start").value
        )
        self.allowed_speeds = tuple(configured.allowed_speed_percent)
        self.supported_api_versions = tuple(configured.supported_api_versions)
        self.axis_names = tuple(configured.axis_names)
        self.joint_names = tuple(configured.joint_names)

        if (
            self.pose_hz <= 0.0
            or self.status_hz <= 0.0
            or self.status_publish_hz <= 0.0
        ):
            raise ValueError(
                "pose_hz, status_hz and status_publish_hz must be greater "
                "than zero"
            )
        if self.stale_timeout_s <= 0.0 or self.verify_timeout_s <= 0.0:
            raise ValueError("stale/verification timeouts must be greater than zero")

        http_config = Hi6HttpConfig(
            host=self.host,
            port=self.rest_port,
            source_address=self.source_address,
            connect_timeout_s=configured.connect_timeout_s,
            read_timeout_s=configured.read_timeout_s,
        )
        # Pose, status and commands deliberately use independent persistent
        # connections.  A slow three-GET status transaction must not block the
        # 20 Hz pose sampler, while command ordering remains isolated from both.
        self.pose_client = Hi6Client(http_config)
        self.status_client = Hi6Client(http_config)
        self.command_client = Hi6Client(http_config)
        # A safe GET can retry once.  Shutdown waits long enough for the one
        # request already on the wire to finish, while queued reads re-check
        # the shutdown flag under its transport lock and do not start.
        self.shutdown_timeout_s = max(
            2.0,
            2.0 * (configured.connect_timeout_s + configured.read_timeout_s) + 1.0,
        )
        self.snapshot = _Snapshot()
        self._shutting_down = threading.Event()
        self.context.on_shutdown(self._shutting_down.set)
        self._snapshot_lock = threading.RLock()
        self._status_refresh_lock = threading.Lock()
        self._pose_transport_lock = threading.Lock()
        self._status_transport_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._api_probe_lock = threading.Lock()
        self._stop_generation_lock = threading.Lock()
        self._stop_generation = 0
        self._api_version: Optional[int] = None
        self._observed_api_version: Optional[int] = None

        # Pose sampling, controller-status polling, and cached-status publishing
        # are independent.  In particular, a status timeout cannot stop age and
        # freshness metadata from continuing to update at the ROS boundary.
        self._pose_callbacks = MutuallyExclusiveCallbackGroup()
        self._status_callbacks = MutuallyExclusiveCallbackGroup()
        self._status_publish_callbacks = MutuallyExclusiveCallbackGroup()
        self._motion_callbacks = MutuallyExclusiveCallbackGroup()
        # A prior stop may spend seconds confirming readback.  Keep the stop
        # group reentrant so a repeated stop is still dispatched immediately;
        # _write_lock serializes the actual HTTP command transports.
        self._stop_callbacks = ReentrantCallbackGroup()

        namespace = f"/robot/{self.robot_id}"
        self.pub_joint = self.create_publisher(
            JointState, namespace + "/joint_states", SENSOR_QOS
        )
        self.pub_pose = self.create_publisher(
            RobotPose, namespace + "/pose", SENSOR_QOS
        )
        # Keep the existing Unity adapter contract: it consumes cmd_degs_raw.
        self.pub_pose_unity = self.create_publisher(
            RobotPose, namespace + "/cmd_degs_raw", SENSOR_QOS
        )
        self.pub_status = self.create_publisher(
            RobotStatus, namespace + "/status", SENSOR_QOS
        )

        self.srv_status = self.create_service(
            GetRobotStatus,
            namespace + "/get_status",
            self.on_get_status,
            callback_group=self._status_callbacks,
        )
        self.srv_stop = self.create_service(
            RequestStop,
            namespace + "/request_stop",
            self.on_request_stop,
            callback_group=self._stop_callbacks,
        )
        self.srv_start = self.create_service(
            RequestStart,
            namespace + "/request_start",
            self.on_request_start,
            callback_group=self._motion_callbacks,
        )
        self.srv_speed = self.create_service(
            SetSpeedPercent,
            namespace + "/set_speed_percent",
            self.on_set_speed_percent,
            callback_group=self._motion_callbacks,
        )

        self.create_timer(
            1.0 / self.pose_hz,
            self.poll_pose,
            callback_group=self._pose_callbacks,
        )
        self.create_timer(
            1.0 / self.status_hz,
            self.poll_status,
            callback_group=self._status_callbacks,
        )
        self.create_timer(
            1.0 / self.status_publish_hz,
            self.publish_status,
            callback_group=self._status_publish_callbacks,
        )

        mode = "COMMANDS ENABLED" if self.allow_commands else "READ ONLY"
        start = "start enabled" if self.allow_start else "start disabled"
        speed_increase = (
            "speed increase enabled"
            if self.allow_speed_increase
            else "speed increase disabled"
        )
        self.get_logger().info(
            f"Hi6 direct {self.robot_id}: {self.host}:{self.rest_port} "
            f"source={self.source_address or 'automatic'} "
            f"pose={self.pose_hz:g}Hz status-poll={self.status_hz:g}Hz "
            f"status-publish={self.status_publish_hz:g}Hz "
            f"[{mode}, {speed_increase}, {start}]"
        )
        if self.allow_commands:
            self.get_logger().warn(
                "Hi6 command services are enabled. They are not safety-rated controls."
            )

    # --------------------------------------------------------- continuous reads
    def poll_pose(self) -> None:
        if not self._is_running():
            return
        try:
            with self._pose_transport_lock:
                if not self._is_running():
                    return
                degrees = list(
                    self.pose_client.get_joint_positions(6, mechinfo=1)
                )
        except Hi6Error as exc:
            if not self._is_running():
                return
            self.get_logger().warn(
                f"Hi6 pose read failed: {exc}", throttle_duration_sec=3.0
            )
            return
        if not self._is_running():
            return

        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.header.frame_id = self.robot_id
        joint.name = list(self.joint_names)
        joint.position = [math.radians(value) for value in degrees]
        self._publish_if_running(self.pub_joint, joint)

        pose = RobotPose()
        pose.header.stamp = stamp
        pose.header.frame_id = self.robot_id
        pose.robot_id = self.robot_id
        pose.axis_names = list(self.axis_names)
        pose.degrees = degrees
        pose.raw = []
        pose.calibrated = True
        pose.clamped = False
        self._publish_if_running(self.pub_pose, pose)
        self._publish_if_running(self.pub_pose_unity, pose)

    def poll_status(self) -> None:
        if not self._is_running():
            return
        self._refresh_status()

    def publish_status(self) -> None:
        """Publish cached controller status with live age/freshness metadata."""
        if not self._is_running():
            return
        self._publish_if_running(self.pub_status, self._build_status_msg())

    def _refresh_status(self) -> bool:
        with self._status_refresh_lock:
            return self._refresh_status_locked()

    def _refresh_status_locked(self) -> bool:
        if not self._is_running():
            return False
        self._probe_api_version()
        if not self._is_running():
            return False
        batch_wall_s = time.time()
        batch_monotonic_s = time.monotonic()
        try:
            with self._status_transport_lock:
                if not self._is_running():
                    return False
                state = self.status_client.get_status()
                if not self._is_running():
                    return False
                motor_state = self.status_client.get_motor_state()
                if not self._is_running():
                    return False
                emergency_stop = self.status_client.get_emergency_stop()
        except Hi6Error as exc:
            with self._snapshot_lock:
                self.snapshot.connection_state = (
                    RobotStatus.CONNECTION_DEGRADED
                    if self.snapshot.sequence
                    else RobotStatus.CONNECTION_DISCONNECTED
                )
                self.snapshot.status_message = str(exc)
            self.status_client.close()
            if not self._is_running():
                return False
            self.get_logger().warn(
                f"Hi6 status read failed: {exc}", throttle_duration_sec=3.0
            )
            return False
        if not self._is_running():
            return False

        batch_age_s = time.monotonic() - batch_monotonic_s
        slow_batch = batch_age_s > self.stale_timeout_s
        with self._snapshot_lock:
            self.snapshot.sequence += 1
            # Use the oldest observation in this multi-request snapshot.  A slow
            # but successful batch must not be labeled fresh at commit time.
            self.snapshot.contact_wall_s = batch_wall_s
            self.snapshot.contact_monotonic_s = batch_monotonic_s
            self.snapshot.connection_state = (
                RobotStatus.CONNECTION_DEGRADED
                if slow_batch
                else RobotStatus.CONNECTION_CONNECTED
            )
            self.snapshot.mode_code = state["mode_code"]
            self.snapshot.motor_state_code = int(motor_state)
            self.snapshot.is_playback = state["is_playback"]
            self.snapshot.is_remote_mode = state["is_remote_mode"]
            self.snapshot.speed_percent = float(
                state["playback_speed_percent"]
            )
            self.snapshot.emergency_stop = bool(emergency_stop)
            self.snapshot.last_error_id = state.get("last_error_id")
            api_suffix = (
                f", Open API v{self._api_version}"
                if self._api_version is not None
                else (
                    f", unsupported Open API v{self._observed_api_version}"
                    if self._observed_api_version is not None
                    else ", Open API version unverified"
                )
            )
            latency_suffix = (
                f", slow status batch {batch_age_s * 1000.0:.0f}ms"
                if slow_batch
                else ""
            )
            self.snapshot.status_message = (
                "controller readback" + api_suffix + latency_suffix
            )
        return True

    def _publish_if_running(self, publisher, message) -> None:
        if not self._is_running():
            return
        try:
            publisher.publish(message)
        except Exception:
            if not self._is_running():
                return
            raise

    def _is_running(self) -> bool:
        return (
            not self._shutting_down.is_set()
            and rclpy.ok(context=self.context)
        )

    # ------------------------------------------------------------- status model
    def _build_status_msg(self) -> RobotStatus:
        with self._snapshot_lock:
            snapshot = replace(self.snapshot)
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.robot_id = self.robot_id
        msg.sequence = snapshot.sequence
        msg.connection_state = snapshot.connection_state

        age = self._status_age_s(snapshot)
        msg.age_sec = float(age if math.isfinite(age) else -1.0)
        msg.fresh = (
            snapshot.connection_state == RobotStatus.CONNECTION_CONNECTED
            and math.isfinite(age)
            and age <= self.stale_timeout_s
        )
        if snapshot.contact_wall_s > 0.0:
            seconds = int(snapshot.contact_wall_s)
            msg.last_controller_contact.sec = seconds
            msg.last_controller_contact.nanosec = int(
                (snapshot.contact_wall_s - seconds) * 1_000_000_000
            )

        if (
            snapshot.mode_code in (3, 4)
            and snapshot.is_remote_mode is True
        ):
            msg.operation_mode = RobotStatus.OPERATION_REMOTE
        elif snapshot.mode_code in (0, 1):
            msg.operation_mode = RobotStatus.OPERATION_MANUAL
        elif snapshot.mode_code in (3, 4):
            msg.operation_mode = RobotStatus.OPERATION_AUTOMATIC
        else:
            msg.operation_mode = RobotStatus.OPERATION_UNKNOWN

        motor_states = {
            0: RobotStatus.MOTOR_ON,
            1: RobotStatus.MOTOR_OFF,
            2: RobotStatus.MOTOR_BUSY,
        }
        msg.motor_power_state = motor_states.get(
            snapshot.motor_state_code, RobotStatus.MOTOR_UNKNOWN
        )
        if snapshot.is_playback is True:
            msg.execution_state = RobotStatus.EXECUTION_RUNNING
        elif snapshot.is_playback is False:
            msg.execution_state = RobotStatus.EXECUTION_STOPPED
        else:
            msg.execution_state = RobotStatus.EXECUTION_UNKNOWN

        msg.emergency_stop_state = self._signal(snapshot.emergency_stop)
        # These states are not exposed by the currently selected documented APIs.
        msg.protective_stop_state = RobotStatus.SIGNAL_UNKNOWN
        msg.fault_state = RobotStatus.SIGNAL_UNKNOWN

        msg.requested_speed_valid = snapshot.requested_speed_percent is not None
        if snapshot.requested_speed_percent is not None:
            msg.requested_speed_percent = float(snapshot.requested_speed_percent)
        msg.actual_speed_valid = snapshot.speed_percent is not None
        if snapshot.speed_percent is not None:
            msg.actual_speed_percent = float(snapshot.speed_percent)

        if snapshot.last_error_id is not None and snapshot.last_error_id >= 0:
            msg.fault_code = str(snapshot.last_error_id)
            msg.fault_message = "last controller error id; active-fault state is unknown"
        msg.status_message = snapshot.status_message
        return msg

    @staticmethod
    def _signal(value: Optional[bool]) -> int:
        if value is None:
            return RobotStatus.SIGNAL_UNKNOWN
        return RobotStatus.SIGNAL_ACTIVE if value else RobotStatus.SIGNAL_INACTIVE

    def _status_age_s(self, snapshot: Optional[_Snapshot] = None) -> float:
        if snapshot is None:
            with self._snapshot_lock:
                snapshot = replace(self.snapshot)
        if snapshot.contact_monotonic_s is None:
            return math.inf
        return max(0.0, time.monotonic() - snapshot.contact_monotonic_s)

    def _probe_api_version(self) -> bool:
        with self._api_probe_lock:
            if self._api_version is not None:
                return True
            if self._observed_api_version is not None:
                return False
            try:
                with self._status_transport_lock:
                    if not self._is_running():
                        return False
                    version = self.status_client.get_api_version()
            except Hi6Error as exc:
                if not self._is_running():
                    return False
                self.get_logger().warn(
                    f"Hi6 Open API version probe failed: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False
            self._observed_api_version = version
            if version not in self.supported_api_versions:
                self.get_logger().error(
                    f"unsupported Hi6 Open API version {version}; "
                    f"reviewed versions are {list(self.supported_api_versions)}",
                    throttle_duration_sec=30.0,
                )
                return False
            self._api_version = version
            self.get_logger().info(
                f"Hi6 Open API version {version} verified"
            )
            return True

    # -------------------------------------------------------------- read service
    def on_get_status(self, request, response):
        controller_ack = False
        if request.force_controller_read:
            controller_ack = self._refresh_status()

        actual = self._build_status_msg()
        max_age = float(request.max_age_sec)
        if max_age <= 0.0:
            max_age = self.stale_timeout_s
        actual_age = float(actual.age_sec)
        response.success = bool(
            actual.sequence
            and actual_age >= 0.0
            and math.isfinite(actual_age)
            and actual_age <= max_age
            and actual.connection_state == RobotStatus.CONNECTION_CONNECTED
        )
        response.controller_ack = controller_ack
        response.from_cache = not request.force_controller_read or not controller_ack
        response.fresh = actual.fresh
        response.actual = actual
        response.error_code = "" if response.success else "STATUS_STALE"
        response.message = "fresh controller status" if response.success else (
            "no fresh controller status is available"
        )
        return response

    # ------------------------------------------------------------ write services
    def on_request_stop(self, request, response):
        request_id = self._request_id(request.request_id)
        response.request_id = request_id
        if not self.allow_commands:
            return self._reject(response, "READ_ONLY", "command services are disabled")
        if not self._probe_api_version():
            return self._reject(
                response,
                "API_UNVERIFIED",
                "Hi6 Open API schema is not in the reviewed allowlist",
            )

        response.accepted = True
        try:
            with self._write_lock:
                # The write lock is the command-ordering linearization point.
                # A start/speed already inside this section is allowed to
                # finish its in-flight transport; this stop is sent next.
                self._mark_stop_requested()
                if not self._is_running():
                    return self._reject(
                        response,
                        "SHUTTING_DOWN",
                        "stop was cancelled because the bridge is shutting down",
                    )
                if not self._command_source_is_available():
                    self.command_client.close()
                    return self._reject_source_unavailable(response)
                self.command_client.stop()
            response.controller_ack = True
        except Hi6Error as exc:
            return self._command_error(response, exc)

        response.confirmed = self._verify(
            lambda status: (
                status.fresh
                and status.execution_state == RobotStatus.EXECUTION_STOPPED
            ),
            self._confirmation_timeout(request.confirmation_timeout_sec),
        )
        response.actual = self._build_status_msg()
        if response.confirmed:
            response.message = "normal stop confirmed by controller readback"
        else:
            response.error_code = "CONFIRMATION_TIMEOUT"
            response.message = "controller acknowledged stop but STOPPED was not confirmed"
        self._audit("STOP", request_id, request.source, request.reason, response)
        return response

    def on_request_start(self, request, response):
        request_id = self._request_id(request.request_id)
        response.request_id = request_id
        stop_generation = self._current_stop_generation()
        if not self.allow_commands:
            return self._reject(response, "READ_ONLY", "command services are disabled")
        if not self.allow_start:
            return self._reject(
                response, "START_DISABLED", "remote start requires allow_start:=true"
            )
        if not self._probe_api_version():
            return self._reject(
                response,
                "API_UNVERIFIED",
                "Hi6 Open API schema is not in the reviewed allowlist",
            )

        if not self._refresh_status():
            return self._reject(response, "STATUS_STALE", "controller status is unavailable")
        status = self._build_status_msg()
        if not status.fresh:
            return self._reject(response, "STATUS_STALE", "controller status is stale")
        if status.operation_mode != RobotStatus.OPERATION_REMOTE:
            return self._reject(response, "NOT_REMOTE", "controller is not in REMOTE mode")
        if status.motor_power_state != RobotStatus.MOTOR_ON:
            return self._reject(response, "MOTOR_NOT_ON", "motor power is not ON")
        if status.emergency_stop_state != RobotStatus.SIGNAL_INACTIVE:
            return self._reject(
                response,
                "ESTOP_NOT_CLEAR",
                "emergency stop is not confirmed clear",
            )
        safety_unknown = (
            status.protective_stop_state == RobotStatus.SIGNAL_UNKNOWN
            or status.fault_state == RobotStatus.SIGNAL_UNKNOWN
        )
        if safety_unknown and not self.allow_unverified_start:
            return self._reject(
                response,
                "SAFETY_STATE_UNKNOWN",
                "protective-stop/current-fault readback is unavailable; "
                "remote start remains locked",
            )
        if (
            status.protective_stop_state == RobotStatus.SIGNAL_ACTIVE
            or status.fault_state == RobotStatus.SIGNAL_ACTIVE
        ):
            return self._reject(
                response,
                "SAFETY_STATE_ACTIVE",
                "protective stop or controller fault is active",
            )

        response.accepted = True
        try:
            with self._write_lock:
                if not self._is_running():
                    return self._reject(
                        response,
                        "SHUTTING_DOWN",
                        "start was cancelled because the bridge is shutting down",
                    )
                if self._stop_was_requested_after(stop_generation):
                    return self._reject(
                        response,
                        "PREEMPTED_BY_STOP",
                        "a newer stop request preempted start before transmission",
                    )
                if not self._command_source_is_available():
                    self.command_client.close()
                    return self._reject_source_unavailable(response)
                self.command_client.start()
            response.controller_ack = True
        except Hi6Error as exc:
            return self._command_error(response, exc)

        response.confirmed = self._verify(
            lambda current: (
                current.fresh
                and current.execution_state == RobotStatus.EXECUTION_RUNNING
            ),
            self._confirmation_timeout(request.confirmation_timeout_sec),
            stop_generation=stop_generation,
        )
        response.actual = self._build_status_msg()
        if response.confirmed:
            response.message = "start confirmed by controller readback"
        elif not self._is_running():
            response.error_code = "SHUTTING_DOWN"
            response.message = "start verification stopped during bridge shutdown"
        elif self._stop_was_requested_after(stop_generation):
            response.error_code = "PREEMPTED_BY_STOP"
            response.message = "start was superseded by a newer stop request"
        else:
            response.error_code = "CONFIRMATION_TIMEOUT"
            response.message = "controller acknowledged start but RUNNING was not confirmed"
        self._audit("START", request_id, request.source, request.reason, response)
        return response

    def on_set_speed_percent(self, request, response):
        request_id = self._request_id(request.request_id)
        response.request_id = request_id
        stop_generation = self._current_stop_generation()
        if not self.allow_commands:
            return self._reject(response, "READ_ONLY", "command services are disabled")

        requested = float(request.speed_percent)
        matched = next(
            (value for value in self.allowed_speeds if abs(requested - value) < 1e-6),
            None,
        )
        if matched is None:
            return self._reject(
                response,
                "INVALID_SPEED",
                f"allowed playback speeds are {list(self.allowed_speeds)} percent",
            )
        if not self._probe_api_version():
            return self._reject(
                response,
                "API_UNVERIFIED",
                "Hi6 Open API schema is not in the reviewed allowlist",
            )
        if not self._refresh_status():
            return self._reject(response, "STATUS_STALE", "controller status is unavailable")
        status = self._build_status_msg()
        if not status.fresh:
            return self._reject(response, "STATUS_STALE", "controller status is stale")
        if status.operation_mode != RobotStatus.OPERATION_REMOTE:
            return self._reject(response, "NOT_REMOTE", "controller is not in REMOTE mode")
        if status.emergency_stop_state != RobotStatus.SIGNAL_INACTIVE:
            return self._reject(
                response,
                "ESTOP_NOT_CLEAR",
                "emergency stop is not confirmed clear",
            )
        if not status.actual_speed_valid:
            return self._reject(
                response,
                "SPEED_UNKNOWN",
                "controller playback-speed readback is unavailable",
            )
        if (
            matched > status.actual_speed_percent + 0.5
            and not self.allow_speed_increase
        ):
            return self._reject(
                response,
                "SPEED_INCREASE_DISABLED",
                "less restrictive speed changes require "
                "allow_speed_increase:=true",
            )

        response.accepted = True
        try:
            with self._write_lock:
                if not self._is_running():
                    return self._reject(
                        response,
                        "SHUTTING_DOWN",
                        "speed request was cancelled because the bridge is shutting down",
                    )
                if self._stop_was_requested_after(stop_generation):
                    return self._reject(
                        response,
                        "PREEMPTED_BY_STOP",
                        "a newer stop request preempted speed before transmission",
                    )
                if not self._command_source_is_available():
                    self.command_client.close()
                    return self._reject_source_unavailable(response)
                self.command_client.set_playback_speed_percent(int(matched))
            response.controller_ack = True
            with self._snapshot_lock:
                self.snapshot.requested_speed_percent = float(matched)
        except Hi6Error as exc:
            return self._command_error(response, exc)

        deadline = time.monotonic() + self._confirmation_timeout(
            request.confirmation_timeout_sec
        )
        while time.monotonic() < deadline and self._is_running():
            if self._stop_was_requested_after(stop_generation):
                break
            try:
                with self._status_transport_lock:
                    if not self._is_running():
                        break
                    configured = self.status_client.get_playback_speed_percent()
            except Hi6Error:
                configured = None
            readback_ok = self._refresh_status()
            actual_status = self._build_status_msg()
            actual = (
                actual_status.actual_speed_percent
                if actual_status.actual_speed_valid
                else None
            )
            if (
                readback_ok
                and actual_status.fresh
                and configured == matched
                and actual is not None
                and abs(actual - matched) <= 0.5
            ):
                response.confirmed = True
                break
            time.sleep(0.1)

        response.actual = self._build_status_msg()
        if response.confirmed:
            response.message = f"playback speed {matched}% confirmed by controller readback"
        elif not self._is_running():
            response.error_code = "SHUTTING_DOWN"
            response.message = "speed verification stopped during bridge shutdown"
        elif self._stop_was_requested_after(stop_generation):
            response.error_code = "PREEMPTED_BY_STOP"
            response.message = "speed verification was superseded by a stop request"
        else:
            response.error_code = "CONFIRMATION_TIMEOUT"
            response.message = (
                "controller acknowledged speed but op_cnd/rgen readback did not converge"
            )
        self._audit("SPEED", request_id, request.source, request.reason, response)
        return response

    def _verify(
        self,
        predicate: Callable[[RobotStatus], bool],
        timeout_s: float,
        *,
        stop_generation: Optional[int] = None,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self._is_running():
            if (
                stop_generation is not None
                and self._stop_was_requested_after(stop_generation)
            ):
                return False
            if self._refresh_status() and predicate(self._build_status_msg()):
                return True
            time.sleep(0.1)
        return False

    def _mark_stop_requested(self) -> int:
        with self._stop_generation_lock:
            self._stop_generation += 1
            return self._stop_generation

    def _current_stop_generation(self) -> int:
        with self._stop_generation_lock:
            return self._stop_generation

    def _stop_was_requested_after(self, generation: int) -> bool:
        return self._current_stop_generation() != generation

    def _confirmation_timeout(self, requested: float) -> float:
        timeout = float(requested)
        if timeout <= 0.0:
            return self.verify_timeout_s
        return min(timeout, 30.0)

    def _command_source_is_available(self) -> bool:
        """Check the configured source address without sending traffic."""
        if self.source_address is None:
            return True
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.bind((self.source_address, 0))
            return True
        except OSError:
            return False

    def _reject_source_unavailable(self, response):
        return self._reject(
            response,
            "SOURCE_ADDRESS_UNAVAILABLE",
            "configured Hi6 source address is not assigned locally: "
            f"{self.source_address}",
        )

    @staticmethod
    def _request_id(requested: str) -> str:
        return str(requested).strip() or str(uuid.uuid4())

    def _reject(self, response, code: str, message: str):
        response.accepted = False
        response.controller_ack = False
        response.confirmed = False
        response.actual = self._build_status_msg()
        response.error_code = code
        response.message = message
        return response

    def _command_error(self, response, error: Exception):
        response.controller_ack = False
        response.confirmed = False
        response.actual = self._build_status_msg()
        response.error_code = "CONTROLLER_ERROR"
        response.message = str(error)
        return response

    def _audit(self, command: str, request_id: str, source: str, reason: str, response) -> None:
        self.get_logger().info(
            f"command={command} id={request_id} source={source or '-'} "
            f"accepted={response.accepted} ack={response.controller_ack} "
            f"confirmed={response.confirmed} reason={reason or '-'}"
        )

    def destroy_node(self) -> bool:
        self._shutting_down.set()
        self.pose_client.close()
        self.status_client.close()
        self.command_client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Hi6RobotNode()
    # Pose and status transports may each be waiting on the controller while a
    # motion request and an earlier stop are confirming.  Six workers preserve
    # cached-status publication and repeated-stop dispatch in that worst case.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutting_down.set()
        executor_drained = False
        try:
            executor_drained = executor.shutdown(
                timeout_sec=node.shutdown_timeout_s
            )
        except KeyboardInterrupt:
            pass
        if executor_drained:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
