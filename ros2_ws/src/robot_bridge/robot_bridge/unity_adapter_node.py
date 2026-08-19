"""메신저 ① — Unity ↔ ROS 2.

Unity(ROS-TCP-Connector)가 커스텀 메시지 코드젠 없이 붙을 수 있도록,
커스텀 msg ↔ std_msgs 를 양방향 변환한다.

  ROS 2 → Unity
    RobotPose          →  std_msgs/Float64MultiArray   /robot/<id>/cmd_degs
    RobotMemory        →  std_msgs/Int32MultiArray     /robot/<id>/state
    PoseArray(작업자)   →  std_msgs/Float32MultiArray   /worker/unity/bodies

  Unity → ROS 2
    std_msgs/Int32MultiArray  /robot/<id>/unity_command  →  RobotCommand

작업자 pose 는 최대 5인 × 28관절을 하나의 평면 배열로 눌러 보낸다.
    [n_bodies, id0, v0x,v0y,v0z, …(28관절)…, id1, …]
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32MultiArray

from robot_bridge_msgs.msg import (
    RobotCommand,
    RobotMemory,
    RobotPose,
    RobotStatus,
    SafetyMode,
)

from .config_loader import BridgeConfig, Hi6Config, PlcBridgeConfig

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)

DEFAULT_STATE_LAYOUT = [
    "run", "hold", "emergency_stop",
    "speed_down_1", "speed_down_2", "speed_down_3",
    "operation_state",
]
DEFAULT_COMMAND_LAYOUT = [
    "run", "hold", "stop",
    "speed_down_1", "speed_down_2", "speed_down_3",
]
DEFAULT_WORKERS = {
    "max_bodies": 5,
    "joints": 28,
    "stale_timeout_ms": 500,
    "topic_prefix": "/worker/pose",
    "suffix": "_joints",
}


@dataclass(frozen=True)
class UnityRobotRoute:
    """Canonical Unity topics for one direct-Hi6 instance."""

    id: str
    topics: Dict[str, str]


def canonical_hi6_unity_route(robot_id: str) -> UnityRobotRoute:
    """Build topics from one config instance id without robots.yaml."""
    namespace = f"/robot/{robot_id}"
    return UnityRobotRoute(
        id=robot_id,
        topics={
            "memory": namespace + "/memory",
            "pose": namespace + "/cmd_degs",
            "state": namespace + "/state",
            "command": namespace + "/command",
        },
    )


class UnityAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("unity_adapter_node")

        self.declare_parameter("profile", "sim")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("robots", "")
        self.declare_parameter("instances", "")
        self.declare_parameter("forward_unity_commands", True)
        self.declare_parameter("use_hi6_instances", False)
        self.declare_parameter("use_plc_instances", False)
        profile = self.get_parameter("profile").value
        config_dir = self.get_parameter("config_dir").value or None
        legacy_robot_text = str(self.get_parameter("robots").value or "")
        instance_text = str(self.get_parameter("instances").value or "")
        forward_commands = bool(
            self.get_parameter("forward_unity_commands").value
        )
        use_hi6_instances = bool(
            self.get_parameter("use_hi6_instances").value
        )
        use_plc_instances = bool(
            self.get_parameter("use_plc_instances").value
        )
        if use_hi6_instances and use_plc_instances:
            raise ValueError(
                "use_hi6_instances and use_plc_instances are mutually exclusive"
            )

        pose_transforms: Dict[
            str,
            Callable[[List[float]], Tuple[List[float], bool]],
        ] = {}
        if use_hi6_instances or use_plc_instances:
            selected_text = instance_text or legacy_robot_text
            instance_config = (
                Hi6Config.load(config_dir)
                if use_hi6_instances
                else PlcBridgeConfig.load(config_dir, profile=profile)
            )
            selected_ids = [
                item.strip()
                for item in selected_text.split(",")
                if item.strip()
            ]
            if selected_ids:
                instances = [
                    instance_config.instance(robot_id)
                    if hasattr(instance_config, "instance")
                    else instance_config.robot(robot_id)
                    for robot_id in selected_ids
                ]
            else:
                enabled = getattr(
                    instance_config,
                    "enabled_instances",
                    instance_config.enabled_robots,
                )
                instances = enabled()
            selected = [
                canonical_hi6_unity_route(instance.robot_id)
                for instance in instances
            ]
            if use_plc_instances:
                pose_transforms = {
                    instance.robot_id: instance.to_visual_degrees
                    for instance in instances
                }
                self.state_layout = list(
                    instance_config.unity.get(
                        "state_layout", DEFAULT_STATE_LAYOUT
                    )
                )
                self.cmd_layout = list(
                    instance_config.unity.get(
                        "command_layout", DEFAULT_COMMAND_LAYOUT
                    )
                )
                worker_config = dict(instance_config.workers)
            else:
                self.state_layout = list(DEFAULT_STATE_LAYOUT)
                self.cmd_layout = list(DEFAULT_COMMAND_LAYOUT)
                worker_config = dict(DEFAULT_WORKERS)
        else:
            selected_ids = [
                item.strip()
                for item in legacy_robot_text.split(",")
                if item.strip()
            ]
            legacy_config = BridgeConfig.load(config_dir, profile)
            self.state_layout = list(
                legacy_config.unity.get(
                    "state_layout", DEFAULT_STATE_LAYOUT
                )
            )
            self.cmd_layout = list(
                legacy_config.unity.get(
                    "command_layout", DEFAULT_COMMAND_LAYOUT
                )
            )
            selected = (
                [legacy_config.robot(robot_id) for robot_id in selected_ids]
                if selected_ids
                else legacy_config.enabled_robots()
            )
            worker_config = dict(legacy_config.workers)

        self.pub_cmd: Dict[str, object] = {}
        for r in selected:
            t = r.topics
            pose_pub = self.create_publisher(Float64MultiArray, t["pose"], QOS)
            state_pub = self.create_publisher(Int32MultiArray, t["state"], QOS)
            ns = t["memory"].rsplit("/", 1)[0]
            mode_pub = self.create_publisher(Int32MultiArray, ns + "/mode_unity", QOS)
            if forward_commands:
                self.pub_cmd[r.id] = self.create_publisher(
                    RobotCommand, t["command"], 10
                )

            self.create_subscription(
                RobotPose, t["pose"] + "_raw",
                lambda m, p=pose_pub, transform=pose_transforms.get(r.id):
                self.on_pose(m, p, transform), QOS)
            self.create_subscription(
                RobotMemory, t["memory"],
                lambda m, p=state_pub: self.on_memory(m, p), QOS)
            self.create_subscription(
                SafetyMode, ns + "/mode",
                lambda m, p=mode_pub: self.on_mode(m, p), QOS)
            self.create_subscription(
                RobotStatus, ns + "/status",
                lambda m, sp=state_pub, mp=mode_pub:
                self.on_robot_status(m, sp, mp), QOS)
            if forward_commands:
                self.create_subscription(
                    Int32MultiArray, ns + "/unity_command",
                    lambda m, rid=r.id: self.on_unity_command(m, rid), 10)
            self.get_logger().info(f"어댑터 등록 : {r.id} → {t['pose']} / {t['state']}")
        if not forward_commands:
            self.get_logger().info(
                "Unity command forwarding is disabled; Hi6 control uses "
                "confirmed ROS services"
            )

        # ------------------------------------------------------- 작업자 pose
        w = worker_config
        self.max_bodies = int(w.get("max_bodies", 5))
        self.joints = int(w.get("joints", 28))
        self.stale_s = float(w.get("stale_timeout_ms", 500)) / 1000.0
        self.bodies: Dict[int, tuple] = {}          # idx → (stamp, [x,y,z]*joints)

        prefix = w.get("topic_prefix", "/worker/pose")
        suffix = w.get("suffix", "_joints")
        for i in range(self.max_bodies):
            self.create_subscription(
                PoseArray, f"{prefix}{i}{suffix}",
                lambda m, idx=i: self.on_worker(m, idx), QOS)
        self.pub_bodies = self.create_publisher(
            Float32MultiArray, "/worker/unity/bodies", QOS)
        self.create_timer(1.0 / 20.0, self.push_bodies)

    # ------------------------------------------------------------ ROS → Unity
    def on_pose(
        self,
        msg: RobotPose,
        pub,
        transform: Optional[
            Callable[[List[float]], Tuple[List[float], bool]]
        ] = None,
    ) -> None:
        out = Float64MultiArray()
        degrees = list(msg.degrees)
        if transform is not None:
            degrees, _clamped = transform(degrees)
        out.data = list(degrees)
        pub.publish(out)

    def on_mode(self, msg: SafetyMode, pub) -> None:
        """SafetyMode → Int32MultiArray[4] = [모드, 속도%, 링크정상, 고정여부]."""
        out = Int32MultiArray()
        out.data = [int(msg.mode), int(round(msg.speed_ratio * 100)),
                    int(msg.link_ok), int(msg.latched)]
        pub.publish(out)

    def on_memory(self, msg: RobotMemory, pub) -> None:
        out = Int32MultiArray()
        out.data = [int(getattr(msg, k, 0)) for k in self.state_layout]
        pub.publish(out)

    def on_robot_status(self, msg: RobotStatus, state_pub, mode_pub) -> None:
        """Direct Hi6 readback → legacy Unity arrays.

        A disconnected/stale message is withheld from the legacy state topic so
        the Unity receiver's own watchdog expires.  Consumers that need
        three-state/validity semantics must subscribe to RobotStatus itself.
        """
        speed = float(msg.actual_speed_percent) if msg.actual_speed_valid else 0.0
        values = {
            "run": msg.execution_state == RobotStatus.EXECUTION_RUNNING,
            "hold": False,
            "emergency_stop": msg.emergency_stop_state == RobotStatus.SIGNAL_ACTIVE,
            "speed_down_1": msg.actual_speed_valid and abs(speed - 75.0) <= 0.5,
            "speed_down_2": msg.actual_speed_valid and abs(speed - 50.0) <= 0.5,
            "speed_down_3": msg.actual_speed_valid and abs(speed - 25.0) <= 0.5,
            "operation_state": int(msg.execution_state),
        }
        link_ok = bool(
            msg.connection_state == RobotStatus.CONNECTION_CONNECTED
            and msg.fresh
        )
        legacy_valid = bool(
            link_ok
            and msg.emergency_stop_state != RobotStatus.SIGNAL_UNKNOWN
            and msg.protective_stop_state != RobotStatus.SIGNAL_UNKNOWN
            and msg.fault_state != RobotStatus.SIGNAL_UNKNOWN
        )
        if legacy_valid:
            state = Int32MultiArray()
            state.data = [int(values.get(name, 0)) for name in self.state_layout]
            state_pub.publish(state)

        mode_value = SafetyMode.MODE_UNKNOWN
        if legacy_valid:
            if msg.emergency_stop_state == RobotStatus.SIGNAL_ACTIVE:
                mode_value = SafetyMode.MODE_EMERGENCY_STOP
            elif (
                msg.execution_state == RobotStatus.EXECUTION_RUNNING
                and msg.actual_speed_valid
            ):
                speed_modes = {
                    100: SafetyMode.MODE_NORMAL,
                    75: SafetyMode.MODE_REDUCED_75,
                    50: SafetyMode.MODE_REDUCED_50,
                    25: SafetyMode.MODE_REDUCED_25,
                }
                mode_value = speed_modes.get(
                    int(round(speed)), SafetyMode.MODE_UNKNOWN
                )

        mode = Int32MultiArray()
        mode.data = [
            int(mode_value),
            int(round(speed)) if legacy_valid and msg.actual_speed_valid else 0,
            int(legacy_valid),
            0,
        ]
        mode_pub.publish(mode)

    def on_worker(self, msg: PoseArray, idx: int) -> None:
        flat: List[float] = []
        for p in msg.poses[: self.joints]:
            flat += [p.position.x, p.position.y, p.position.z]
        # 관절 수가 모자라면 0 으로 채워 길이를 고정한다
        flat += [0.0] * (self.joints * 3 - len(flat))
        self.bodies[idx] = (time.time(), flat)

    def push_bodies(self) -> None:
        now = time.time()
        live = {i: v for i, v in self.bodies.items() if now - v[0] <= self.stale_s}
        out = Float32MultiArray()
        data: List[float] = [float(len(live))]
        for idx in sorted(live):
            data.append(float(idx))
            data += [float(x) for x in live[idx][1]]
        out.data = data
        self.pub_bodies.publish(out)

    # ------------------------------------------------------------ Unity → ROS
    def on_unity_command(self, msg: Int32MultiArray, robot_id: str) -> None:
        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = "unity"
        cmd.priority = 0                        # 우선순위는 safety_gate 표를 따름
        for i, name in enumerate(self.cmd_layout):
            if i < len(msg.data):
                setattr(cmd, name, bool(msg.data[i]))
        cmd.reason = "Unity 조작 패널"
        self.pub_cmd[robot_id].publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnityAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
