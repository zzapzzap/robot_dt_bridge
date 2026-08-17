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
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32MultiArray

from robot_bridge_msgs.msg import RobotCommand, RobotMemory, RobotPose, SafetyMode

from .config_loader import BridgeConfig

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)


class UnityAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("unity_adapter_node")

        self.declare_parameter("profile", "sim")
        self.declare_parameter("config_dir", "")
        profile = self.get_parameter("profile").value
        config_dir = self.get_parameter("config_dir").value or None

        self.cfg = BridgeConfig.load(config_dir, profile)
        self.state_layout: List[str] = list(
            self.cfg.unity.get("state_layout",
                               ["run", "hold", "emergency_stop",
                                "speed_down_1", "speed_down_2", "speed_down_3",
                                "operation_state"]))
        self.cmd_layout: List[str] = list(
            self.cfg.unity.get("command_layout",
                               ["run", "hold", "stop",
                                "speed_down_1", "speed_down_2", "speed_down_3"]))

        self.pub_cmd: Dict[str, object] = {}
        for r in self.cfg.enabled_robots():
            t = r.topics
            pose_pub = self.create_publisher(Float64MultiArray, t["pose"], QOS)
            state_pub = self.create_publisher(Int32MultiArray, t["state"], QOS)
            ns = t["memory"].rsplit("/", 1)[0]
            mode_pub = self.create_publisher(Int32MultiArray, ns + "/mode_unity", QOS)
            self.pub_cmd[r.id] = self.create_publisher(RobotCommand, t["command"], 10)

            self.create_subscription(
                RobotPose, t["pose"] + "_raw",
                lambda m, p=pose_pub: self.on_pose(m, p), QOS)
            self.create_subscription(
                RobotMemory, t["memory"],
                lambda m, p=state_pub: self.on_memory(m, p), QOS)
            self.create_subscription(
                SafetyMode, ns + "/mode",
                lambda m, p=mode_pub: self.on_mode(m, p), QOS)
            self.create_subscription(
                Int32MultiArray, ns + "/unity_command",
                lambda m, rid=r.id: self.on_unity_command(m, rid), 10)
            self.get_logger().info(f"어댑터 등록 : {r.id} → {t['pose']} / {t['state']}")

        # ------------------------------------------------------- 작업자 pose
        w = self.cfg.workers
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
    def on_pose(self, msg: RobotPose, pub) -> None:
        out = Float64MultiArray()
        out.data = list(msg.degrees)
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
