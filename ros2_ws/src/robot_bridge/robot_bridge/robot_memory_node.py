"""메신저 ② — PLC ↔ ROS 2.

읽기 : D1000~D1013 을 주기 폴링해 RobotMemory / RobotPose 를 발행한다.
쓰기 : RobotCommand 를 구독해 버퍼1(긴급) · 버퍼2(감속) 워드를 기록한다.

별첨자료 `robotpose_python.py` 의 골격을 유지하되, TODO 로 비어 있던
`RobotMemoryClient` 자리를 실제 MC 프로토콜 구현(mc_client)으로 채웠다.

    ros2 run robot_bridge robot_memory_node --ros-args \
        -p profile:=field -p robot_id:=loading
"""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from robot_bridge_msgs.msg import RobotCommand, RobotMemory, RobotPose

from .config_loader import BridgeConfig
from .mc_client import McClient, McConfig, McError, bit_of, words_to_dword
from .safety_gate import SafetyGate

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RobotMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_memory_node")

        self.declare_parameter("profile", "sim")
        self.declare_parameter("robot_id", "loading")
        self.declare_parameter("config_dir", "")

        profile = self.get_parameter("profile").value
        robot_id = self.get_parameter("robot_id").value
        config_dir = self.get_parameter("config_dir").value or None

        self.cfg = BridgeConfig.load(config_dir, profile)
        self.robot = self.cfg.robot(robot_id)
        self.get_logger().info(
            f"프로파일 '{profile}' · 로봇 '{self.robot.id}' ({self.robot.label})"
        )
        if not self.robot.calibrated:
            self.get_logger().warn(
                "[미교정] scale/dir/offset 이 확정되지 않았습니다. "
                "각도값을 그대로 신뢰하지 마십시오 — docs/04_calibration.md 참조"
            )

        self.client = McClient(McConfig.from_dict(self.cfg.connection), self.get_logger())
        self.gate = SafetyGate(self.cfg.safety, self.get_logger())

        t = self.robot.topics
        self.pub_memory = self.create_publisher(RobotMemory, t["memory"], SENSOR_QOS)
        self.pub_pose = self.create_publisher(RobotPose, t["pose"] + "_raw", SENSOR_QOS)
        self.sub_cmd = self.create_subscription(
            RobotCommand, t["command"], self.on_command, 10
        )

        self.seq = 0
        self.last_ok: Optional[float] = None
        self.last_degrees: Optional[List[float]] = None

        self.create_timer(1.0 / max(self.cfg.poll_hz, 1.0), self.poll)
        self.create_timer(1.0 / max(self.cfg.rewrite_hz, 1.0), self.push_command)

    # ------------------------------------------------------------------ 읽기
    def poll(self) -> None:
        if not self.client.connected:
            try:
                self.client.connect()
            except OSError as e:
                self.client.note_failure()
                self.get_logger().warn(
                    f"PLC 연결 실패 ({e}) — {self.client.backoff_delay():.1f} s 후 재시도",
                    throttle_duration_sec=5.0,
                )
                self.publish_link_down()
                time.sleep(self.client.backoff_delay())
                return

        t0 = time.perf_counter()
        try:
            words = self.client.read_words(self.cfg.read_head, self.cfg.read_words)
            bits_word = self.client.read_words(self.cfg.status_bit_word, 1)[0]
        except (OSError, McError) as e:
            self.client.note_failure()
            self.get_logger().error(f"읽기 실패 : {e}", throttle_duration_sec=2.0)
            self.publish_link_down()
            return
        latency_ms = (time.perf_counter() - t0) * 1000.0
        self.last_ok = time.time()

        raw = [
            words_to_dword(words[a.offset], words[a.offset + 1])
            if a.type == "dword" else words[a.offset]
            for a in self.cfg.axes
        ]
        degrees, clamped = self.robot.to_degrees(raw)
        self.last_degrees = degrees

        now = self.get_clock().now().to_msg()
        self.seq += 1

        mem = RobotMemory()
        mem.header.stamp = now
        mem.header.frame_id = self.robot.id
        b = self.cfg.status_bits
        mem.hold = bit_of(bits_word, b.get("hold", 0))
        mem.emergency_stop = bit_of(bits_word, b.get("emergency_stop", 1))
        mem.run = not (mem.hold or mem.emergency_stop)
        st = self.gate.last_applied()
        mem.speed_down_1 = st.speed_down_1
        mem.speed_down_2 = st.speed_down_2
        mem.speed_down_3 = st.speed_down_3
        mem.operation_state = int(words[self.cfg.status_offset])
        (mem.s_axis, mem.h_axis, mem.v_axis,
         mem.r2_axis, mem.b_axis, mem.r1_axis) = raw
        mem.link_ok = True
        mem.seq = self.seq
        mem.read_latency_ms = float(latency_ms)
        self.pub_memory.publish(mem)

        pose = RobotPose()
        pose.header.stamp = now
        pose.header.frame_id = self.robot.id
        pose.robot_id = self.robot.id
        pose.axis_names = self.robot.axis_names
        pose.degrees = degrees
        pose.raw = raw
        pose.calibrated = self.robot.calibrated
        pose.clamped = clamped
        self.pub_pose.publish(pose)

        if clamped:
            self.get_logger().warn(
                "관절각이 limits_deg 를 벗어나 잘렸습니다 — scale/offset 확인 필요",
                throttle_duration_sec=10.0,
            )

    def publish_link_down(self) -> None:
        mem = RobotMemory()
        mem.header.stamp = self.get_clock().now().to_msg()
        mem.header.frame_id = self.robot.id
        mem.link_ok = False
        mem.seq = self.seq
        # 통신 두절은 안전측으로 알린다
        fail_safe = self.cfg.safety.get("fail_safe", "hold")
        mem.hold = fail_safe == "hold"
        mem.emergency_stop = fail_safe == "stop"
        self.pub_memory.publish(mem)

    # ------------------------------------------------------------------ 쓰기
    def on_command(self, msg: RobotCommand) -> None:
        self.gate.submit(msg)

    def push_command(self) -> None:
        """중재된 최종 지령을 PLC 버퍼에 기록한다 (워치독 겸용 주기 재기록)."""
        if not self.client.connected:
            return
        stale = (self.last_ok is None
                 or (time.time() - self.last_ok) * 1000.0
                 > self.cfg.safety.get("watchdog_timeout_ms", 500))
        applied = self.gate.resolve(link_stale=stale)

        blocks = self.cfg.write_blocks
        try:
            emer = blocks.get("emergency")
            if emer:
                f = emer["fields"]
                w = [0] * int(emer.get("words", 3))
                w[f["run"]] = int(applied.run)
                w[f["hold"]] = int(applied.hold)
                w[f["stop"]] = int(applied.stop)
                self.client.write_words(emer["device"], w)

            slow = blocks.get("slowdown")
            if slow:
                f = slow["fields"]
                w = [0] * int(slow.get("words", 3))
                w[f["speed_down_1"]] = int(applied.speed_down_1)
                w[f["speed_down_2"]] = int(applied.speed_down_2)
                w[f["speed_down_3"]] = int(applied.speed_down_3)
                self.client.write_words(slow["device"], w)
        except (OSError, McError) as e:
            self.client.note_failure()
            self.get_logger().error(f"명령 기록 실패 : {e}", throttle_duration_sec=2.0)

    def destroy_node(self) -> bool:
        self.client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
