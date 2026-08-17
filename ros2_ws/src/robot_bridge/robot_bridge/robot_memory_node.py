"""메신저 ② — PLC ↔ ROS 2.

읽기 : D1000~D1013 을 주기 폴링해 RobotMemory / RobotPose 를 발행한다.
쓰기 : RobotCommand 를 구독해 버퍼1(정지) · 버퍼2(속도제한) 워드를 기록한다.

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

from robot_bridge_msgs.msg import RobotCommand, RobotMemory, RobotPose, SafetyMode
from robot_bridge_msgs.srv import GetSafetyMode, SetSafetyMode

from .config_loader import BridgeConfig
from .mc_client import McClient, McConfig, McError, bit_of, words_to_dword
from .safety_gate import SafetyGate, mode_label, mode_name

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

        # 모드 방송 — 지령이 바뀌면 즉시, 그 외에는 1 Hz 로 유지 신호
        mode_topic = t.get("mode", t["memory"].rsplit("/", 1)[0] + "/mode")
        self.pub_mode = self.create_publisher(SafetyMode, mode_topic, SENSOR_QOS)

        # 모드 서비스 — 설정 / 조회
        ns = t["memory"].rsplit("/", 1)[0]
        self.srv_set = self.create_service(
            SetSafetyMode, ns + "/set_mode", self.on_set_mode)
        self.srv_get = self.create_service(
            GetSafetyMode, ns + "/get_mode", self.on_get_mode)
        self.get_logger().info(
            f"모드 서비스 : {ns}/set_mode · {ns}/get_mode   방송 : {mode_topic}")
        self._last_mode_pub = 0.0
        self._last_mode = -1

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

    # ---------------------------------------------------------------- 서비스
    def on_set_mode(self, req, res):
        ok, message = self.gate.set_mode(
            int(req.mode),
            source=req.source or "service",
            reason=req.reason,
            hold_seconds=float(req.hold_seconds),
            clear_latched=bool(req.clear_latched),
        )
        applied = self.gate.resolve(link_stale=self._link_stale())
        res.accepted = ok
        res.applied_mode = applied.mode
        res.applied_mode_name = applied.mode_name
        res.applied_mode_label = applied.mode_label
        res.message = message
        self.publish_mode(force=True)
        self.push_command()                      # 즉시 PLC 에 반영
        self.get_logger().info(
            f"set_mode({mode_name(int(req.mode))}) ← {req.source or 'service'} "
            f"→ {'수락' if ok else '거절'} · 현재 {applied.mode_name} "
            f"({applied.mode_label})")
        return res

    def on_get_mode(self, _req, res):
        res.state = self.build_mode_msg()
        return res

    # ------------------------------------------------------------- 모드 방송
    def _link_stale(self) -> bool:
        return (self.last_ok is None
                or (time.time() - self.last_ok) * 1000.0
                > self.cfg.safety.get("watchdog_timeout_ms", 500))

    def build_mode_msg(self) -> SafetyMode:
        a = self.gate.last_applied()
        m = SafetyMode()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.robot.id
        m.mode = a.mode
        m.mode_name = a.mode_name
        m.mode_label = a.mode_label
        m.speed_ratio = float(a.speed_ratio)
        m.source = a.source
        m.reason = a.reason
        m.priority = int(min(a.priority, 255))
        m.latched = bool(a.latched)
        m.latch_remaining_s = float(a.latch_remaining_s)
        m.link_ok = not self._link_stale()
        since = self.gate.mode_since()
        m.since.sec = int(since)
        m.since.nanosec = int((since - int(since)) * 1e9)
        return m

    def publish_mode(self, force: bool = False) -> None:
        now = time.time()
        cur = self.gate.mode()
        if force or cur != self._last_mode or (now - self._last_mode_pub) >= 1.0:
            self.pub_mode.publish(self.build_mode_msg())
            self._last_mode = cur
            self._last_mode_pub = now

    def push_command(self) -> None:
        """중재된 최종 지령을 PLC 버퍼에 기록한다 (워치독 겸용 주기 재기록)."""
        if not self.client.connected:
            return
        applied = self.gate.resolve(link_stale=self._link_stale())
        self.publish_mode()

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
