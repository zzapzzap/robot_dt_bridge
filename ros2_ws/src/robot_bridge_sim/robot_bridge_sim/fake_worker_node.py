"""작업자 pose 더미 발행 — Unity 배치 확인용.

MVP(multiview_pose) 가 없는 환경에서도 사람 형상이 움직이도록
28관절 PoseArray 를 2명분 발행한다.
"""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

JOINTS = 28

# 대략적인 인체 비율 (m) — [x, y, z] 오프셋. z 가 높이.
SKELETON = [
    (0.00, 0.00, 1.70), (0.00, 0.00, 1.55), (-0.18, 0.00, 1.45), (0.18, 0.00, 1.45),
    (-0.20, 0.00, 1.18), (0.20, 0.00, 1.18), (-0.22, 0.00, 0.92), (0.22, 0.00, 0.92),
    (-0.12, 0.00, 1.00), (0.12, 0.00, 1.00), (-0.12, 0.00, 0.55), (0.12, 0.00, 0.55),
    (-0.12, 0.00, 0.10), (0.12, 0.00, 0.10), (-0.14, 0.08, 0.04), (0.14, 0.08, 0.04),
    (-0.14, -0.06, 0.02), (0.14, -0.06, 0.02), (0.00, 0.00, 1.00), (0.00, 0.00, 1.30),
    (-0.25, 0.00, 0.88), (0.25, 0.00, 0.88), (0.00, 0.00, 0.98), (0.00, 0.00, 1.62),
    (-0.06, 0.00, 1.74), (0.06, 0.00, 1.74), (-0.10, 0.02, 1.70), (0.10, 0.02, 1.70),
]


class FakeWorkerNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_worker_node")
        self.declare_parameter("bodies", 2)
        self.n = int(self.get_parameter("bodies").value)
        self.pubs = [
            self.create_publisher(PoseArray, f"/worker/pose{i}_joints", 1)
            for i in range(self.n)
        ]
        self.t = 0.0
        self.create_timer(1.0 / 20.0, self.tick)
        self.get_logger().info(f"작업자 더미 {self.n} 명 발행 (28관절 · 20 Hz)")

    def tick(self) -> None:
        self.t += 1.0 / 20.0
        for i, pub in enumerate(self.pubs):
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stag_marker"
            # 로봇 주위를 서로 반대 방향으로 천천히 돈다
            r = 2.0 + 0.6 * i
            ang = 0.25 * self.t * (1 if i % 2 == 0 else -1) + i * math.pi
            cx, cy = r * math.cos(ang), r * math.sin(ang)
            sway = 0.05 * math.sin(3.0 * self.t + i)
            for (jx, jy, jz) in SKELETON:
                p = Pose()
                p.position.x = cx + jx * math.cos(ang) - (jy + sway) * math.sin(ang)
                p.position.y = cy + jx * math.sin(ang) + (jy + sway) * math.cos(ang)
                p.position.z = jz
                p.orientation.w = 1.0
                msg.poses.append(p)
            pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeWorkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
