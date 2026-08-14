"""가상 PLC 를 ROS 2 노드로 감싸 launch 에서 함께 띄운다."""

import threading

import rclpy
from rclpy.node import Node

from .fake_plc import serve


class FakePlcNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_plc_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5010)
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        threading.Thread(target=serve, args=(host, port), daemon=True).start()
        self.get_logger().info(f"가상 PLC 기동 : {host}:{port} (MC 3E 바이너리)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePlcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
