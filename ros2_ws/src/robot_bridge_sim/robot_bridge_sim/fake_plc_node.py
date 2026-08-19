"""ROS 2 process wrapper for the functional fake MELSEC PLC."""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from .fake_plc import FakePlcServer, make_server


class FakePlcNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_plc_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5010)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        # Bind before starting a background thread.  If the port is occupied,
        # node construction fails and launch sees a non-zero process exit.
        self._server: Optional[FakePlcServer] = make_server(host, port)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="fake-plc-server",
            daemon=True,
        )
        self._thread.start()
        actual_host, actual_port = self._server.server_address[:2]
        self.get_logger().info(
            f"fake PLC ready: {actual_host}:{actual_port} (MC 3E binary/ascii)"
        )

    def destroy_node(self) -> None:
        server = self._server
        self._server = None
        if server is not None:
            server.shutdown()
            server.server_close()
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[FakePlcNode] = None
    try:
        node = FakePlcNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
