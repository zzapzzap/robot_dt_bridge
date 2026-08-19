"""Config ownership and pose conversion checks for the Unity adapter."""

from robot_bridge.unity_adapter_node import (
    UnityAdapterNode,
    canonical_hi6_unity_route,
)
from robot_bridge_msgs.msg import RobotPose


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_direct_hi6_topics_come_only_from_instance_id() -> None:
    route = canonical_hi6_unity_route("loading")

    assert route.id == "loading"
    assert route.topics == {
        "memory": "/robot/loading/memory",
        "pose": "/robot/loading/cmd_degs",
        "state": "/robot/loading/state",
        "command": "/robot/loading/command",
    }


def test_each_hi6_instance_gets_an_isolated_namespace() -> None:
    loading = canonical_hi6_unity_route("loading")
    unloading = canonical_hi6_unity_route("unloading")

    assert set(loading.topics.values()).isdisjoint(unloading.topics.values())


def test_plc_visual_transform_is_only_applied_when_supplied() -> None:
    controller = [38.56, 136.25, -49.48, 0.17, -86.85, -50.68]
    visual = [-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]
    message = RobotPose()
    message.degrees = list(controller)

    direct_pub = _Publisher()
    UnityAdapterNode.on_pose(None, message, direct_pub)
    assert list(direct_pub.messages[-1].data) == controller

    plc_pub = _Publisher()
    calls = []

    def transform(values):
        calls.append(list(values))
        return list(visual), False

    UnityAdapterNode.on_pose(None, message, plc_pub, transform)
    assert calls == [controller]
    assert list(plc_pub.messages[-1].data) == visual
    # Adapter conversion must not rewrite canonical controller telemetry.
    assert list(message.degrees) == controller
