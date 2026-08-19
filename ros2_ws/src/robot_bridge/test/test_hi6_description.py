"""Static checks for the packaged Hi6 RViz debug description."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_DIR / "urdf" / "ys080_hh050_debug.urdf.xacro"
MESH_DIR = PACKAGE_DIR / "meshes" / "ys080_hh050"


def test_xacro_is_valid_serial_six_axis_description(tmp_path: Path) -> None:
    urdf_file = tmp_path / "robot.urdf"
    rendered = subprocess.run(
        ["xacro", str(XACRO_FILE)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    urdf_file.write_text(rendered, encoding="utf-8")

    subprocess.run(
        ["check_urdf", str(urdf_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(rendered)
    moving_joints = [
        joint for joint in root.findall("joint")
        if joint.attrib.get("type") == "revolute"
    ]
    assert [joint.attrib["name"] for joint in moving_joints] == [
        f"joint_{index}" for index in range(1, 7)
    ]
    assert [joint.find("axis").attrib["xyz"] for joint in moving_joints] == [
        "0 0 -1",
        "0 -1 0",
        "0 1 0",
        "-1 0 0",
        "0 -1 0",
        "-1 0 0",
    ]


def test_all_cad_mesh_resources_are_packaged() -> None:
    expected = {
        "BASE.obj", "J1_S.obj", "J2_H.obj", "J3_V.obj",
        "J4_R2.obj", "J5_B.obj", "J6_R1.obj", "kinematics.json",
    }
    assert {path.name for path in MESH_DIR.iterdir()} == expected
    assert all((MESH_DIR / name).stat().st_size > 0 for name in expected)


def test_rviz_config_uses_namespaced_transient_descriptions() -> None:
    rviz_path = PACKAGE_DIR / "rviz" / "hi6_debug.rviz"
    rviz_text = rviz_path.read_text(encoding="utf-8")
    rviz_config = yaml.safe_load(rviz_text)
    displays = {
        display["Name"]: display
        for display in rviz_config["Visualization Manager"]["Displays"]
    }
    assert displays["Loading Robot"]["TF Prefix"] == "loading"
    assert displays["Unloading Robot"]["TF Prefix"] == "unloading"
    assert (
        displays["Loading Robot"]["Description Topic"]["Value"]
        == "/robot/loading/robot_description"
    )
    assert (
        displays["Unloading Robot"]["Description Topic"]["Value"]
        == "/robot/unloading/robot_description"
    )
    assert rviz_text.count("Durability Policy: Transient Local") == 2
