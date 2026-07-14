"""Export the checked-in G1 URDF and STL visuals as a web-ready GLB.

The exporter also writes the 29 movable joint origins in glTF's Y-up frame so
the portfolio can render accurate interactive degree-of-freedom hotspots.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "XRTeleoperation/Assets/Teleoperation/Robots/G1/Model/g1_29dof.urdf"
OUTPUT = ROOT / "portfolio/assets/g1-29dof.glb"
JOINTS_OUTPUT = ROOT / "portfolio/assets/g1-29dof-joints.json"

# A relaxed standing pose is easier to read than the URDF's arms-forward zero
# configuration. Values are radians and remain inside the source joint limits.
DISPLAY_POSE = {
    "left_shoulder_pitch_joint": 1.28,
    "right_shoulder_pitch_joint": 1.28,
    "left_shoulder_roll_joint": 0.12,
    "right_shoulder_roll_joint": -0.12,
    "left_elbow_joint": 0.32,
    "right_elbow_joint": 0.32,
}


def floats(value: str | None, length: int = 3) -> np.ndarray:
    if not value:
        return np.zeros(length)
    return np.array([float(part) for part in value.split()], dtype=float)


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if element is None:
        return matrix

    x, y, z = floats(element.get("xyz"))
    roll, pitch, yaw = floats(element.get("rpy"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (x, y, z)
    return matrix


def main() -> None:
    robot = ET.parse(URDF).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = robot.findall("joint")
    child_names = {joint.find("child").get("link") for joint in joints}
    root_name = next(name for name in links if name not in child_names)

    children: dict[str, list[ET.Element]] = {}
    for joint in joints:
        parent = joint.find("parent").get("link")
        children.setdefault(parent, []).append(joint)

    link_transforms: dict[str, np.ndarray] = {root_name: np.eye(4)}
    movable: list[dict[str, object]] = []

    def walk(parent_name: str) -> None:
        parent_transform = link_transforms[parent_name]
        for joint in children.get(parent_name, []):
            child_name = joint.find("child").get("link")
            joint_origin = origin_matrix(joint.find("origin"))
            joint_angle = DISPLAY_POSE.get(joint.get("name"), 0.0)
            axis_element = joint.find("axis")
            axis = floats(axis_element.get("xyz") if axis_element is not None else None)
            joint_rotation = (
                trimesh.transformations.rotation_matrix(joint_angle, axis)
                if joint_angle and np.linalg.norm(axis) > 0
                else np.eye(4)
            )
            child_transform = parent_transform @ joint_origin @ joint_rotation
            link_transforms[child_name] = child_transform
            if joint.get("type") != "fixed":
                limit = joint.find("limit")
                movable.append(
                    {
                        "name": joint.get("name"),
                        "position": child_transform[:3, 3].tolist(),
                        "axis": axis.tolist(),
                        "lower": float(limit.get("lower", "0")) if limit is not None else 0,
                        "upper": float(limit.get("upper", "0")) if limit is not None else 0,
                    }
                )
            walk(child_name)

    walk(root_name)
    if len(movable) != 29:
        raise RuntimeError(f"Expected 29 movable joints, found {len(movable)}")

    # URDF is Z-up; glTF/model-viewer is Y-up. Rotate -90 degrees around X.
    web_frame = trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0])
    scene = trimesh.Scene()
    material_colors = {
        "dark": [42, 43, 42, 255],
        "white": [198, 197, 191, 255],
    }

    for link_name, link in links.items():
        link_transform = link_transforms.get(link_name)
        if link_transform is None:
            continue
        for visual_index, visual in enumerate(link.findall("visual")):
            mesh_node = visual.find("geometry/mesh")
            if mesh_node is None:
                continue
            mesh_path = URDF.parent / mesh_node.get("filename")
            mesh = trimesh.load_mesh(mesh_path, process=False)
            material = visual.find("material")
            material_name = material.get("name", "white") if material is not None else "white"
            color = material_colors.get(material_name, material_colors["white"])
            mesh.visual = trimesh.visual.ColorVisuals(
                mesh=mesh,
                vertex_colors=np.tile(np.array(color, dtype=np.uint8), (len(mesh.vertices), 1)),
            )
            transform = web_frame @ link_transform @ origin_matrix(visual.find("origin"))
            scene.add_geometry(mesh, node_name=f"{link_name}_{visual_index}", transform=transform)

    transformed_joints = []
    for index, joint in enumerate(movable, start=1):
        point = web_frame @ np.array([*joint["position"], 1.0])
        transformed_joints.append(
            {
                **joint,
                "index": index,
                "position": [round(float(value), 6) for value in point[:3]],
            }
        )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(scene.export(file_type="glb"))
    JOINTS_OUTPUT.write_text(json.dumps(transformed_joints, indent=2), encoding="utf-8")
    print(f"Exported {len(scene.geometry)} meshes to {OUTPUT} ({OUTPUT.stat().st_size / 1_048_576:.1f} MB)")
    print(f"Exported {len(transformed_joints)} movable joint hotspots to {JOINTS_OUTPUT}")
    print(f"Bounds: {scene.bounds.tolist()}")


if __name__ == "__main__":
    main()
