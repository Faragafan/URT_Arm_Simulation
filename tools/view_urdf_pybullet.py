from __future__ import annotations

import argparse
import json
import math
import pathlib
import struct
import sys
import time
import xml.etree.ElementTree as ET


def import_pybullet():
    try:
        import pybullet as p  # type: ignore
        import pybullet_data  # type: ignore
    except ModuleNotFoundError as exc:
        print(
            "PyBullet is not installed.\n"
            "Install it with:\n"
            "  python -m pip install pybullet\n"
            "or run tools/run_urdf_viewer.ps1 after PyBullet is installed.",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    return p, pybullet_data


def parse_args() -> argparse.Namespace:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    default_urdf = repo_root / "assets" / "urdf" / "urdf_assembly_rigid_stl_collapsed.urdf"

    parser = argparse.ArgumentParser(description="Interactive PyBullet URDF viewer for the URT arm model.")
    parser.add_argument("--urdf", default=str(default_urdf), help="Path to the URDF file to load.")
    parser.add_argument("--fixed-base", action="store_true", default=True, help="Load robot with fixed base.")
    parser.add_argument(
        "--keep-collision",
        action="store_true",
        help="Keep collision meshes. By default, collisions are stripped for visual validation.",
    )
    parser.add_argument(
        "--mesh-only",
        action="store_true",
        help="Display all visual STL meshes as independent fixed objects instead of loading the URDF joints.",
    )
    parser.add_argument(
        "--box-visuals",
        action="store_true",
        help="Replace STL visuals with bounding boxes in the generated PyBullet URDF.",
    )
    parser.add_argument(
        "--mesh-dir",
        default="",
        help="Directory containing STL files to use instead of the mesh paths in the source URDF.",
    )
    parser.add_argument(
        "--hide-joint-axes",
        action="store_true",
        help="Do not draw revolute/prismatic joint axes in the PyBullet view.",
    )
    parser.add_argument("--scale-debug", action="store_true", help="Print joint/link metadata after loading.")
    return parser.parse_args()


def finite_or_default(value: float | None, default: float) -> float:
    if value is None or not math.isfinite(value):
        return default
    return value


def read_binary_stl_bounds(path: pathlib.Path) -> tuple[list[float], list[float]] | None:
    data = path.read_bytes()
    if len(data) < 84:
        return None

    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if expected_size > len(data):
        return None

    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack("<12f", data[offset : offset + 48])
        for value_index in (3, 6, 9):
            x, y, z = values[value_index : value_index + 3]
            mins[0] = min(mins[0], x)
            mins[1] = min(mins[1], y)
            mins[2] = min(mins[2], z)
            maxs[0] = max(maxs[0], x)
            maxs[1] = max(maxs[1], y)
            maxs[2] = max(maxs[2], z)
        offset += 50
    return mins, maxs


def read_cached_stl_bounds(path: pathlib.Path) -> tuple[list[float], list[float]] | None:
    cache_path = path.parent.parent / "urdf" / "mesh_bounds.json"
    if cache_path.exists():
        try:
            cache = json.loads(cache_path.read_text(encoding="utf-8"))
            entry = cache.get(path.name)
            stat = path.stat()
            if (
                entry
                and entry.get("size_bytes") == stat.st_size
                and entry.get("mtime_ns") == stat.st_mtime_ns
                and "min" in entry
                and "max" in entry
            ):
                return [float(value) for value in entry["min"]], [float(value) for value in entry["max"]]
        except (OSError, ValueError, TypeError):
            pass

    return read_binary_stl_bounds(path)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotate_vector(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def format_vector(values: list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def normalize_vector(values: tuple[float, float, float] | list[float]) -> list[float]:
    length = math.sqrt(sum(value * value for value in values))
    if length <= 0.0:
        return [0.0, 0.0, 0.0]
    return [value / length for value in values]


def compute_mesh_camera_target(urdf_path: pathlib.Path) -> tuple[list[float], float]:
    root = ET.parse(urdf_path).getroot()
    urdf_dir = urdf_path.parent
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    found = False

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_path = (urdf_dir / filename).resolve()
        bounds = read_cached_stl_bounds(mesh_path)
        if bounds is None:
            continue
        scale_values = [1.0, 1.0, 1.0]
        if "scale" in mesh.attrib:
            scale_values = [float(value) for value in mesh.attrib["scale"].split()]
        mesh_min, mesh_max = bounds
        for axis in range(3):
            mins[axis] = min(mins[axis], mesh_min[axis] * scale_values[axis])
            maxs[axis] = max(maxs[axis], mesh_max[axis] * scale_values[axis])
        found = True

    if not found:
        return [0.0, 0.0, 0.3], 1.8

    center = [(mins[i] + maxs[i]) * 0.5 for i in range(3)]
    size = [maxs[i] - mins[i] for i in range(3)]
    distance = max(size) * 2.2
    return center, max(distance, 0.8)


def prepare_pybullet_urdf(
    source_urdf: pathlib.Path,
    keep_collision: bool,
    box_visuals: bool,
    mesh_dir: pathlib.Path | None = None,
) -> pathlib.Path:
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    urdf_dir = source_urdf.parent

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_path = mesh_dir / pathlib.Path(filename).name if mesh_dir else urdf_dir / filename
        mesh_path = mesh_path.resolve()
        mesh.attrib["filename"] = mesh_path.as_posix()

    if box_visuals:
        for visual in root.findall(".//visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            mesh_path = pathlib.Path(mesh.attrib["filename"])
            bounds = read_cached_stl_bounds(mesh_path)
            if bounds is None:
                continue

            scale = [1.0, 1.0, 1.0]
            if "scale" in mesh.attrib:
                scale = [float(value) for value in mesh.attrib["scale"].split()]

            mesh_min, mesh_max = bounds
            center = [((mesh_min[i] + mesh_max[i]) * 0.5) * scale[i] for i in range(3)]
            size = [(mesh_max[i] - mesh_min[i]) * scale[i] for i in range(3)]

            origin = visual.find("origin")
            if origin is None:
                origin = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
                visual.insert(0, origin)

            origin_xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
            origin_rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]
            rotated_center = rotate_vector(rpy_matrix(*origin_rpy), center)
            origin.attrib["xyz"] = format_vector([origin_xyz[i] + rotated_center[i] for i in range(3)])
            origin.attrib["rpy"] = format_vector(origin_rpy)

            geometry = visual.find("geometry")
            if geometry is None:
                continue
            geometry.clear()
            ET.SubElement(geometry, "box", {"size": format_vector(size)})

            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material", {"name": "debug_box"})
                ET.SubElement(material, "color", {"rgba": "0.2 0.55 0.95 0.55"})

    if not keep_collision:
        for link in root.findall("link"):
            for collision in list(link.findall("collision")):
                link.remove(collision)

    out_dir = source_urdf.resolve().parents[1] / "pybullet"
    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = "boxes_pybullet" if box_visuals else "pybullet"
    out_path = out_dir / f"{source_urdf.stem}_{suffix}.urdf"
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def load_mesh_only_view(p, urdf_path: pathlib.Path) -> None:
    root = ET.parse(urdf_path).getroot()
    urdf_dir = urdf_path.parent
    loaded = 0

    for mesh in root.findall(".//visual/geometry/mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_path = (urdf_dir / filename).resolve()
        scale = [1.0, 1.0, 1.0]
        if "scale" in mesh.attrib:
            scale = [float(value) for value in mesh.attrib["scale"].split()]

        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(mesh_path),
            meshScale=scale,
            rgbaColor=[0.65, 0.72, 0.78, 1.0],
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
        )
        loaded += 1
        print(f"  mesh-only loaded: {mesh_path.name}, scale={scale}")

    print(f"Mesh-only objects loaded: {loaded}")


def update_joint_axis_debug(p, robot_id: int, items: list[tuple[int, int]], length: float = 0.06) -> list[tuple[int, int]]:
    updated: list[tuple[int, int]] = []
    for joint_index, line_id in items:
        info = p.getJointInfo(robot_id, joint_index)
        joint_axis = normalize_vector(info[13])
        parent_frame_pos = info[14]
        parent_frame_orn = info[15]
        parent_index = info[16]

        if parent_index == -1:
            parent_pos, parent_orn = p.getBasePositionAndOrientation(robot_id)
        else:
            parent_state = p.getLinkState(robot_id, parent_index)
            parent_pos, parent_orn = parent_state[4], parent_state[5]

        joint_pos, joint_orn = p.multiplyTransforms(parent_pos, parent_orn, parent_frame_pos, parent_frame_orn)
        axis_end_offset, _ = p.multiplyTransforms([0, 0, 0], joint_orn, joint_axis, [0, 0, 0, 1])
        line_end = [joint_pos[i] + axis_end_offset[i] * length for i in range(3)]
        line_id = p.addUserDebugLine(
            joint_pos,
            line_end,
            [1.0, 0.1, 0.1],
            lineWidth=4,
            lifeTime=0,
            replaceItemUniqueId=line_id,
        )
        updated.append((joint_index, line_id))
    return updated


def main() -> int:
    args = parse_args()
    urdf_path = pathlib.Path(args.urdf).resolve()
    if not urdf_path.exists():
        print(f"URDF not found: {urdf_path}", file=sys.stderr)
        return 1
    mesh_dir = pathlib.Path(args.mesh_dir).resolve() if args.mesh_dir else None
    if mesh_dir is not None and not mesh_dir.exists():
        print(f"Mesh dir not found: {mesh_dir}", file=sys.stderr)
        return 1

    pybullet_urdf_path = prepare_pybullet_urdf(
        urdf_path,
        keep_collision=args.keep_collision,
        box_visuals=args.box_visuals,
        mesh_dir=mesh_dir,
    )
    p, pybullet_data = import_pybullet()

    client = p.connect(p.GUI)
    if client < 0:
        print("Failed to connect to PyBullet GUI.", file=sys.stderr)
        return 1

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    camera_target, camera_distance = compute_mesh_camera_target(urdf_path)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=camera_target,
    )

    p.loadURDF("plane.urdf")
    if args.mesh_only:
        load_mesh_only_view(p, urdf_path)
        print("\nMesh-only mode. Close the window to exit.")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
        finally:
            if p.isConnected():
                p.disconnect()
        return 0

    try:
        robot_id = p.loadURDF(str(pybullet_urdf_path), useFixedBase=args.fixed_base, flags=p.URDF_USE_INERTIA_FROM_FILE)
    except Exception:
        print(f"Failed to load source URDF: {urdf_path}", file=sys.stderr)
        print(f"Failed to load PyBullet URDF: {pybullet_urdf_path}", file=sys.stderr)
        print("Close the PyBullet window before retrying.", file=sys.stderr)
        raise

    sliders: list[tuple[int, int]] = []
    joint_axis_lines: list[tuple[int, int]] = []
    print(f"Loaded source URDF: {urdf_path}")
    print(f"Loaded PyBullet URDF: {pybullet_urdf_path}")
    print(f"Camera target: {camera_target}, distance: {camera_distance:.3f}")
    print("Joints:")

    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        lower = finite_or_default(info[8], -math.pi)
        upper = finite_or_default(info[9], math.pi)

        if joint_type == p.JOINT_REVOLUTE:
            if lower >= upper:
                lower, upper = -math.pi, math.pi
            slider_id = p.addUserDebugParameter(joint_name, lower, upper, 0.0)
            sliders.append((joint_index, slider_id))
            if not args.hide_joint_axes:
                joint_axis_lines.append((joint_index, -1))
            print(f"  [{joint_index}] revolute {joint_name}: {lower:.4f} .. {upper:.4f}, axis={info[13]}")
        else:
            print(f"  [{joint_index}] type={joint_type} {joint_name}")

    if args.scale_debug:
        print("\nLink states at zero pose:")
        for joint_index in range(-1, p.getNumJoints(robot_id)):
            if joint_index == -1:
                pos, orn = p.getBasePositionAndOrientation(robot_id)
                print(f"  base: pos={pos}, orn={orn}")
            else:
                state = p.getLinkState(robot_id, joint_index)
                print(f"  link via joint {joint_index}: pos={state[4]}, orn={state[5]}")

    print("\nUse the PyBullet sliders to move each joint. Close the window to exit.")

    try:
        while p.isConnected():
            for joint_index, slider_id in sliders:
                value = p.readUserDebugParameter(slider_id)
                p.resetJointState(robot_id, joint_index, value)
            if joint_axis_lines:
                joint_axis_lines = update_joint_axis_debug(p, robot_id, joint_axis_lines)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    finally:
        if p.isConnected():
            p.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
