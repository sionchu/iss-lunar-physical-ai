"""Build the documented ISS zero-g R2 derivative from the untouched NASA URDF."""

import hashlib
import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/raw/nasa-urdf-robots/r2_description/robots/r2c6.urdf"
DEST = ROOT / "outputs/r2_derived/r2c6_zero_g.urdf"
CHANGES = DEST.with_name("changes.json")
PACKAGE = SOURCE.parents[1]
KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
SUPPORT_LINK = "r2/objects/argos_support/argos_support_link"
SUPPORT_JOINT = "r2/objects/argos_support/argos_support_joint"
WORLD_LINK = "r2/world_ref"
WORLD_JOINT = "r2/fixed/world_ref/robot_world"


def stl_vertices(path):
    data = path.read_bytes()
    if len(data) >= 84 and 84 + 50 * struct.unpack_from("<I", data, 80)[0] == len(data):
        count = struct.unpack_from("<I", data, 80)[0]
        for i in range(count):
            for j in range(3):
                yield struct.unpack_from("<fff", data, 84 + 50 * i + 12 + 12 * j)
    else:
        for line in data.decode("utf-8", errors="replace").splitlines():
            words = line.strip().split()
            if len(words) == 4 and words[0].lower() == "vertex":
                yield tuple(float(x) for x in words[1:])


def dae_vertices(path):
    root = ET.parse(path).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    for mesh in root.findall(".//c:geometry/c:mesh", ns):
        sources = {}
        for source in mesh.findall("c:source", ns):
            array = source.find("c:float_array", ns)
            accessor = source.find("c:technique_common/c:accessor", ns)
            if array is None or accessor is None or not array.text:
                continue
            values = [float(x) for x in array.text.split()]
            stride = int(accessor.get("stride", "1"))
            offset = int(accessor.get("offset", "0"))
            count = int(accessor.get("count", "0"))
            sources["#" + source.get("id")] = [tuple(values[offset + i * stride:offset + i * stride + 3]) for i in range(count)]
        positions = set()
        for vertices in mesh.findall("c:vertices", ns):
            for input_node in vertices.findall("c:input", ns):
                if input_node.get("semantic") == "POSITION":
                    positions.add(input_node.get("source"))
        # Only POSITION sources define spatial extent; normals and UVs do not.
        for source in sorted(positions):
            for vertex in sources.get(source, []):
                if len(vertex) == 3:
                    yield vertex


def mesh_bbox(link):
    for kind in ("collision", "visual"):
        vertices = []
        paths = []
        for node in link.findall(kind):
            mesh = node.find("geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.get("filename", "")
            if not uri.startswith("package://r2_description/"):
                continue
            path = PACKAGE / uri.split("package://r2_description/", 1)[1]
            if not path.is_file():
                continue
            scale = [float(x) for x in mesh.get("scale", "1 1 1").split()]
            if len(scale) != 3:
                continue
            try:
                raw = stl_vertices(path) if path.suffix.lower() == ".stl" else dae_vertices(path) if path.suffix.lower() == ".dae" else ()
                current = [tuple(p[i] * scale[i] for i in range(3)) for p in raw]
            except (OSError, ET.ParseError, ValueError, struct.error):
                continue
            if current and all(len(p) == 3 and all(math.isfinite(x) for x in p) for p in current):
                vertices.extend(current)
                paths.append(str(path.relative_to(ROOT)).replace("\\", "/"))
        if vertices:
            low = [min(p[i] for p in vertices) for i in range(3)]
            high = [max(p[i] for p in vertices) for i in range(3)]
            size = [high[i] - low[i] for i in range(3)]
            if all(x > 0 for x in size):
                return {"min_m": low, "max_m": high, "size_m": size, "meshes": paths, "source": kind}
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remove-world-ref", action="store_true",
                        help="also remove massless world_ref and its fixed joint; emit a separate floating-root derivative")
    args = parser.parse_args()
    dest = (DEST.with_name("r2c6_zero_g_noworldref.urdf") if args.remove_world_ref else DEST)
    changes = dest.with_name("changes_noworldref.json" if args.remove_world_ref else "changes.json")
    tree = ET.parse(SOURCE)
    robot = tree.getroot()
    removed = []
    for tag, name in (("joint", SUPPORT_JOINT), ("link", SUPPORT_LINK)):
        node = next((x for x in robot.findall(tag) if x.get("name") == name), None)
        if node is None:
            raise ValueError(f"Expected {tag} absent: {name}")
        if tag == "joint" and (node.get("type") != "fixed" or node.find("parent").get("link") != "r2/pelvis" or node.find("child").get("link") != SUPPORT_LINK):
            raise ValueError("ARGOS joint structure changed")
        if tag == "link" and any(x.find("parent") is not None and x.find("parent").get("link") == SUPPORT_LINK for x in robot.findall("joint")):
            raise ValueError("ARGOS support is not a leaf")
        robot.remove(node)
        removed.append({"type": tag, "name": name})
    if args.remove_world_ref:
        joint = next((x for x in robot.findall("joint") if x.get("name") == WORLD_JOINT), None)
        link = next((x for x in robot.findall("link") if x.get("name") == WORLD_LINK), None)
        if (joint is None or link is None or joint.get("type") != "fixed"
                or joint.find("parent").get("link") != WORLD_LINK
                or joint.find("child").get("link") != "r2/robot_world"
                or link.find("inertial") is not None
                or any(j is not joint and j.find("parent") is not None
                       and j.find("parent").get("link") == WORLD_LINK for j in robot.findall("joint"))):
            raise ValueError("world_ref structure changed; inspect source before removing")
        robot.remove(joint)
        robot.remove(link)
        removed.extend(({"type": "joint", "name": WORLD_JOINT},
                        {"type": "link", "name": WORLD_LINK}))
    edits = []
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None or inertial.find("inertia") is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        inertia = inertial.find("inertia")
        old = {k: float(inertia.get(k)) for k in KEYS}
        if not (mass > 0 and old["ixx"] == old["iyy"] == old["izz"] and old["ixy"] == old["ixz"] == old["iyz"] == 0 and old["ixx"] > 0):
            continue
        k = math.sqrt(old["ixx"] / mass)
        if k <= 0.3:
            continue
        bbox = mesh_bbox(link)
        if bbox is None:
            value = 0.4 * mass * 0.03 ** 2
            new = dict(zip(KEYS, (value, 0.0, 0.0, value, 0.0, value)))
            method = "fallback_sphere"
        else:
            sx, sy, sz = bbox["size_m"]
            new = dict(zip(KEYS, (mass * (sy * sy + sz * sz) / 12, 0.0, 0.0,
                                   mass * (sx * sx + sz * sz) / 12, 0.0,
                                   mass * (sx * sx + sy * sy) / 12)))
            method = "collision_mesh_box" if bbox["source"] == "collision" else "visual_mesh_box"
        for key, value in new.items():
            inertia.set(key, format(value, ".17g"))
        edits.append({"link": link.get("name"), "mass_kg": mass, "old_k_m": k,
                      "old_tensor_kg_m2": old, "new_tensor_kg_m2": new,
                      "method": method, "bbox": bbox})
    dest.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(dest, encoding="utf-8", xml_declaration=True)
    result = {"source": str(SOURCE.relative_to(ROOT)).replace("\\", "/"),
              "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              "derivative": str(dest.relative_to(ROOT)).replace("\\", "/"),
              "date": "2026-09-30", "removed": removed, "inertia_changes": edits,
              "remove_world_ref": args.remove_world_ref,
              "inertia_change_count": len(edits),
              "fallback_sphere_count": sum(x["method"] == "fallback_sphere" for x in edits)}
    changes.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Removed {len(removed)} ARGOS elements; changed {len(edits)} inertias; fallback spheres {result['fallback_sphere_count']}")
    print(f"URDF: {dest}\nChanges: {changes}")


if __name__ == "__main__":
    main()
