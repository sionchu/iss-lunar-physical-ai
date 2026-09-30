"""Static, stdlib-only audit of a supplied Robonaut 2 URDF."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "assets/raw/nasa-urdf-robots/r2_description/robots/r2c6.urdf"
INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")


def principal_moments(values):
    """Jacobi eigenvalues for a symmetric 3x3 inertia tensor."""
    xx, xy, xz, yy, yz, zz = values
    a = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    for _ in range(40):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) < 1e-13 * max(1.0, *(abs(a[i][i]) for i in range(3))):
            break
        angle = 0.5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c * c * app - 2 * c * s * apq + s * s * aqq
        a[q][q] = s * s * app + 2 * c * s * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0
        for k in range(3):
            if k not in (p, q):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = a[p][k] = c * akp - s * akq
                a[k][q] = a[q][k] = s * akp + c * akq
    return sorted(a[i][i] for i in range(3))


def geometry_types(link, tag):
    result = []
    for node in link.findall(tag):
        geometry = node.find("geometry")
        if geometry is None:
            result.append("missing_geometry")
        else:
            result.extend(child.tag for child in geometry)
    return result


def group_for_joint(name):
    parts = name.split("/")
    if len(parts) < 3 or parts[1] == "fixed":
        return "other/fixed"
    if parts[1] in ("left_arm", "right_arm"):
        return parts[1] + ("/hand_fingers" if any(x in name for x in ("finger", "thumb", "index", "middle", "ring", "pinky")) else "")
    if parts[1] in ("left_leg", "right_leg"):
        return parts[1] + ("/end_effector" if any(x in name for x in ("foot", "gripper", "jaw")) else "")
    return parts[1]


def audit(path):
    robot = ET.parse(path).getroot()
    links = []
    signatures = defaultdict(list)
    for link in robot.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        mass_node = inertial.find("mass") if inertial is not None else None
        origin = inertial.find("origin") if inertial is not None else None
        inertia_node = inertial.find("inertia") if inertial is not None else None
        mass = float(mass_node.get("value")) if mass_node is not None else None
        com = [float(x) for x in origin.get("xyz", "0 0 0").split()] if origin is not None else [0.0, 0.0, 0.0]
        values = [float(inertia_node.get(k)) for k in INERTIA_KEYS] if inertia_node is not None else None
        flags = []
        if inertial is None or mass is None or values is None:
            flags.append("missing_inertial")
        if mass is not None and mass <= 0:
            flags.append("non_positive_mass")
        moments = None
        if values is not None:
            moments = principal_moments(values)
            scale = max(abs(x) for x in moments)
            tol = max(1e-12, scale * 1e-8)
            if moments[0] <= tol:
                flags.append("non_positive_definite")
            if moments[2] > moments[0] + moments[1] + tol:
                flags.append("triangle_inequality")
            if all(abs(values[i]) <= tol for i in (1, 2, 4)) and abs(values[0] - values[3]) <= tol and abs(values[3] - values[5]) <= tol:
                flags.append("identity_placeholder_candidate")
            signatures[tuple(values)].append(name)
        links.append({"name": name, "mass_kg": mass, "com_xyz_m": com,
                      "inertia_kg_m2": dict(zip(INERTIA_KEYS, values)) if values is not None else None,
                      "principal_moments_kg_m2": moments,
                      "visual_geometry": geometry_types(link, "visual"),
                      "collision_geometry": geometry_types(link, "collision"), "flags": flags})
    repeated = {str(key): names for key, names in signatures.items() if len(names) >= 3}
    for item in links:
        if item["inertia_kg_m2"] is not None and len(signatures[tuple(item["inertia_kg_m2"][k] for k in INERTIA_KEYS)]) >= 3:
            item["flags"].append("inertia_repeated_3plus")
    groups = defaultdict(list)
    joints = []
    for joint in robot.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        record = {"name": name, "type": joint.get("type"), "parent": joint.find("parent").get("link"),
                  "child": joint.find("child").get("link"), "group": group_for_joint(name),
                  "limit": {k: float(v) for k, v in limit.attrib.items()} if limit is not None else None}
        joints.append(record)
        groups[record["group"]].append(record)
    count_flags = Counter(flag for link in links for flag in link["flags"])
    missing_or_placeholder = sum("missing_inertial" in x["flags"] or "identity_placeholder_candidate" in x["flags"] for x in links)
    return {"urdf": str(path), "robot_name": robot.get("name"), "link_count": len(links),
            "joint_count": len(joints), "total_urdf_mass_kg": sum(x["mass_kg"] or 0 for x in links),
            "missing_or_identity_placeholder_count": missing_or_placeholder,
            "missing_or_identity_placeholder_fraction": missing_or_placeholder / len(links),
            "flag_counts": dict(count_flags), "repeated_inertias_3plus": repeated,
            "joint_group_counts": {k: len(v) for k, v in sorted(groups.items())},
            "links": links, "joints": joints}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--json", type=Path, default=ROOT / "outputs/r2_urdf_audit.json")
    args = parser.parse_args()
    result = audit(args.urdf.resolve())
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"URDF: {result['urdf']}\nLinks: {result['link_count']}  Joints: {result['joint_count']}  Mass: {result['total_urdf_mass_kg']:.6f} kg")
    print(f"Missing/identity-placeholder: {result['missing_or_identity_placeholder_count']}/{result['link_count']} ({result['missing_or_identity_placeholder_fraction']:.1%})")
    print("Flags:", result["flag_counts"])
    print("\nLINK | MASS kg | COM xyz m | INERTIA (ixx,ixy,ixz,iyy,iyz,izz) kg m2 | VISUAL | COLLISION | FLAGS")
    for x in result["links"]:
        tensor = x["inertia_kg_m2"]
        vals = "-" if tensor is None else ",".join(f"{tensor[k]:.5g}" for k in INERTIA_KEYS)
        print(f"{x['name']} | {x['mass_kg']} | {x['com_xyz_m']} | {vals} | {','.join(x['visual_geometry']) or '-'} | {','.join(x['collision_geometry']) or '-'} | {','.join(x['flags']) or '-'}")
    print("\nJOINT GROUP COUNTS:", result["joint_group_counts"])
    print("JOINT | TYPE | GROUP | LOWER | UPPER | EFFORT | VELOCITY")
    for x in result["joints"]:
        lim = x["limit"] or {}
        print(f"{x['name']} | {x['type']} | {x['group']} | {lim.get('lower', '-')} | {lim.get('upper', '-')} | {lim.get('effort', '-')} | {lim.get('velocity', '-')}")
    print("JSON:", args.json)


if __name__ == "__main__":
    try:
        main()
    except (ET.ParseError, OSError, ValueError) as exc:
        print(f"AUDIT_FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
