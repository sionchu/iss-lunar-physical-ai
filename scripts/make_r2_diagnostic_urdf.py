"""Create isolated R2 URDF joint-lock variants from the gate derivative."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "outputs/r2_derived/r2c6_zero_g.urdf"
OUT = ROOT / "outputs/diagnostics"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", choices=("v6_lock_fingers", "v7_lock_leg_grippers"))
    args = parser.parse_args()
    tree = ET.parse(SOURCE)
    joints = []
    for joint in tree.getroot().findall("joint"):
        name = joint.get("name", "")
        selected = ("/hand/" in name if args.variant == "v6_lock_fingers" else "_leg/gripper/" in name)
        if selected and joint.get("type") != "fixed":
            joint.set("type", "fixed")
            for tag in ("axis", "limit", "dynamics", "safety_controller", "calibration", "mimic"):
                node = joint.find(tag)
                if node is not None:
                    joint.remove(node)
            joints.append(name)
    if not joints:
        raise RuntimeError("No expected movable joints selected")
    OUT.mkdir(parents=True, exist_ok=True)
    target = OUT / (args.variant + ".urdf")
    ET.indent(tree, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=True)
    (OUT / (args.variant + "_changes.json")).write_text(
        json.dumps({"source": str(SOURCE), "variant": args.variant,
                    "single_factor": "selected joint types changed to fixed", "joint_names": joints}, indent=2) + "\n",
        encoding="utf-8")
    print(f"{target}: fixed {len(joints)} joints")


if __name__ == "__main__":
    main()
