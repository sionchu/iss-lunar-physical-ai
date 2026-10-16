"""Offline URDF conditioning report for the Test B driven arm chains (no Isaac needed).

For each driven joint (arm joint0-3, both sides): child-link mass, the lightest descendant link, their mass ratio,
and the ratio of the child's smallest principal inertia to the smallest principal inertia among descendants.
Also lists movable joints whose q = 0 lies outside or within 1e-3 rad of the authored URDF limits.
Run: python scripts/r2_chain_ratios.py <urdf> [<urdf> ...] [--out report.json]
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def inertial(link):
    node = link.find("inertial")
    if node is None or node.find("mass") is None:
        return 0.0, None
    i = {k: float(node.find("inertia").get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    tensor = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]])
    return float(node.find("mass").get("value")), float(np.linalg.eigvalsh(tensor).min())


def report(urdf):
    robot = ET.parse(urdf).getroot()
    links = {l.get("name"): inertial(l) for l in robot.findall("link")}
    children = {}
    joints = {}
    for j in robot.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j.find("child").get("link"))
        joints[j.get("name")] = j

    def descendants(link):
        out = []
        for c in children.get(link, []):
            out += [c] + descendants(c)
        return out

    chains = []
    for side in ("left", "right"):
        for k in range(4):
            j = joints[f"r2/{side}_arm/joint{k}"]
            child = j.find("child").get("link")
            sub = [d for d in descendants(child) if links[d][0] > 0]
            light = min(sub, key=lambda d: links[d][0])
            min_i = min(sub, key=lambda d: links[d][1])
            m_c, i_c = links[child]
            chains.append({"joint": j.get("name"), "child": child, "child_mass_kg": m_c,
                           "subtree_mass_kg": m_c + sum(links[d][0] for d in sub), "subtree_links": len(sub) + 1,
                           "lightest_descendant": light, "lightest_mass_kg": links[light][0],
                           "mass_ratio": m_c / links[light][0],
                           "smallest_principal_inertia_descendant": min_i,
                           "inertia_ratio": i_c / links[min_i][1]})
    limits = []
    for name, j in joints.items():
        lim = j.find("limit")
        if j.get("type") in ("revolute", "prismatic") and lim is not None:
            lo, hi = float(lim.get("lower")), float(lim.get("upper"))
            state = "outside" if not lo <= 0.0 <= hi else "at_limit" if min(-lo, hi) <= 1e-3 else None
            if state:
                limits.append({"joint": name, "lower": lo, "upper": hi, "q0_state": state})
    masses = [m for m, _ in links.values() if m > 0]
    return {"urdf": str(urdf), "movable_joints": sum(j.get("type") != "fixed" for j in joints.values()),
            "links": len(links), "total_mass_kg": sum(masses), "min_link_mass_kg": min(masses),
            "max_link_mass_kg": max(masses),
            "min_principal_inertia_kg_m2": min(i for m, i in links.values() if m > 0),
            "driven_chains": chains, "q0_outside_or_at_limit": limits}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("urdf", nargs="+", type=Path)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    result = [report(u) for u in args.urdf]
    for r in result:
        print(f"{r['urdf']}: {r['movable_joints']} movable, {r['total_mass_kg']:.4f} kg, "
              f"link mass {r['min_link_mass_kg']:.4g}-{r['max_link_mass_kg']:.4g} kg")
        for c in r["driven_chains"]:
            print(f"  {c['joint']:<22} child {c['child_mass_kg']:.3f} kg  subtree {c['subtree_mass_kg']:.3f} kg "
                  f"({c['subtree_links']} links)  lightest {c['lightest_mass_kg']:.4g} kg ({c['lightest_descendant']})  "
                  f"mass ratio {c['mass_ratio']:.0f}  inertia ratio {c['inertia_ratio']:.0f}")
        for x in r["q0_outside_or_at_limit"]:
            print(f"  q0 {x['q0_state']:<8} {x['joint']} [{x['lower']}, {x['upper']}]")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
