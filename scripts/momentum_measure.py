"""Shared PhysX tensor-view momentum measurement for ISS harness cases."""

import numpy as np


def rotation(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def measure(bodies, masses, inertias, coms, names):
    poses, quats = (x.numpy().astype(np.float64) for x in bodies.get_world_poses())
    linear, angular = (x.numpy().astype(np.float64) for x in bodies.get_velocities())
    positions = []
    velocities = []
    spins = []
    for i in range(len(masses)):
        rb = rotation(quats[i])
        offset = rb @ coms[i]
        positions.append(poses[i] + offset)
        # PhysX RigidPrim tensor linear velocity is already at the mass center.
        # Adding omega x COM offset double-counts transport for off-origin COMs.
        velocities.append(linear[i])
        # Tensor inertia is already expressed in the link body frame, including
        # the principal-axis orientation from the authored mass properties.
        spins.append(rb @ inertias[i] @ rb.T @ angular[i])
    positions = np.asarray(positions)
    velocities = np.asarray(velocities)
    center = np.sum(masses[:, None] * positions, axis=0) / masses.sum()
    terms_p = masses[:, None] * velocities
    terms_l = np.cross(positions - center, terms_p) + spins
    return {"P": terms_p.sum(axis=0), "L": terms_l.sum(axis=0),
            "Pref": float(np.linalg.norm(terms_p, axis=1).sum()),
            "Lref": float(np.linalg.norm(terms_l, axis=1).sum()),
            "largest_P_link": names[int(np.argmax(np.linalg.norm(terms_p, axis=1)))],
            "largest_L_link": names[int(np.argmax(np.linalg.norm(terms_l, axis=1)))]}
