"""Quaternion helpers. q = [q0, q1, q2, q3], q0 is the scalar part, Hamilton convention.

Filter convention: Qq(q) rotates a world vector into the body (sensor) frame,
so a sensor reads  y = Qq(q) @ v_world.  The phone's own orientation output uses
the opposite convention (see docs/conventions.md) and is conjugated on load.
"""
import numpy as np


def Sq(q):
    """4x3 matrix S(q): dq/dt = 0.5 * S(q) @ omega_body."""
    q0, q1, q2, q3 = q
    return np.array([[-q1, -q2, -q3], [q0, -q3, q2], [q3, q0, -q1], [-q2, q1, q0]])


def Somega(w):
    """4x4 matrix S(omega): dq/dt = 0.5 * S(omega) @ q."""
    x, y, z = w
    return np.array([[0, -x, -y, -z], [x, 0, z, -y], [y, -z, 0, x], [z, y, -x, 0]])


def Somega_wb(w):
    """4x4 matrix with dq/dt = 0.5 * Somega_wb(w) @ q for q = world->body (the filter convention).

    Equals -L(omega), L = left-multiplication by the pure quaternion omega. The course-style
    Somega above is the *right*-multiplication matrix, valid for q = body->world only.
    """
    x, y, z = w
    return np.array([[0, x, y, z], [-x, 0, z, -y], [-y, -z, 0, x], [-z, y, -x, 0]])


def Sq_wb(q):
    """4x3 matrix with  omega (x) q = Sq_wb(q) @ omega  (noise input matrix for Somega_wb)."""
    q0, q1, q2, q3 = q
    return np.array([[-q1, -q2, -q3], [q0, q3, -q2], [-q3, q0, q1], [q2, -q1, q0]])


def Qq(q):
    """3x3 rotation matrix of a unit quaternion."""
    q0, q1, q2, q3 = q
    return np.array([
        [2 * (q0**2 + q1**2) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 2 * (q0**2 + q2**2) - 1, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0**2 + q3**2) - 1]])


def dQqdq(q):
    """[dQ/dq0, dQ/dq1, dQ/dq2, dQ/dq3], four 3x3 matrices (Jacobian of Q(q) v w.r.t. q)."""
    q0, q1, q2, q3 = q
    return [2 * np.array(a, dtype=float) for a in (
        [[2 * q0, -q3, q2], [q3, 2 * q0, -q1], [-q2, q1, 2 * q0]],
        [[2 * q1, q2, q3], [q2, 0, -q0], [q3, q0, 0]],
        [[0, q1, q0], [q1, 2 * q2, q3], [-q0, q3, 0]],
        [[0, -q0, q1], [q0, 0, q2], [q1, q2, 2 * q3]])]


def normalize(q):
    """Renormalise and fix the sign so q0 >= 0 (q and -q are the same rotation)."""
    q = q / np.linalg.norm(q)
    return -q if q[0] < 0 else q


# ---- batch helpers, arrays of shape (4, N) ---------------------------------

def conj(q):
    return q * np.array([1.0, -1, -1, -1])[:, None]


def qmul(a, b):
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    return np.array([a0*b0 - a1*b1 - a2*b2 - a3*b3, a0*b1 + a1*b0 + a2*b3 - a3*b2,
                     a0*b2 - a1*b3 + a2*b0 + a3*b1, a0*b3 + a1*b2 - a2*b1 + a3*b0])


def to_rotmat(q):
    """(4, N) -> (N, 3, 3), same as Qq for each column."""
    q0, q1, q2, q3 = q
    R = np.empty((q.shape[1], 3, 3))
    R[:, 0, 0] = 2*(q0**2 + q1**2) - 1; R[:, 0, 1] = 2*(q1*q2 - q0*q3); R[:, 0, 2] = 2*(q1*q3 + q0*q2)
    R[:, 1, 0] = 2*(q1*q2 + q0*q3); R[:, 1, 1] = 2*(q0**2 + q2**2) - 1; R[:, 1, 2] = 2*(q2*q3 - q0*q1)
    R[:, 2, 0] = 2*(q1*q3 - q0*q2); R[:, 2, 1] = 2*(q2*q3 + q0*q1); R[:, 2, 2] = 2*(q0**2 + q3**2) - 1
    return R


def error_decomposition(q_est, q_ref):
    """Split the rotation between two orientations (both in filter convention).

    Returns (total, tilt, heading) in degrees, each of shape (N,).
      total   : geodesic angle between the two orientations
      tilt    : angle between the two 'up' directions (roll/pitch error, needs no magnetometer)
      heading : signed rotation about vertical (yaw error), wrapped to [-180, 180]
    Swing-twist split of  conj(q_est) (x) q_ref  about the world z axis.
    """
    e = qmul(conj(q_est), q_ref)
    total = 2 * np.arccos(np.clip(np.abs(e[0]), 0, 1))
    tilt = 2 * np.arccos(np.clip(np.hypot(e[0], e[3]), 0, 1))
    heading = 2 * np.arctan2(e[3], e[0])
    heading = (heading + np.pi) % (2 * np.pi) - np.pi
    return np.degrees(total), np.degrees(tilt), np.degrees(heading)


def to_euler_zyx(q):
    """Body->world yaw, pitch, roll [deg] of filter-convention quaternions (4, N)."""
    R = np.transpose(to_rotmat(q), (0, 2, 1))  # body -> world
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    pitch = np.arcsin(np.clip(-R[:, 2, 0], -1, 1))
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    return np.degrees(np.array([roll, pitch, yaw]))
