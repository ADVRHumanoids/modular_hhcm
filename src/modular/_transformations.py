"""
Minimal drop-in replacement for tf.transformations / tf_transformations.

Implements only the subset of functions used in this codebase using numpy and
scipy, which are already required dependencies - no additional packages needed.

Supported Euler convention: 'sxyz' (static/extrinsic xyz), which is the only
one used in this codebase. scipy uses uppercase letters for extrinsic rotations,
so 'sxyz' maps to scipy's 'XYZ'.
"""
import numpy as np
from functools import reduce
from scipy.spatial.transform import Rotation



def identity_matrix():
    """Return 4x4 identity matrix."""
    return np.eye(4)


def translation_matrix(direction):
    """Return 4x4 homogeneous matrix to translate by direction vector."""
    M = np.eye(4)
    M[:3, 3] = direction[:3]
    return M


def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return 4x4 homogeneous rotation matrix from Euler angles and axis sequence.

    Only 'sxyz' (static/extrinsic xyz) is supported.
    """
    if axes != 'sxyz':
        raise NotImplementedError(
            f"Euler convention '{axes}' not supported; only 'sxyz' is implemented."
        )
    # 'sxyz' = static/extrinsic xyz = scipy lowercase 'xyz'
    # NOTE: in scipy, UPPERCASE = intrinsic, lowercase = extrinsic (unusual convention)
    R = Rotation.from_euler('xyz', [ai, aj, ak]).as_matrix()
    M = np.eye(4)
    M[:3, :3] = R
    return M


def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    Only 'sxyz' (static/extrinsic xyz) is supported.
    Returns (roll, pitch, yaw).
    """
    if axes != 'sxyz':
        raise NotImplementedError(
            f"Euler convention '{axes}' not supported; only 'sxyz' is implemented."
        )
    # 'sxyz' = static/extrinsic xyz = scipy lowercase 'xyz'
    # NOTE: in scipy, UPPERCASE = intrinsic, lowercase = extrinsic (unusual convention)
    R = np.array(matrix)[:3, :3]
    angles = Rotation.from_matrix(R).as_euler('xyz')
    return float(angles[0]), float(angles[1]), float(angles[2])


def rotation_matrix(angle, direction, point=None):
    """Return 4x4 homogeneous matrix to rotate about axis through origin (or point).

    angle:     rotation angle in radians
    direction: axis direction vector (need not be unit length)
    point:     point on the rotation axis (default: origin)
    """
    direction = np.array(direction[:3], dtype=np.float64)
    norm = np.linalg.norm(direction)
    if norm == 0.0:
        raise ValueError("Direction vector has zero length")
    direction = direction / norm
    R = Rotation.from_rotvec(angle * direction).as_matrix()
    M = np.eye(4)
    M[:3, :3] = R
    if point is not None:
        point = np.array(point[:3], dtype=np.float64)
        M[:3, 3] = point - R @ point
    return M


def concatenate_matrices(*matrices):
    """Return concatenation (product) of a series of 4x4 transformation matrices."""
    return reduce(np.dot, matrices)


def inverse_matrix(matrix):
    """Return the inverse of a 4x4 transformation matrix."""
    return np.linalg.inv(matrix)
