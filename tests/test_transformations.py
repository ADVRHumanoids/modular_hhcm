"""
Unit tests for modular._transformations

Verifies that the internal scipy/numpy-based implementation produces results
numerically consistent with the tf/tf_transformations API it replaces.
Run with:  pytest tests/test_transformations.py
"""
import math
import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Optional: cross-validate against the original ROS tf_transformations library
# when it is available in the environment.
# ---------------------------------------------------------------------------
try:
    import tf
    _ref = tf.transformations
except ImportError:
    try:
        import tf_transformations as _ref
    except ImportError:
        _ref = None

_has_ref = pytest.mark.skipif(
    _ref is None,
    reason="tf / tf_transformations not installed; skipping cross-validation"
)


@pytest.fixture(scope="session", autouse=True)
def _warn_if_no_ref():
    """Emit a single session-level warning when the ROS reference library is
    absent so the user knows cross-validation tests are not being executed."""
    if _ref is None:
        import warnings
        warnings.warn(
            "tf / tf_transformations not found: cross-validation tests are "
            "SKIPPED.  Install the ROS package (ros-$ROS_DISTRO-tf-transformations) "
            "and re-run to enable them.",
            UserWarning,
            stacklevel=1,
        )

from modular._transformations import (
    identity_matrix,
    translation_matrix,
    euler_matrix,
    euler_from_matrix,
    rotation_matrix,
    concatenate_matrices,
    inverse_matrix,
)

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

ATOL = 1e-10


def assert_matrix(M, expected, atol=ATOL):
    assert np.allclose(M, expected, atol=atol), f"\nGot:\n{M}\nExpected:\n{expected}"


# ──────────────────────────────────────────────────────────────────────────────
# identity_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestIdentityMatrix:
    def test_shape(self):
        assert identity_matrix().shape == (4, 4)

    def test_values(self):
        assert_matrix(identity_matrix(), np.eye(4))

    def test_returns_new_copy(self):
        # Mutating one result must not affect another
        M1 = identity_matrix()
        M2 = identity_matrix()
        M1[0, 0] = 99
        assert M2[0, 0] == 1.0


# ──────────────────────────────────────────────────────────────────────────────
# translation_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestTranslationMatrix:
    def test_shape(self):
        assert translation_matrix([1, 2, 3]).shape == (4, 4)

    def test_translation_stored_in_right_column(self):
        T = translation_matrix([1.0, 2.0, 3.0])
        assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0])

    def test_rotation_part_is_identity(self):
        T = translation_matrix([1.0, 2.0, 3.0])
        assert_matrix(T[:3, :3], np.eye(3))

    def test_homogeneous_row(self):
        T = translation_matrix([1.0, 2.0, 3.0])
        assert_matrix(T[3, :], [0, 0, 0, 1])

    def test_zero_translation(self):
        assert_matrix(translation_matrix([0, 0, 0]), np.eye(4))


# ──────────────────────────────────────────────────────────────────────────────
# euler_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestEulerMatrix:
    def test_shape(self):
        assert euler_matrix(0.1, 0.2, 0.3, 'sxyz').shape == (4, 4)

    def test_zero_angles_is_identity(self):
        assert_matrix(euler_matrix(0, 0, 0, 'sxyz'), np.eye(4))

    def test_rotation_part_is_orthogonal(self):
        R = euler_matrix(0.5, -0.3, 1.1, 'sxyz')[:3, :3]
        assert_matrix(R @ R.T, np.eye(3))

    def test_pure_roll_90deg(self):
        # 90° rotation around X: y->z, z->-y
        M = euler_matrix(math.pi / 2, 0, 0, 'sxyz')
        point = np.array([0, 1, 0, 1])
        result = M @ point
        assert np.allclose(result, [0, 0, 1, 1], atol=1e-10)

    def test_pure_pitch_90deg(self):
        # 90° rotation around Y: z->x, x->-z
        M = euler_matrix(0, math.pi / 2, 0, 'sxyz')
        point = np.array([0, 0, 1, 1])
        result = M @ point
        assert np.allclose(result, [1, 0, 0, 1], atol=1e-10)

    def test_pure_yaw_90deg(self):
        # 90° rotation around Z: x->y, y->-x
        M = euler_matrix(0, 0, math.pi / 2, 'sxyz')
        point = np.array([1, 0, 0, 1])
        result = M @ point
        assert np.allclose(result, [0, 1, 0, 1], atol=1e-10)

    def test_unsupported_convention_raises(self):
        with pytest.raises(NotImplementedError):
            euler_matrix(0.1, 0.2, 0.3, 'rxyz')


# ──────────────────────────────────────────────────────────────────────────────
# euler_from_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestEulerFromMatrix:
    @pytest.mark.parametrize("roll,pitch,yaw", [
        (0.0, 0.0, 0.0),
        (0.1, 0.2, 0.3),
        (-1.0, 0.5, 2.1),
        (math.pi / 4, -math.pi / 6, math.pi / 3),
    ])
    def test_roundtrip(self, roll, pitch, yaw):
        M = euler_matrix(roll, pitch, yaw, 'sxyz')
        r, p, y = euler_from_matrix(M, 'sxyz')
        assert abs(r - roll) < 1e-10
        assert abs(p - pitch) < 1e-10
        assert abs(y - yaw) < 1e-10

    def test_accepts_4x4_matrix(self):
        M = euler_matrix(0.3, -0.2, 0.1, 'sxyz')
        angles = euler_from_matrix(M, 'sxyz')
        assert len(angles) == 3

    def test_accepts_3x3_submatrix(self):
        M = euler_matrix(0.3, -0.2, 0.1, 'sxyz')
        angles_4x4 = euler_from_matrix(M, 'sxyz')
        angles_3x3 = euler_from_matrix(M[:3, :3], 'sxyz')
        assert np.allclose(angles_4x4, angles_3x3, atol=1e-10)

    def test_unsupported_convention_raises(self):
        with pytest.raises(NotImplementedError):
            euler_from_matrix(np.eye(4), 'rxyz')


# ──────────────────────────────────────────────────────────────────────────────
# rotation_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestRotationMatrix:
    def test_shape(self):
        assert rotation_matrix(0.5, [0, 0, 1]).shape == (4, 4)

    def test_zero_angle_is_identity(self):
        assert_matrix(rotation_matrix(0.0, [0, 0, 1]), np.eye(4))

    def test_rotation_part_is_orthogonal(self):
        R = rotation_matrix(1.23, [1, 1, 0])[:3, :3]
        assert_matrix(R @ R.T, np.eye(3))

    def test_180deg_around_z(self):
        M = rotation_matrix(math.pi, [0, 0, 1])
        expected = np.diag([-1.0, -1.0, 1.0, 1.0])
        assert_matrix(M, expected)

    def test_90deg_around_x(self):
        M = rotation_matrix(math.pi / 2, [1, 0, 0])
        point = np.array([0, 1, 0, 1])
        result = M @ point
        assert np.allclose(result, [0, 0, 1, 1], atol=1e-10)

    def test_unnormalized_axis_same_result(self):
        M1 = rotation_matrix(1.0, [0, 0, 1])
        M2 = rotation_matrix(1.0, [0, 0, 5])
        assert_matrix(M1, M2)

    def test_rotation_about_point_leaves_point_fixed(self):
        point = [1.0, 2.0, 0.0]
        M = rotation_matrix(math.pi / 2, [0, 0, 1], point=point)
        result = M @ np.array([1.0, 2.0, 0.0, 1.0])
        assert np.allclose(result[:3], point, atol=1e-10)

    def test_zero_direction_raises(self):
        with pytest.raises(ValueError):
            rotation_matrix(1.0, [0, 0, 0])


# ──────────────────────────────────────────────────────────────────────────────
# concatenate_matrices
# ──────────────────────────────────────────────────────────────────────────────

class TestConcatenateMatrices:
    def test_two_translations_add_up(self):
        T1 = translation_matrix([1, 0, 0])
        T2 = translation_matrix([0, 2, 0])
        result = concatenate_matrices(T1, T2)
        assert np.allclose(result[:3, 3], [1, 2, 0])

    def test_single_matrix_is_itself(self):
        T = translation_matrix([3, 4, 5])
        assert_matrix(concatenate_matrices(T), T)

    def test_three_matrices(self):
        T1 = translation_matrix([1, 0, 0])
        T2 = translation_matrix([0, 1, 0])
        T3 = translation_matrix([0, 0, 1])
        result = concatenate_matrices(T1, T2, T3)
        assert np.allclose(result[:3, 3], [1, 1, 1])

    def test_rotation_then_translation(self):
        # A 90° rotation around Z followed by a translation along X
        # should end up translating along Y in the original frame
        R = rotation_matrix(math.pi / 2, [0, 0, 1])
        T = translation_matrix([1, 0, 0])
        M = concatenate_matrices(R, T)
        point = np.array([0, 0, 0, 1])
        result = M @ point
        assert np.allclose(result[:3], [0, 1, 0], atol=1e-10)

    def test_inverse_cancels(self):
        T = translation_matrix([3, -1, 2])
        R = euler_matrix(0.4, -0.2, 1.1, 'sxyz')
        M = concatenate_matrices(T, R)
        assert_matrix(concatenate_matrices(M, inverse_matrix(M)), np.eye(4))


# ──────────────────────────────────────────────────────────────────────────────
# inverse_matrix
# ──────────────────────────────────────────────────────────────────────────────

class TestInverseMatrix:
    def test_identity_inverse_is_identity(self):
        assert_matrix(inverse_matrix(np.eye(4)), np.eye(4))

    def test_M_times_inv_is_identity(self):
        T = translation_matrix([1, 2, 3])
        R = euler_matrix(0.5, -0.3, 1.1, 'sxyz')
        M = concatenate_matrices(T, R)
        assert_matrix(M @ inverse_matrix(M), np.eye(4))

    def test_inv_times_M_is_identity(self):
        T = translation_matrix([1, 2, 3])
        R = euler_matrix(0.5, -0.3, 1.1, 'sxyz')
        M = concatenate_matrices(T, R)
        assert_matrix(inverse_matrix(M) @ M, np.eye(4))

    def test_pure_translation_inverse(self):
        T = translation_matrix([3, -1, 2])
        T_inv = inverse_matrix(T)
        assert np.allclose(T_inv[:3, 3], [-3, 1, -2])


# ──────────────────────────────────────────────────────────────────────────────
# Cross-validation against the original tf / tf_transformations library
# These tests are skipped if ROS is not installed in the current environment.
# ──────────────────────────────────────────────────────────────────────────────

CROSS_ANGLES = [
    (0.0,  0.0,  0.0),
    (0.1,  0.2,  0.3),
    (-1.0, 0.5,  2.1),
    (math.pi / 4, -math.pi / 6, math.pi / 3),
    (math.pi, 0.0, 0.0),
    (0.0, math.pi, 0.0),
    (0.0, 0.0, math.pi),
]


class TestCrossValidation:

    @_has_ref
    @pytest.mark.parametrize("roll,pitch,yaw", CROSS_ANGLES)
    def test_euler_matrix(self, roll, pitch, yaw):
        ours = euler_matrix(roll, pitch, yaw, 'sxyz')
        ref  = _ref.euler_matrix(roll, pitch, yaw, 'sxyz')
        assert np.allclose(ours, ref, atol=1e-12), (
            f"euler_matrix({roll},{pitch},{yaw}) mismatch:\n ours={ours}\n ref={ref}"
        )

    @_has_ref
    @pytest.mark.parametrize("roll,pitch,yaw", CROSS_ANGLES)
    def test_euler_from_matrix_roundtrip(self, roll, pitch, yaw):
        # Build matrix with reference, extract angles with ours, and vice-versa
        M_ref = _ref.euler_matrix(roll, pitch, yaw, 'sxyz')
        r_ours, p_ours, y_ours = euler_from_matrix(M_ref, 'sxyz')
        r_ref,  p_ref,  y_ref  = _ref.euler_from_matrix(M_ref, 'sxyz')
        assert np.allclose([r_ours, p_ours, y_ours], [r_ref, p_ref, y_ref], atol=1e-12), (
            f"euler_from_matrix mismatch for ({roll},{pitch},{yaw}):\n"
            f" ours=({r_ours},{p_ours},{y_ours})\n ref=({r_ref},{p_ref},{y_ref})"
        )

    @_has_ref
    @pytest.mark.parametrize("xyz", [
        [0, 0, 0], [1, 2, 3], [-0.5, 0.0, 100.0]
    ])
    def test_translation_matrix(self, xyz):
        ours = translation_matrix(xyz)
        ref  = _ref.translation_matrix(xyz)
        assert np.allclose(ours, ref, atol=1e-12)

    @_has_ref
    @pytest.mark.parametrize("angle,axis", [
        (0.0,       [0, 0, 1]),
        (math.pi,   [0, 0, 1]),
        (math.pi/2, [1, 0, 0]),
        (1.23,      [1, 1, 0]),
        (3.14,      [0, 1, 0]),
    ])
    def test_rotation_matrix(self, angle, axis):
        ours = rotation_matrix(angle, axis)
        ref  = _ref.rotation_matrix(angle, axis)
        assert np.allclose(ours, ref, atol=1e-12), (
            f"rotation_matrix({angle},{axis}) mismatch:\n ours={ours}\n ref={ref}"
        )

    @_has_ref
    def test_rotation_matrix_with_point(self):
        ours = rotation_matrix(math.pi / 2, [0, 0, 1], point=[1, 2, 0])
        ref  = _ref.rotation_matrix(math.pi / 2, [0, 0, 1], point=[1, 2, 0])
        assert np.allclose(ours, ref, atol=1e-12)

    @_has_ref
    def test_identity_matrix(self):
        assert np.allclose(identity_matrix(), _ref.identity_matrix(), atol=1e-12)

    @_has_ref
    def test_concatenate_matrices(self):
        T = translation_matrix([1, 2, 3])
        R = euler_matrix(0.4, -0.2, 1.1, 'sxyz')
        ours = concatenate_matrices(T, R)
        ref  = _ref.concatenate_matrices(T, R)
        assert np.allclose(ours, ref, atol=1e-12)

    @_has_ref
    def test_inverse_matrix(self):
        T = translation_matrix([1, 2, 3])
        R = euler_matrix(0.4, -0.2, 1.1, 'sxyz')
        M = concatenate_matrices(T, R)
        ours = inverse_matrix(M)
        ref  = _ref.inverse_matrix(M)
        assert np.allclose(ours, ref, atol=1e-12)
