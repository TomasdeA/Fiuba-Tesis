from types import SimpleNamespace

from output_viewer.odometry_conventions import (
    rtabmap_position,
    rtabmap_yaw_quaternion,
)
import pytest


def _orientation(w, x, y, z):
    return SimpleNamespace(w=w, x=x, y=y, z=z)


def test_rtabmap_position_matches_mapper_xz_convention():
    assert rtabmap_position(3.0, 2.0) == (-2.0, 3.0)


def test_degenerate_forward_projection_falls_back_to_identity():
    quaternion = rtabmap_yaw_quaternion(_orientation(1.0, 0.0, 0.0, 0.0))

    assert quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_invalid_rtabmap_quaternion_is_handled_as_identity():
    quaternion = rtabmap_yaw_quaternion(_orientation(0.0, 0.0, 0.0, 0.0))

    assert quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_rtabmap_optical_basis_becomes_internal_identity():
    half = 0.5
    quaternion = rtabmap_yaw_quaternion(
        _orientation(half, -half, half, -half)
    )

    assert quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
