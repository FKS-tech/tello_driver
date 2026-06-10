import pytest

from tello_driver.visual_math import (
    clamp,
    compute_bbox_center,
    compute_normalized_error,
)


def test_clamp_limits_values():
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5
    assert clamp(2.0, -1.0, 1.0) == 1.0


def test_compute_bbox_center_returns_float_center():
    assert compute_bbox_center(10, 20, 30, 60) == (20.0, 40.0)


def test_compute_normalized_error_at_image_center_is_zero():
    assert compute_normalized_error(320, 240, 640, 480) == (0.0, 0.0)


def test_compute_normalized_error_uses_half_frame_scale():
    error_x, error_y = compute_normalized_error(640, 480, 640, 480)

    assert error_x == pytest.approx(1.0)
    assert error_y == pytest.approx(1.0)


def test_compute_normalized_error_handles_invalid_frame_size():
    assert compute_normalized_error(10, 10, 0, 480) == (0.0, 0.0)
    assert compute_normalized_error(10, 10, 640, 0) == (0.0, 0.0)
