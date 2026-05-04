#!/usr/bin/env python3


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def compute_bbox_center(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> tuple[float, float]:
    return (float(x1) + float(x2)) / 2.0, (float(y1) + float(y2)) / 2.0


def compute_normalized_error(
    cx: float,
    cy: float,
    frame_w: int,
    frame_h: int,
) -> tuple[float, float]:
    if frame_w <= 0 or frame_h <= 0:
        return 0.0, 0.0

    error_x = (float(cx) - float(frame_w) / 2.0) / (float(frame_w) / 2.0)
    error_y = (float(cy) - float(frame_h) / 2.0) / (float(frame_h) / 2.0)
    return error_x, error_y
