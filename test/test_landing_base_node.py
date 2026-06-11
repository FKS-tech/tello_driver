import cv2
import numpy as np

from tello_driver.landing_base_node import LandingBaseNode


def make_detector():
    detector = LandingBaseNode.__new__(LandingBaseNode)
    detector.min_area_ratio = 0.01
    detector.max_area_ratio = 0.80
    detector.min_yellow_ratio_in_bbox = 0.02
    detector.min_blue_ratio_in_bbox = 0.05
    detector.min_color_score = 0.45
    detector.min_shape_score = 0.35
    detector.min_pattern_score = 0.35
    detector.max_color_only_confidence = 0.60
    detector.max_overexposed_ratio = 0.35
    detector.landing_square_max_aspect = 1.60
    detector.enable_takeoff_base_detection = True
    detector.takeoff_min_aspect = 1.60
    detector.takeoff_max_aspect = 4.00
    detector.pattern_roi_size = 240
    detector.border_band_fraction = 0.08
    detector.circle_radius_min_fraction = 0.21
    detector.circle_radius_max_fraction = 0.34
    detector.cross_band_fraction = 0.06
    detector.temporal_window_size = 5
    detector.min_temporal_hits = 2
    detector.temporal_max_center_distance_norm = 0.18
    detector.temporal_max_area_ratio_delta = 0.12
    detector.morph_kernel_size = 5
    detector.yellow_lower = (20, 80, 80)
    detector.yellow_upper = (40, 255, 255)
    detector.blue_lower = (90, 60, 50)
    detector.blue_upper = (135, 255, 255)
    detector.recent_candidates = []
    return detector


def draw_base_pattern(frame, outer_p1, outer_p2, border_px):
    x1, y1 = outer_p1
    x2, y2 = outer_p2
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    width = x2 - x1
    height = y2 - y1
    min_size = min(width, height)

    cv2.rectangle(frame, outer_p1, outer_p2, (0, 255, 255), -1)
    cv2.rectangle(
        frame,
        (x1 + border_px, y1 + border_px),
        (x2 - border_px, y2 - border_px),
        (255, 0, 0),
        -1,
    )
    cv2.circle(
        frame,
        (cx, cy),
        int(min_size * 0.30),
        (0, 255, 255),
        max(8, int(min_size * 0.035)),
    )
    cv2.line(
        frame,
        (int(cx - min_size * 0.22), cy),
        (int(cx + min_size * 0.22), cy),
        (0, 255, 255),
        max(10, int(min_size * 0.045)),
    )
    cv2.line(
        frame,
        (cx, int(cy - min_size * 0.22)),
        (cx, int(cy + min_size * 0.22)),
        (0, 255, 255),
        max(10, int(min_size * 0.045)),
    )


def make_landing_base_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    draw_base_pattern(frame, (170, 90), (470, 390), 18)
    return frame


def make_takeoff_base_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    draw_base_pattern(frame, (80, 140), (560, 380), 18)
    return frame


def test_detects_synthetic_blue_yellow_landing_base():
    detector = make_detector()
    frame = make_landing_base_frame()
    first_detections, _, first_debug = detector._detect_and_annotate(frame)
    detections, annotated, debug = detector._detect_and_annotate(frame)

    assert annotated.shape == (480, 640, 3)
    assert first_detections == []
    assert first_debug['selected_candidate']['temporal_hits'] == 1
    assert len(detections) == 1
    assert debug['landing_base_count'] == 1
    assert debug['valid_candidate_count'] >= 1
    assert debug['combined_mask'].shape == (480, 640)

    detection = detections[0]
    assert detection['class_id'] == -1
    assert detection['class_name'] == 'landing_base'
    assert detection['confidence'] > 0.8
    assert detection['frame_size'] == [640, 480]
    assert detection['yellow_ratio_in_bbox'] >= 0.02
    assert detection['blue_ratio_in_bbox'] >= 0.05
    assert detection['shape_score'] >= 0.35
    assert detection['pattern_score'] >= 0.35
    assert detection['temporal_hits'] >= 2

    cx, cy = detection['center_px']
    assert 300.0 <= cx <= 340.0
    assert 220.0 <= cy <= 260.0


def test_classifies_rectangular_base_as_takeoff_base():
    detector = make_detector()
    frame = make_takeoff_base_frame()
    detector._detect_and_annotate(frame)
    detections, _, debug = detector._detect_and_annotate(frame)

    assert len(detections) == 1
    assert detections[0]['class_name'] == 'takeoff_base'
    assert detections[0]['aspect_ratio'] >= detector.takeoff_min_aspect
    assert debug['detection_class_counts'] == {'takeoff_base': 1}


def test_rejects_candidate_without_yellow_pattern():
    detector = make_detector()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (120, 180), (520, 430), (255, 0, 0), -1)

    detections, _, debug = detector._detect_and_annotate(frame)

    assert detections == []
    assert debug['landing_base_count'] == 0
    assert debug['valid_candidate_count'] == 0


def test_rejects_blue_yellow_patch_without_base_pattern():
    detector = make_detector()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (120, 180), (520, 430), (255, 0, 0), -1)
    cv2.rectangle(frame, (130, 190), (240, 250), (0, 255, 255), -1)

    detections, _, debug = detector._detect_and_annotate(frame)

    assert detections == []
    assert debug['landing_base_count'] == 0
    assert debug['selected_candidate'] is None


def test_debug_reports_hsv_thresholds():
    detector = make_detector()
    frame = make_landing_base_frame()
    detector._detect_and_annotate(frame)
    detections, _, debug = detector._detect_and_annotate(frame)

    assert len(detections) == 1
    assert debug['thresholds']['yellow_lower_hsv'] == [20, 80, 80]
    assert debug['thresholds']['yellow_upper_hsv'] == [40, 255, 255]
    assert debug['thresholds']['blue_lower_hsv'] == [90, 60, 50]
    assert debug['thresholds']['blue_upper_hsv'] == [135, 255, 255]
    assert debug['thresholds']['min_temporal_hits'] == 2
