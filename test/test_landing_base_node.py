import cv2
import numpy as np

from tello_driver.landing_base_node import LandingBaseNode


def make_detector():
    detector = LandingBaseNode.__new__(LandingBaseNode)
    detector.min_area_ratio = 0.01
    detector.max_area_ratio = 0.80
    detector.min_yellow_ratio_in_bbox = 0.02
    detector.min_blue_ratio_in_bbox = 0.05
    detector.morph_kernel_size = 5
    detector.yellow_lower = (20, 80, 80)
    detector.yellow_upper = (40, 255, 255)
    detector.blue_lower = (90, 60, 50)
    detector.blue_upper = (135, 255, 255)
    return detector


def make_landing_base_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (120, 180), (520, 430), (255, 0, 0), -1)
    cv2.line(frame, (160, 305), (480, 305), (0, 255, 255), 28)
    cv2.line(frame, (320, 210), (320, 400), (0, 255, 255), 28)
    return frame


def test_detects_synthetic_blue_yellow_landing_base():
    detector = make_detector()
    detections, annotated, debug = detector._detect_and_annotate(
        make_landing_base_frame(),
    )

    assert annotated.shape == (480, 640, 3)
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

    cx, cy = detection['center_px']
    assert 300.0 <= cx <= 340.0
    assert 285.0 <= cy <= 325.0


def test_rejects_candidate_without_yellow_pattern():
    detector = make_detector()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (120, 180), (520, 430), (255, 0, 0), -1)

    detections, _, debug = detector._detect_and_annotate(frame)

    assert detections == []
    assert debug['landing_base_count'] == 0
    assert debug['valid_candidate_count'] == 0


def test_debug_reports_hsv_thresholds():
    detector = make_detector()
    detections, _, debug = detector._detect_and_annotate(
        make_landing_base_frame(),
    )

    assert len(detections) == 1
    assert debug['thresholds']['yellow_lower_hsv'] == [20, 80, 80]
    assert debug['thresholds']['yellow_upper_hsv'] == [40, 255, 255]
    assert debug['thresholds']['blue_lower_hsv'] == [90, 60, 50]
    assert debug['thresholds']['blue_upper_hsv'] == [135, 255, 255]
