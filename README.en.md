# tello_driver

ROS 2 Python package for using a DJI Tello with control, video, telemetry,
computer vision, and a safer bridge between autonomous commands and real Tello
RC commands.

Portuguese documentation: [README.md](README.md)

## Nodes

- `joy_node`: subscribes to `/joy`, applies a deadzone, sends Tello `rc` commands, and maps buttons to `takeoff` and `land`.
- `stream_node`: initializes the Tello SDK/video stream, opens the UDP video feed, and publishes ROS images.
- `telemetry_node`: listens to UDP telemetry on port `8890` and publishes raw and JSON telemetry.
- `vision_node`: subscribes to Tello images, runs YOLO, and publishes annotated images plus JSON detections.
- `qr_node`: detects and decodes QR Codes from `/tello/image_raw`, publishing detections on `/vision/qr_codes`.
- `landing_base_node`: detects blue/yellow bases with OpenCV from `/tello/image_raw`, validating color, shape, and pattern to publish `landing_base` or `takeoff_base` on `/vision/landing_base`.
- `visual_servo_node`: reads visual detections and computes visual-centering commands on `/tello/autonomy/cmd_vel`; it does not send commands directly to the drone.
- `mission_node`: runs a safe mission state machine, consuming telemetry and landing-base detections to publish status, a simple visual map, and either real commands or command previews in `dry_run`.
- `command_mux_node`: subscribes to autonomous commands on `/tello/autonomy/cmd_vel`, converts `Twist` to real Tello `rc`, and executes `takeoff`/`land`/`emergency` through topics with an armed/disarmed mode.

## Topics

| Topic | Type | Direction | Node |
| --- | --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | subscribed | `joy_node` |
| `/tello/image_raw` | `sensor_msgs/Image` | published | `stream_node` |
| `/tello/image_raw` | `sensor_msgs/Image` | subscribed | `vision_node`, `qr_node`, `landing_base_node` |
| `/tello/telemetry/raw` | `std_msgs/String` | published | `telemetry_node` |
| `/tello/telemetry/json` | `std_msgs/String` | published | `telemetry_node` |
| `/tello/telemetry/json` | `std_msgs/String` | subscribed | `mission_node` |
| `/vision/image_annotated` | `sensor_msgs/Image` | published | `vision_node` |
| `/vision/detections` | `std_msgs/String` | published | `vision_node` |
| `/vision/detections` | `std_msgs/String` | subscribed | `visual_servo_node` |
| `/vision/qr_codes` | `std_msgs/String` | published | `qr_node` |
| `/vision/qr_codes` | `std_msgs/String` | optionally subscribed | `visual_servo_node` |
| `/vision/qr_image_annotated` | `sensor_msgs/Image` | published | `qr_node` |
| `/vision/qr_debug` | `std_msgs/String` | published | `qr_node` |
| `/vision/landing_base` | `std_msgs/String` | published | `landing_base_node` |
| `/vision/landing_base` | `std_msgs/String` | subscribed | `mission_node` |
| `/vision/landing_base_image_annotated` | `sensor_msgs/Image` | published | `landing_base_node` |
| `/vision/landing_base_debug` | `std_msgs/String` | published | `landing_base_node` |
| `/vision/landing_base_mask` | `sensor_msgs/Image` | optionally published | `landing_base_node` |
| `/tello/autonomy/cmd_vel` | `geometry_msgs/Twist` | published | `visual_servo_node` |
| `/tello/autonomy/cmd_vel` | `geometry_msgs/Twist` | subscribed | `command_mux_node` |
| `/tello/autonomy/takeoff` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/land` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/enable` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/disable` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/stop` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/emergency` | `std_msgs/Empty` | subscribed | `command_mux_node` |
| `/tello/autonomy/debug` | `std_msgs/String` | published | `visual_servo_node` |
| `/mission/start` | `std_msgs/Bool` | subscribed | `mission_node` |
| `/mission/abort` | `std_msgs/Empty` | subscribed | `mission_node` |
| `/mission/reset` | `std_msgs/Empty` | subscribed | `mission_node` |
| `/mission/status` | `std_msgs/String` | published | `mission_node` |
| `/mission/event` | `std_msgs/String` | published | `mission_node` |
| `/mission/map` | `std_msgs/String` | published | `mission_node` |

## Architecture

The package separates perception, decision, and execution. Vision nodes never
send commands directly to the drone; they publish detections. The
`visual_servo_node` turns a compatible visual detection into `cmd_vel`, and
`command_mux_node` is the only bridge from the autonomous stack to real Tello
commands.

```text
stream_node
  -> /tello/image_raw
     -> vision_node         -> /vision/detections
     -> qr_node             -> /vision/qr_codes
     -> landing_base_node   -> /vision/landing_base

mission_node
  -> /mission/status
  -> /mission/map
  -> /tello/autonomy/enable
  -> /tello/autonomy/takeoff
  -> /tello/autonomy/cmd_vel
  -> /tello/autonomy/land

optional visual_servo_node
  -> /tello/autonomy/cmd_vel

command_mux_node
  -> real rc / takeoff / land / emergency
```

## Build

From the workspace root:

```bash
colcon build --packages-select tello_driver
source install/setup.bash
```

## Tests

The current tests cover shared visual math, a synthetic blue/yellow landing-base
detector case, and safe `mission_node` helpers without requiring a connected
drone:

```bash
colcon test --packages-select tello_driver
colcon test-result --verbose
```

## Basic Usage

```bash
ros2 run tello_driver stream_node
ros2 run tello_driver telemetry_node
ros2 run tello_driver vision_node
ros2 run tello_driver qr_node
ros2 run tello_driver landing_base_node
ros2 run tello_driver mission_node
ros2 run tello_driver visual_servo_node
ros2 run tello_driver command_mux_node
ros2 run tello_driver joy_node
```

The basic launch starts video, telemetry, and YOLO vision without starting
joystick control:

```bash
ros2 launch tello_driver tello_basic.launch.py
```

Disable OpenCV preview windows:

```bash
ros2 launch tello_driver tello_basic.launch.py show_preview:=false
```

Dry run without a connected drone:

```bash
ros2 launch tello_driver tello_basic.launch.py show_preview:=false enable_sdk_init:=false enable_stream_on:=false
```

## Autonomous Launch

The safe autonomous launch starts video, telemetry, YOLO vision, QR detection,
landing-base detection, and `command_mux_node`, but does not start joystick
control. By default, `stream_node` sends `command` and `streamon`, while
`command_mux_node` does not send `command`; it assumes the SDK was already
initialized by `stream_node`.

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false start_armed:=false
```

Dry run without `command` or `streamon`:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false stream_enable_sdk_init:=false enable_stream_on:=false command_mux_enable_sdk_init:=false start_armed:=false
```

Enable the landing-base mask while keeping the command mux disarmed:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false landing_base_publish_mask:=true start_armed:=false
```

`mission_node` is disabled by default in the autonomous launch. Start the
`phase1_demo` mission in safe dry-run mode:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false enable_mission_node:=true mission_dry_run:=true start_armed:=false
```

In `dry_run`, `mission_node` does not publish `cmd_vel`, `takeoff`, `land`,
`enable`, `disable`, or `stop`. It only publishes status, events, a simple map,
and `cmd_preview` showing what it would publish if `dry_run:=false`.

Run only the mission node:

```bash
ros2 run tello_driver mission_node --ros-args -p dry_run:=true -p mission_id:=phase1_demo
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /mission/status
ros2 topic echo /mission/map
ros2 topic echo /mission/event
```

Current `phase1_demo` states:

```text
IDLE -> ARM -> TAKEOFF -> STABILIZE -> SCAN_ARENA -> SELECT_BASE
  -> ALIGN_BASE -> APPROACH_BASE -> LAND -> FINISHED
```

The current map is visual, not metric: it groups seen bases by class and
normalized image position. Do not use it as real drone `x,y` coordinates.

## Landing Base Detection

The `landing_base_node` uses OpenCV instead of YOLO. HSV color segmentation is
only the candidate generator; the final decision also validates shape, internal
pattern, exposure, and temporal stability. The node looks for:

- a square moving base as `landing_base`;
- a rectangular takeoff base as `takeoff_base`, on the same topic.

The expected moving-base pattern is a yellow border, blue field, yellow circle,
and central yellow cross. Detections follow the same JSON list shape used by
`vision_node`:

```json
[
  {
    "class_id": -1,
    "class_name": "landing_base",
    "confidence": 0.82,
    "bbox_xyxy": [120.0, 180.0, 520.0, 430.0],
    "area_ratio": 0.18,
    "aspect_ratio": 1.02,
    "rectangularity": 0.95,
    "center_px": [320.0, 305.0],
    "error_norm": [0.0, 0.22],
    "frame_size": [640, 480],
    "yellow_ratio_in_bbox": 0.12,
    "blue_ratio_in_bbox": 0.55,
    "color_score": 0.92,
    "shape_score": 0.94,
    "pattern_score": 0.88,
    "temporal_hits": 2,
    "temporal_score": 1.0,
    "overexposed_ratio": 0.03
  }
]
```

By default, a candidate must appear in at least 2 recent frames before it is
published. This reduces one-frame false positives caused by reflection,
compression artifacts, or blur.

Run only stream, telemetry, and landing-base detection:

```bash
ros2 launch tello_driver landing_base_test.launch.py show_preview:=false
ros2 topic echo /vision/landing_base
ros2 topic hz /vision/landing_base
```

Enable the combined blue/yellow mask for calibration:

```bash
ros2 launch tello_driver landing_base_test.launch.py show_preview:=false publish_mask:=true
ros2 topic echo /vision/landing_base_debug
```

The main HSV parameters can be changed without editing code:

```bash
ros2 run tello_driver landing_base_node --ros-args \
  -p publish_mask:=true \
  -p yellow_lower_h:=20 -p yellow_upper_h:=40 \
  -p blue_lower_h:=90 -p blue_upper_h:=135
```

If the official landing base is not available, use blue cardboard/fabric and
yellow tape. Try to reproduce the yellow border, blue field, yellow circle, and
central cross; that tests the stronger detector much better than loose blue and
yellow patches.

## QR Detection And Visual Servo

The `qr_node` detects QR Codes from `/tello/image_raw` with OpenCV and publishes
detections on `/vision/qr_codes`. The `visual_servo_node` can consume QR
detections by setting `input_detection_topic:=/vision/qr_codes` and
`target_class_name:=qr_code`.

```bash
ros2 run tello_driver visual_servo_node --ros-args -p input_detection_topic:=/vision/qr_codes -p target_class_name:=qr_code
```

There is also a safe QR + visual-servo launch. By default, it does not start
`joy_node` and does not send `command`/`streamon`:

```bash
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true
```

With a real drone and stream initialization:

```bash
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true enable_sdk_init:=true enable_stream_on:=true
```

## Real Autonomous Command Bridge

`command_mux_node` is the bridge between autonomous decisions and real Tello
commands. It subscribes to `/tello/autonomy/cmd_vel`, converts
`geometry_msgs/Twist` into `send_rc(left_right, forward_back, up_down, yaw)`,
and sends zero RC if it stops receiving messages for longer than
`watchdog_timeout`.

The `Twist` values are not meters per second. They use the Tello RC scale from
`-100` to `100`, with additional parameter limits such as `max_xy_speed`,
`max_z_speed`, and `max_yaw_speed`.

| Twist | Tello RC |
| --- | --- |
| `linear.x` | forward/back |
| `linear.y` | left/right |
| `linear.z` | up/down |
| `angular.z` | yaw |

Do not run `joy_node` and `command_mux_node` as active drone controllers at the
same time.

By default, the mux starts disarmed (`start_armed:=false`). When disarmed, it
sends zero RC and ignores `/tello/autonomy/cmd_vel`. Publishing
`/tello/autonomy/enable` arms autonomy; `/tello/autonomy/disable` disarms it;
`/tello/autonomy/stop` immediately sends zero RC while keeping the current armed
state. `land` works even when disarmed; `takeoff` requires autonomy to be
armed; `emergency` works even when disarmed.

Safety topics:

```bash
ros2 topic pub --once /tello/autonomy/enable std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/disable std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/stop std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/land std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/emergency std_msgs/msg/Empty "{}"
```

## Configuration

Defaults are still declared inside the nodes. The optional
`config/tello_default.yaml` file mirrors those defaults and helps with future
tuning.

For `landing_base_node`, the YAML includes yellow/blue HSV thresholds, area
limits, minimum color/shape/pattern scores, overexposure limits, temporal
parameters, morphology settings, and `publish_mask`. Start by watching
`/vision/landing_base_debug`, then enable `/vision/landing_base_mask` when you
need to tune color thresholds.

For `mission_node`, the YAML includes the `phase1_demo` mission, `dry_run`,
control/status/map topics, and initial scan, alignment, and approach gains. The
defaults remain safe: `dry_run: true` and the autonomous launch keeps
`enable_mission_node:=false`.

## Planned Autonomy Work

Ideas not implemented yet:

- use QR readings inside a mission;
- visit multiple Phase 1 landing bases;
- visually return to `takeoff_base`;
- real automatic landing outside dry-run.
