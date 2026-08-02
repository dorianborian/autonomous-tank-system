<p align="center">
  <img
    src="https://github.com/user-attachments/assets/fe66b835-f37d-494d-a863-cc87c8635cee"
    alt="fridge-tank-github-titlecard"
    width="70%"
  />
</p>
An autonomous robot built around a Frigidaire retro mini fridge on a
tank-tread chassis. It follows or flees from a detected person, avoids
obstacles, and is controlled from a Steam Deck via a browser-based web UI
over a WiFi hotspot the robot itself broadcasts.

## Hardware

| Role | Part |
|---|---|
| Compute | Jetson Orin Nano Super -- JetPack 7.2, L4T R39.2, CUDA 13.2, Ubuntu 24.04, ROS 2 Jazzy |
| Front sensor | ODIN 1 spatial memory module (dToF depth + RGB + IMU + onboard SLAM, USB-C) |
| Rear sensor | USB webcam (RGB only, no depth) |
| Drive | 2x Repeat Ultra Mk2 brushless gearmotors, AM32 dual bidirectional ESC |
| Motor/servo controller | ESP32-S3 (PWM to the ESC, door servo via a PCA9685 I2C board, independent hardware safety watchdog) |
| Chassis | Frigidaire retro mini fridge body on a tank-tread base |
| Operator interface | Steam Deck, browser-based, over the robot's own WiFi hotspot |

## Repository layout

```
ros2_ws/src/        the ROS 2 workspace -- build this with colcon
  motor_controller/     serial_driver: ROS <-> ESP32 bridge
  robot_behavior_msgs/  shared message types (TargetDetection, TargetState, BehaviorState)
  robot_behavior/       perception, state estimation, behavior FSM, drive stack, Nav2 config
  robot_web_ui/         FastAPI/uvicorn web control+telemetry UI (backend + static frontend)
firmware/            ESP32-S3 sketch (source not recovered -- see its README)
vendor/odin_ros_driver/  pin + local patch for the third-party ODIN driver (not vendored in-tree)
legacy/aruco_ws/     archived ArUco marker tracker, superseded by the YOLO trackers
deploy/              systemd units, boot scripts, hotspot toggle, jetson-clocks
docs/                supplementary handoff notes from earlier project phases
```

This is a monorepo over what were previously several independent
`colcon` workspaces (`odin_ws`, `aruco_ws`, `motor_ws`, `behavior_ws`,
plus a nonexistent `web_ws` -- the web UI actually lived inside
`behavior_ws`). Everything that's still active ROS code now lives in one
`ros2_ws/`, since it was always built and run together on the same
Jetson; the ODIN vendor driver stays a separate workspace by necessity
(third-party, its own release cadence -- see `vendor/odin_ros_driver/`),
and the superseded ArUco tracker is archived rather than deleted so the
project's evolution and the reasoning for the YOLO swap stay visible.

## Software architecture

### The mode model

Two independent dimensions, both commanded as `std_msgs/String` and held
by `mode_manager_node` on latched (`transient_local`) QoS topics -- every
subscriber to a mode-state topic must also use that QoS, or a node that
(re)starts after a mode was last set will silently run with its default
mode until the next command:

- **`drive_source`** (`/drive_source_cmd` -> `/drive_source_state`): `manual` | `semi` | `auto`
  - `manual` -- Steam Deck drives directly, no autonomy.
  - `semi` -- autonomous command + joystick bias blended on top, or (with
    no target detected) joystick-authored goals that Nav2 routes to.
  - `auto` -- pure autonomy, controller ignored for driving.
- **`track_mode`** (`/track_mode_cmd` -> `/track_mode_state`): `follow` | `flee`
  - `follow` -- front ODIN camera, chase toward the person.
  - `flee` -- rear webcam, drive away (control law inverted).

Boot default is `manual` / `follow`: nothing drives autonomously until
explicitly commanded. `auto`/`semi` autonomous output additionally
requires a deadman gate to be held (R2 on the controller, or a web UI
toggle) before it reaches the motors -- Nav2 keeps planning and the goal
stays visible either way, it just isn't executed.

### Perception -> estimation -> behavior -> drive pipeline

```
ODIN /odin1/image ----> odin_yolo_tracker_node --\
                                                   \
rear webcam /image_raw -> rear_yolo_tracker_node ---> target_state_estimator_node
                                                   /         |
                          (track_mode selects) ---/          | world-frame
                                                               | alpha-beta filter
                                                               v
                                                    behavior_state_machine_node
                                              (TRACKING/COASTING/SEARCHING/RECOVERING/LOST)
                                                               |
                          intent_to_goal_node <-- target_state (world-frame goal)
                                 |
                          controller_server (DWB) --path--> local costmap (A* via costmap_planner)
                                 |
                          velocity_smoother
                                 |
                          drive_mux_node <-- ps_controller_bridge_node (manual/bias, deadman, e-stop)
                                 |
                          serial_driver (motor_controller) --binary serial--> ESP32-S3 --> ESC/motors
```

Both cameras run YOLO11n via a TensorRT FP16 engine, detecting the
largest person in frame. The front tracker looks up real depth from
ODIN's sparse dToF point cloud; the rear tracker has no depth sensor and
estimates distance from bounding-box height with an assumed average
person height -- materially less accurate, see Known Issues below.
`target_state_estimator_node` fixes a real circling bug from an earlier
version of this pipeline (see its module docstring and the clock-domain
note below), tracking the person in the odom frame with a
constant-velocity filter rather than re-deriving position from the
robot's current (possibly stale) pose every frame.

`behavior_state_machine_node` adds real persistence on top: brief
dropouts don't interrupt driving (COASTING), a longer loss triggers an
active SEARCHING behavior (follow stops and scans; flee keeps moving on
its last heading -- a critical asymmetry), and a stuck-against-something
failure triggers a costmap-checked RECOVERING backup+reorient.

Two older, direct-velocity control paths (`target_pid_node` +
`target_selector_node`, and the ODIN-side ArUco tracker
`odin_target_tracker_node`) are intentionally retired from the active
launch file but kept in the `robot_behavior` package as documented
fallbacks, along with the Phase 6/7.1-era `bringup.launch.py`.

### Clock domain (important, do not "fix")

ODIN's point cloud/image/TF stream is stamped on the sensor's own
hardware clock -- offset from ROS/system time by roughly 56.6 years, not
a small drift. `target_state_estimator_node` therefore uses
`header.stamp` **only** for tf2 time-travel TF lookups (self-consistent
regardless of the clock domain) and `time.monotonic()` for all age/
confidence bookkeeping. This is also why `nav2_collision_monitor` is
absent from `nav2_params.yaml`: it compares message stamps directly
against `now()`, which permanently distrusted live ODIN data and
silently withheld all `/cmd_vel` output. If you ever see this apparent
inconsistency and are tempted to "clean it up," don't -- see the module
docstrings in `target_state_estimator_node.py` and `drive_mux_node.py`
for the full story.

## Setup

### 1. ROS 2 workspace

```bash
mkdir -p ~/autonomous-tank-system
# (clone this repo here)
cd ~/autonomous-tank-system/ros2_ws
colcon build
source install/setup.bash
```

### 2. ODIN vendor driver

Not included in this repo -- see [`vendor/odin_ros_driver/NOTES.md`](vendor/odin_ros_driver/NOTES.md)
for the exact commit to clone, the local patch to apply, and why the fork
isn't vendored in-tree.

### 3. ESP32-S3 firmware

The `dual_esc_door_controller` sketch's source was not recovered when this
repo was packaged -- see [`firmware/esp32_dual_esc_door_controller/README.md`](firmware/esp32_dual_esc_door_controller/README.md)
for the documented wire protocol and the hardware-specific knowledge
(AM32 arming, PCA9685 vs. direct GPIO, ESP32Servo vs. raw LEDC) any
replacement/rewrite must preserve.

### 4. YOLO model + TensorRT engine

Model weights (`yolo11n.pt`) and the exported TensorRT engine
(`yolo11n.engine`) are **not** committed -- the engine is a binary
artifact that is not portable across TensorRT/CUDA versions and silently
fails to load on a different stack (this has already caused a real
regression on this project once, on a JetPack update). Regenerate it on
the target machine:

```bash
pip install ultralytics
yolo export model=yolo11n.pt format=engine half=True imgsz=640 device=0
```

This produces `yolo11n.engine` next to `yolo11n.pt`. Point
`odin_yolo_tracker_node`/`rear_yolo_tracker_node`'s `engine_path`
parameter at it (both default to `~/behavior_ws/models/yolo11n.engine` --
override via a launch parameter or `ros2 param set` to match wherever you
put it, e.g. under this repo).

### 5. WiFi hotspot

`deploy/scripts/network-toggle-hotspot.sh` / `network-toggle-home.sh`
switch the Jetson between its own broadcast hotspot and a normal client
WiFi connection. Both scripts are scrubbed of this robot's actual SSID,
password, and network names -- fill in your own `IFACE`/`HOTSPOT`/
`HOME_CONN` placeholders at the top of each script, and create the
matching NetworkManager profiles (the hotspot script's header comment has
the exact `nmcli` commands).

### 6. Deployment

`deploy/systemd/*.service` are boot-time units for each part of the
stack (`robot-odin`, `robot-serial`, `robot-rear-cam`, `robot-behavior`,
`robot-manual`), plus `jetson-clocks.service` to lock clocks to max.
Replace the `YOUR_USERNAME` placeholders, copy them to
`/etc/systemd/system/`, then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-odin robot-serial robot-rear-cam robot-behavior
```

### 7. Running it

With everything built and the systemd units up, open
`http://<robot-ip>:8088/` from a browser on the Steam Deck (connected to
the robot's hotspot) for the control/telemetry UI. For manual-only
testing without the full autonomy stack, use `robot-manual.service` /
`deploy/scripts/start_manual_mode.sh` instead of `robot-behavior`.

## Known issues / limitations

- **Full-occlusion tracking failure.** If a person leaves frame and stays
  out of both cameras' view for longer than the mode's search timeout
  (SEARCHING), the state machine goes LOST and stops. There is no
  persistent re-identification -- reacquisition is just "a person is
  detected again," which could be a different person.
- **Rear-camera distance is a heuristic, not a measurement.** The rear
  webcam has no depth sensor; its distance estimate comes from bounding-
  box height against an assumed average person height. Flee mode is
  designed around this being a coarse "how close" signal, not a precise
  range.
- **No persistent target re-identification.** Neither tracker distinguishes
  one person from another; "the target" is whoever produces the largest
  detection this frame.
- **Front tracker throughput.** `odin_yolo_tracker_node` is observed
  capped around ~10 Hz against a 30 Hz target rate for the estimator's
  own tick -- the estimator's alpha-beta filter coasts/predicts between
  real detections, but the perception side itself does not yet hit the
  target rate.
- **Outstanding TF timeout.** `target_state_estimator_node`'s capture-time
  TF lookup uses a 150 ms timeout that could, in principle, stall its
  30 Hz tick if TF lookups start taking longer than that. Marked as a
  known issue in that module's docstring rather than silently accepted --
  watch for its fallback-to-latest-transform warning firing often in
  practice.
- **`velocity_smoother` has no "smooth manual too" toggle.** Manual mode
  is intentionally unsmoothed (raw controller feel); a per-mode toggle to
  smooth it too was flagged as a Phase 8 web UI idea and never built (see
  the `TODO` in `nav2_params.yaml`).
- **ESP32-S3 firmware source not recovered** -- see Setup step 3 and
  [`firmware/esp32_dual_esc_door_controller/README.md`](firmware/esp32_dual_esc_door_controller/README.md).

## A note on this repository's own packaging

A real bug was found and fixed while assembling this repo: the vendor
ODIN driver's `script/build_ros2.sh` does `cd $WS_DIR` where `$WS_DIR` is
never defined anywhere in the script, so it silently `cd`s to `$HOME`
before running `rm -rf build install log` -- deleting `~/build`,
`~/install`, `~/log` if they happen to exist, not the intended workspace
directories. See [`vendor/odin_ros_driver/NOTES.md`](vendor/odin_ros_driver/NOTES.md)
for the patch; it has not yet been reported upstream.
