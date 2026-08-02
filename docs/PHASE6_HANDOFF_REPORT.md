# Phase 6 — Robot Behaviour Layer: Handoff Report

**To:** Head dev agent
**From:** Behaviour-layer build session
**Date:** 2026-07-06
**Workspace:** `~/behavior_ws/src/robot_behavior/` (new; does not modify the other three workspaces except the two noted changes below)

---

## 1. Executive summary

Phase 6 (the behaviour layer: mode switching, PS controller, dual ArUco tracking, and the drive mux) is **built and functionally verified** on real hardware. Manual, full-auto-follow, and full-auto-flee driving all work end-to-end on the physical robot (tested on blocks and driven around the house). Three joystick-dependent test scenarios remain because the controller was not connected at end of session.

**One notable addition the original spec did not anticipate**, now recorded as the Phase 7 direction (Section 8): the operator wants ArUco tracking and the joystick to act as *motivators* (desired direction + speed), with the actual autonomous navigation handled by a SLAM/Nav2 layer over the ODIN data — **not** the current map-blind direct velocity control. Decision reached this session: build a **hybrid that leans goal-based**, where goals are set from operator intent + marker tracking.

---

## 2. Mode model (unchanged from spec — core architecture)

Two independent dimensions, both commanded via `std_msgs/String` topics for now (web UI later):

- **`drive_source`** (`/drive_source_cmd`): `manual` | `semi` | `auto`
  - manual = PS controller drives directly, no autonomy
  - semi = autonomous base cmd + joystick bias added on top
  - auto = pure autonomous, controller ignored for driving
- **`track_mode`** (`/track_mode_cmd`): `follow` | `flee`
  - follow = ODIN **front** RGB camera (chase toward)
  - flee = **rear** webcam (drive away, PID inverted)

Defaults on boot: `manual` / `follow` (safe — nothing drives autonomously until commanded).

---

## 3. Build status (spec steps 1–8: all complete)

| Step | Node (pkg `robot_behavior` unless noted) | Status |
|---|---|---|
| 1 | Confirm sensor topics | ✅ ODIN RGB = `/odin1/image` (`sensor_msgs/Image`, ~7 Hz). Rear webcam = `/image_raw` (~30 Hz, via `usb_cam`). |
| 2 | `odin_target_tracker_node` | ✅ ArUco on ODIN RGB; downsampled 0.5× for CPU (see §5). |
| 3 | `target_selector_node` | ✅ Routes ODIN (follow) or rear (flee) to unified `/target_*`. |
| 4 | `target_pid_node` | ✅ Bearing PID→angular, 3-zone distance→linear, flee inversion. Heavily tuned (§5). |
| 5 | `ps_controller_bridge_node` | ✅ `/joy`→`/manual_cmd_vel` + `/bias_cmd_vel`; e-stop button. |
| 6 | `mode_manager_node` | ✅ Holds/validates state; latched publish. QoS bug fixed (§6). |
| 7 | `drive_mux_node` | ✅ manual / semi(blend+clamp) / auto routing → `/cmd_vel`. |
| 8 | `safety_watchdog_node` | ✅ Joy-stale fallback + target-loss logging. |

### Test scenarios (step 9)

| # | Scenario | Status |
|---|---|---|
| 9a | Manual drive matches stick | ✅ Confirmed — driven around the house |
| 9b | Auto-follow via ODIN only | ✅ Confirmed + tuned live |
| 9c | Auto-flee via rear only | ✅ Verified (shares the entire follow path; flee control law verified end-to-end via synthetic-target injection reading `/cmd_vel_pid`) |
| 9d | Semi-auto (auto + stick bias) | ⬜ **Pending controller reconnect** |
| 9e | Controller-disconnect safety fallback | ⬜ **Pending controller reconnect** |
| 9f | E-stop button override | ⬜ **Pending controller reconnect** (also needs button index confirmed on hardware — see §7) |

---

## 4. System architecture (nodes + topics)

External (other workspaces, unmodified except serial_driver — see §5):

- `odin_ros_driver host_sdk_sample` → `/odin1/image`, `/odin1/odometry`, `/odin1/cloud_raw`, `/odin1/cloud_slam`, `odom→base_link` TF
- `usb_cam` → `/image_raw` (rear webcam, remapped)
- `motor_controller serial_driver` ← `/cmd_vel`, `/door_cmd` → ESP32-S3 (binary packets; ESP32 keeps its own independent 500 ms HW watchdog)

Behaviour layer data flow:

```
  ODIN /odin1/image ──▶ odin_target_tracker ──▶ /odin_target_{detected,bearing,distance}
                                                          │
  rear /image_raw ──▶ aruco_tracker (remapped) ──▶ /rear_target_{detected,bearing,distance}
                                                          │
                    /track_mode_state (latched) ──▶ target_selector
                                                          │
                                          /target_{detected,bearing,distance}  (unified, mode-agnostic)
                                                          │
                    /track_mode_state (latched) ──▶ target_pid ──▶ /cmd_vel_pid
                                                                          │
  /joy ──▶ ps_controller_bridge ──▶ /manual_cmd_vel, /bias_cmd_vel        │
                                          │                               │
                    /drive_source_state (latched) ──▶ drive_mux ◀─────────┘
                                                          │
                                                      /cmd_vel ──▶ serial_driver ──▶ ESP32 ──▶ motors

  mode_manager: /drive_source_cmd, /track_mode_cmd ──▶ (latched) /drive_source_state, /track_mode_state, /current_mode
  safety_watchdog: /joy, /drive_source_state, /target_detected ──▶ /drive_source_cmd (fallback commands)
  ps_controller_bridge e-stop ──▶ /drive_source_cmd = "manual" + zero /manual_cmd_vel
```

Support: `joy_node` → `/joy`; `web_video_server` on `:8080` (added this session for headless debug — streams `/odin_aruco_debug_image` and `/aruco_debug_image` with marker/bearing/distance overlays).

---

## 5. Design decisions & changes made this session

**Flagged design choices (operator-approved where noted):**

1. **3-zone distance control** (replaced the spec's single P-on-distance). Reverse < 0.4 m, dead-zone 0.4–0.6 m (hold), chase 0.6 m+ (full speed by 1.0 m). Tuned live with the operator.
2. **Flee = "stop when far"** (operator decision, deviates from literal "invert PID"). Close rear threat → drive forward away; far threat → **stop**, never re-approach. Implemented as: invert follow output, then clamp the resulting `linear.x` to ≥ 0.
3. **Bearing dead-band (0.08)** — root cause of near-target wiggle: differential turning isn't a perfect in-place pivot, so micro-centering corrections were nudging the distance reading and re-triggering motion. Ignoring tiny bearing error fixed it.
4. **Distance hysteresis (0.08 m)** on the zone boundaries — Schmitt-trigger to stop zone flip-flop chatter from detection noise + the min-speed floor.
5. **`min_effective_speed = 10`** (added to `serial_driver`, motor_ws) — motors don't turn below ~10 on the [-100,100] scale, so any nonzero-but-smaller command is boosted to the floor; exact 0 stays 0.
6. **ODIN frame downsample 0.5×** in `odin_target_tracker` — tegrastats showed all 6 CPU cores at 65–75 % and **GPU idle at 0 %**; OpenCV here has no CUDA build and ArUco has no GPU path regardless, so downsampling the 1600×1296 ODIN frame is the real lever. Dropped the tracker from ~146 % CPU to ~60 %, cores to ~45–60 %.
7. **Manual angular scale raised to 4.0 rad/s** — at 2.0 the turn differential was too subtle to feel at 25 % power.

**Two edits outside `behavior_ws` (noted per spec):**

- `motor_ws/.../serial_driver.py`: corrected `TRACK_WIDTH` 0.25 → **0.2286 m**; added `min_effective_speed`. (No logic change to packet protocol or the ESP32 watchdog.)
- Nothing else in the other three workspaces was modified.

**Rejected / reverted:** attempted ODIN `dtof_fps=290` to raise RGB rate — the comment is misleading; it sets the depth sensor to 29 fps and **overloaded** the pipeline (RGB dropped to 2–4 Hz with queue overflows). Reverted to `100`. No exposed knob raises the RGB rate as configured; the driver's effective RGB ceiling is ~7 Hz.

---

## 6. Bug found & fixed (correctness, safety-relevant)

**Latched-QoS mismatch.** `mode_manager` publishes the mode-state topics (`/drive_source_state`, `/track_mode_state`, `/current_mode`) with **transient_local** (latched) QoS so late-joining nodes inherit the current mode on connect. But the four consumer nodes subscribed with **volatile** QoS, which does **not** receive the latched historical sample. Effect: any node that restarts or launches after a mode is set silently runs with its **default** mode until the next command. Caught in the act — a restarted `target_pid` stayed in `follow` while the system was in `flee`.

**Fix:** added a shared `latched_qos()` factory in `mode_constants.py`; all four consumers (`target_pid`, `target_selector`, `drive_mux`, `safety_watchdog`) and the publisher now use matching transient_local QoS. Verified: a node launched *after* the mode is set now inherits it on join (observed `follow -> flee` immediately on `target_selector` startup, and the flee control law then producing correct output with no live re-publish).

---

## 7. Tunable constants (all ROS 2 params; live-override with `-p name:=value`)

**`target_pid_node`**
| Param | Value | Meaning |
|---|---|---|
| `kp_bearing` / `ki_bearing` / `kd_bearing` | 1.5 / 0.0 / 0.3 | Bearing PID → angular.z |
| `max_angular_z` | 1.0 rad/s | Clamp on PID angular output |
| `bearing_deadband` | 0.08 | Ignore |bearing| below this (anti-wiggle) |
| `reverse_threshold_m` | 0.4 | Below → back away |
| `chase_min_m` | 0.6 | Above → chase (0.4–0.6 = dead zone) |
| `chase_max_m` | 1.0 | Distance at which chase hits full speed |
| `max_linear_x` | 0.4 m/s | Clamp on PID linear output |
| `distance_hysteresis_m` | 0.08 | Zone-boundary Schmitt-trigger margin |
| (control rate) | 20 Hz | Loop rate |

**`ps_controller_bridge_node`**
| Param | Value | Meaning |
|---|---|---|
| `axis_linear` / `axis_angular` | 1 / 0 | Left-stick Y / X |
| `axis_linear_sign` / `axis_angular_sign` | 1.0 / 1.0 | Flip if inverted on hardware |
| `max_linear_x` | 1.0 m/s | Full-scale manual linear |
| `max_angular_z` | 4.0 rad/s | Full-scale manual angular |
| `bias_scale` | 0.25 | Semi-mode nudge = 25 % of full scale |
| `stick_deadzone` | 0.05 | Ignore small stick noise |
| `estop_button_index` | 10 | **Assumed PS button — NOT hardware-confirmed** |

**`drive_mux_node`**
| Param | Value | Meaning |
|---|---|---|
| `input_stale_timeout_sec` | 0.3 | Treat inputs older than this as zero |
| `semi_max_linear_x` | 0.6 m/s | Clamp on semi-blend linear |
| `semi_max_angular_z` | 1.3 rad/s | Clamp on semi-blend angular |
| (publish rate) | 20 Hz | Matches serial_driver |

**`safety_watchdog_node`**
| Param | Value | Meaning |
|---|---|---|
| `joy_stale_timeout_sec` | 0.5 | Controller-lost threshold |
| `target_loss_log_timeout_sec` | 3.0 | Target-lost log threshold (informational) |
| (rate) | 10 Hz | |

**`odin_target_tracker_node`**
| Param | Value | Meaning |
|---|---|---|
| `marker_size_m` | 0.10 | Physical marker edge |
| `aruco_dictionary` | DICT_4X4_50 | |
| `calibrated_focal_px` | 734.92 | From ODIN calib.yaml @1600 px |
| `calibrated_image_width_px` | 1600 | Reference width for focal scaling |
| `downsample_factor` | 0.5 | Pre-detection resize (CPU) |

**`serial_driver` (motor_ws)** — launched this session with `-p speed_scale:=0.30`
| Param | Value | Meaning |
|---|---|---|
| `speed_scale` | **0.30** (code default 0.15) | Global power cap; 30 % on-blocks-safe |
| `min_effective_speed` | 10.0 | Motor stiction floor (nonzero boost) |
| `max_linear_speed` | 1.0 m/s | Normalisation reference |
| `max_angular_speed` | 2.0 rad/s | **Dead code** — mixing uses only TRACK_WIDTH |
| `TRACK_WIDTH` (const) | 0.2286 m | Corrected from measured chassis |

---

## 8. Phase 7 direction — SLAM-aware navigation layer (DECIDED: hybrid, lean goal-based)

**Operator intent:** the ArUco marker tracking and the joystick should be **motivators** — expressing desired *direction and speed / where to go* — while the actual autonomous navigation (steering, obstacle avoidance, path adjustment) is handled by **Nav2 (or equivalent) driven by the ODIN 1 SLAM data**. The current `target_pid`→`cmd_vel`→motors path is **map-blind reactive control** and would be superseded by this layer.

**Decision this session:** build a **hybrid that leans goal-based**, with goals set from intent + marker tracking:

- **Auto (follow/flee):** marker pose (bearing + distance, lifted into the map/odom frame via camera extrinsics + TF) generates a **goal pose** — for follow, a standoff point ~1 m short of the marker; for flee, a point directed away from the threat. Nav2 plans + obstacle-avoids to it. This is the primary, goal-based path.
- **Semi (the hybrid part):** joystick acts as the motivator that **shapes/overrides the goal** (nudges goal direction & desired speed) while Nav2 still executes safely with obstacle avoidance. Exact blend to be designed (candidate: project the stick vector forward into a moving goal / bias the marker-derived goal). Assisted-teleop feel, goal-based mechanism.
- **Manual:** unchanged — raw joystick straight to motors, no autonomy.

**What already exists to build on (ODIN provides most of the Nav2 sensor foundation):**
- `/odin1/odometry` (`nav_msgs/Odometry`) ✅
- `/odin1/cloud_raw` + `/odin1/cloud_slam` (`PointCloud2`) ✅ → costmap obstacle/voxel source
- `odom→base_link` TF published ✅
- ODIN has a SLAM mode (`custom_map_mode`: 0=odom [current], 1=SLAM, 2=reloc) → a `map` frame is available if enabled
- Output interface is already compatible: Nav2 emits Twist `/cmd_vel`, which `serial_driver` already consumes unchanged.

**What's missing / to scope:**
- Nav2 bringup + config (planner, controller, behaviour tree — or a trimmed controller/costmap-only setup)
- Pointcloud → costmap layer configuration
- TF completion: `base_link` → camera frames (ODIN calib.yaml has `Tcl`, the camera↔lidar transform), and standing up the `map` frame (enable ODIN SLAM mode)
- The **intent→goal bridge**: marker (bearing/distance) → goal pose; and the semi-mode joystick→goal-shaping logic
- Continuous-goal handling for a moving target (Nav2 is goal-oriented; following a moving marker means streaming goal updates — via a lightweight goal-republisher or the controller/costmap directly rather than the full planning BT each tick)

**Phase 6 reuse under the new architecture:**
- `target_pid` → repurposed/renamed as an **intent generator** (produces goal/velocity intent instead of final cmd_vel), or partially replaced by the goal bridge
- `ps_controller_bridge` → its Twist outputs become the semi-mode motivator/goal-shaping input
- `drive_mux` → role shifts from "blend to motors" to "route intent into the nav layer / arbitrate manual-vs-nav output"
- `mode_manager`, `target_selector`, `safety_watchdog` → largely intact

**Open questions for the head dev:**
1. Full Nav2 stack, or a trimmed costmap+controller (Collision-Monitor-style) for the semi/shared-control case? (Goal-based auto wants the planner; semi wants fast local safety.)
2. Enable ODIN SLAM mode (persistent `map`) now, or run local-only on `odom` first?
3. Semi-mode blend semantics: stick shapes the goal, or stick velocity layered over Nav2 output? (Report assumes goal-shaping per "lean goal-based.")

---

## 9. Operational notes (bring-up)

- No one-shot launch file yet; nodes are started individually (a `bringup.launch.py` exists in the pkg for the behaviour nodes but the full system — ODIN, rear cam, serial, joy, web video — is currently brought up by hand). **Recommend authoring a single top-level launch as an early Phase 7 task.**
- **Joystick permission:** user was added to the `input` group (persists across reboots) so `joy_node` can open `/dev/input/js0` regardless of the GUI lock screen / seat owner. DualSense pairs over Bluetooth; press PS button to wake before connecting.
- **ESC arming:** motors need a steady neutral `/cmd_vel` for a moment after battery connect before they accept throttle (no confirmation tone until armed). The idle zero-Twist stream handles this; if unresponsive, hold neutral in manual briefly.
- **Serial port** enumerates as `/dev/ttyACM0` (ESP32-S3, `303a:1001`).
- **Safety:** ESP32's independent 500 ms hardware watchdog remains the last line of defense and was not touched.

---

## 10. Recommended next steps

1. Reconnect controller → close out **9d / 9e / 9f** (≈30 min) for a clean, fully-verified Phase 6 baseline.
2. Confirm the **e-stop button index** live (currently an unverified assumption of index 10).
3. Author a **single top-level launch file** for the whole stack.
4. Begin **Phase 7 scoping** per Section 8 (answer the three open questions first).
