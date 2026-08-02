# ESP32-S3 firmware — `dual_esc_door_controller`

**Source not recovered.** This firmware runs on the robot's ESP32-S3 and was
developed/flashed from a separate machine (Arduino IDE) whose sketch was
never copied onto the Jetson or into any of the ROS workspaces. A filesystem-
wide search of the Jetson at packaging time turned up no `.ino` file, no
Arduino sketchbook directory, and no copy in any workspace. Until the
original sketch (or its host machine) is located, this directory documents
the wire protocol and the hardware-specific knowledge the firmware embodies,
reverse-engineered from the ROS-side driver
([`ros2_ws/src/motor_controller/motor_controller/serial_driver.py`](../../ros2_ws/src/motor_controller/motor_controller/serial_driver.py))
and from prior debugging history. **If you have the sketch, add it here as
`dual_esc_door_controller.ino` and delete this notice.**

## Wire protocol (as implemented by the ROS-side driver)

Fixed-size 4-byte packets over USB-CDC serial at 115200 baud, sent
continuously at 20 Hz regardless of whether a new `/cmd_vel` has arrived:

| Byte | Meaning |
|---|---|
| 0 | `0xFF` sync/header byte |
| 1 | Left motor speed, `[0, 200]`, `100` = neutral (matches AM32 bidirectional ESC convention: below 100 = reverse, above 100 = forward) |
| 2 | Right motor speed, `[0, 200]`, `100` = neutral |
| 3 | Door servo command: `0` / `1` = move (inverted relative to "open"/"close" as a temporary ROS-side bandaid — see the driver's `_door_cmd_cb` comment), `2` = no change since the last packet |

The firmware must independently enforce a hardware watchdog: if no valid
packet arrives within roughly 500ms, it must command the motors to neutral
itself. The 20 Hz send rate on the ROS side exists specifically to stay
comfortably inside that window. **Do not weaken or remove this watchdog** —
it is the last line of defense if the Jetson, ROS graph, or USB link hangs.

## Hardware-specific knowledge the firmware must preserve

These are not implementation details to "clean up" or simplify on a
rewrite — each one was found by hitting a real, confusing failure on the
actual hardware.

1. **AM32 ESC arming sequence.** The AM32 ESCs will not accept throttle
   commands until they have seen a stable neutral (1500us-equivalent, byte
   value `100` in the protocol above) signal held for some duration first.
   Sending a throttle command before that stable-neutral period elapses is
   ignored (or worse, treated as invalid and the ESC stays disarmed). The
   startup sequence must hold neutral before it starts forwarding real
   motor commands.

2. **Do not add `while (!Serial)` before the arming sequence.** This was
   tried and reverted: on the ESP32-S3's native USB-CDC peripheral (as
   opposed to a separate USB-UART bridge chip), `while (!Serial)` races with
   host-side enumeration in a way that isn't guaranteed to resolve cleanly,
   and blocking the arming sequence on it delayed/broke ESC arming on some
   boots. The firmware should begin its arming sequence unconditionally
   rather than waiting on `Serial`.

3. **Door servo is driven via a PCA9685 I2C PWM board, not a direct ESP32
   GPIO pin.** This was a deliberate choice, not a wiring convenience:
   `ESP32Servo`'s `attach()` started returning a negative value (allocation
   failure) once 3 or more servo channels were attached directly to ESP32
   GPIOs simultaneously — the library ran out of internal PWM
   timer/channel slots. Moving the door servo (and any other servo beyond
   the first couple) to the PCA9685 sidesteps that limit entirely, since
   the PCA9685 does its own PWM generation over I2C.

4. **Use the `ESP32Servo` library, not raw `ledcSetup`/`ledcAttach`.** The
   raw LEDC (LED PWM Controller) API changed its function signatures
   between ESP32 Arduino core 2.x and 3.x. `ESP32Servo` wraps that
   difference in a compatibility layer, so sketches written against it
   don't break across a core upgrade. Do not "simplify" servo control down
   to raw `ledc*` calls — that reintroduces the exact portability problem
   the library exists to solve.

## Known bandaid (ROS side, documented in case the firmware is reflashed)

The door servo's physical open/close directions are wired backwards
relative to what the firmware protocol calls `0`/`1`. Rather than reflash,
`serial_driver.py`'s `_door_cmd_cb` swaps the byte values it sends (`open`
-> byte `0`, `close` -> byte `1`, opposite of the "natural" mapping). If the
firmware is ever reflashed with the servo wiring or firmware-side mapping
corrected, revert that swap in `serial_driver.py` at the same time — see the
comment at that call site.
