"""Phase 8 web control/telemetry UI backend.

FastAPI + WebSocket chosen over Flask: this app's actual job is holding many
concurrent long-lived connections at once (two MJPEG multipart streams plus a
telemetry WebSocket, all pushed continuously rather than request/response) —
that is exactly what an ASGI app on an asyncio event loop (uvicorn) is built
for. Flask's default WSGI model serves one blocking request per worker
thread, which works but fights this workload (a stuck/slow client on one
stream can tie up a whole worker); FastAPI/uvicorn multiplexes all of it on
one event loop for free, and gets a typed WebSocket API without extra
libraries. rclpy runs its own executor in a background thread (see main())
so neither system blocks the other; ros_bridge.SharedState is the single
thread-safe handoff point between them.

Frontend note: served as one static index.html/app.js — no Node/npm toolchain
is installed on this Jetson (checked before choosing this route), and none is
needed for a single-page vanilla-JS app, so there's no build step to keep in
sync with the backend.
"""
import asyncio
import json
import os
import threading
import time

import rclpy
import uvicorn
from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from rclpy.executors import SingleThreadedExecutor

from robot_web_ui.ros_bridge import SharedState, WebUiBridgeNode

STATIC_DIR = __file__.rsplit('/', 1)[0] + '/static'

FAST_HZ = 10.0   # target bbox/bearing/distance — needs to feel live as a person moves
SLOW_HZ = 2.0    # node health / mode / door / cpu-gpu / event log
MJPEG_HZ = 15.0  # camera stream frame rate cap

# --- Steam Deck gamepad -> ps_controller_bridge translation ---------------
# ps_controller_bridge_node.py's button-index parameters (estop_button_index
# =10, deadman_button_index=7, lock=1, delete=2, door=1) were hardware-tuned
# against joy_node's LINUX JOYSTICK DRIVER mapping of a real PS5 pad — NOT
# the W3C "standard gamepad" mapping browsers expose via the Gamepad API,
# which uses different index assignments for the same physical buttons
# (e.g. standard-mapping index 1/2 are Circle/Square, not the X/Square pair
# joy_node reported at those indices). Forwarding the browser's raw button
# array as-is into /joy would silently scramble e-stop/lock/door/deadman.
#
# So this table translates FROM semantic browser Gamepad API slots (assuming
# the Deck reports the W3C standard mapping — TO BE CONFIRMED live, see test
# step 1) TO the exact array positions ps_controller_bridge already expects,
# so that node needs zero changes. Only this table should need editing if
# live testing shows a different mapping.
WEB_AXIS_LINEAR = 1   # standard mapping: left stick Y
WEB_AXIS_ANGULAR = 0  # standard mapping: left stick X
# Browser convention: pushing the stick UP reports -1 on the Y axis.
# ps_controller_bridge/teleop_twist_joy convention (see AXIS_LINEAR_SIGN
# comment in that file) expects forward/up = +1. Inverted here so "forward"
# means the same thing on both sides of the translation — VERIFY live
# (test step 3): if the robot drives backward when the stick is pushed
# forward, this sign (not ps_controller_bridge's own axis_linear_sign) is
# the first thing to flip.
WEB_AXIS_LINEAR_SIGN = -1.0
WEB_AXIS_ANGULAR_SIGN = 1.0

# Output array slot ps_controller_bridge reads (see AXIS_LINEAR/AXIS_ANGULAR
# in that file) — do not change unless that node's params change too.
OUT_AXIS_LINEAR = 1
OUT_AXIS_ANGULAR = 0

# display index -> (output button slot, ps_controller_bridge role)
# RT (standard index 7) is the one mapping likely to survive unchanged: R2
# read as a clean digital button on the real pad too (see DEADMAN_BUTTON_INDEX
# comment). The Guide/PS/Steam button (role: e-stop) is commonly NOT exposed
# to web pages at all by design (reserved for OS/Steam UI) — if step 1's
# live log confirms that, e-stop MUST be triggered from the on-screen button
# and/or a substitute like Select+Start, not a single browser button index.
# Placeholder below assumes standard index 16 (Home) reports through; flag
# immediately if it does not, per the task's explicit instruction.
WEB_BUTTON_DEADMAN = 7    # RT
WEB_BUTTON_LOCK = 2       # standard mapping: left face button (X-equivalent)
WEB_BUTTON_DELETE = 1     # standard mapping: right face button (Circle-equivalent)
WEB_BUTTON_DOOR = WEB_BUTTON_LOCK       # same physical button as lock, see ps_controller_bridge door comment
WEB_BUTTON_ESTOP = 16     # Guide/Home — UNVERIFIED, see comment above

OUT_BUTTON_DEADMAN = 7
OUT_BUTTON_LOCK = 1
OUT_BUTTON_DELETE = 2
OUT_BUTTON_DOOR = 1
OUT_BUTTON_ESTOP = 10

OUT_BUTTONS_LEN = 11  # highest OUT_BUTTON_* index + 1


def translate_gamepad_to_joy_arrays(axes, buttons):
    def axis(i):
        return float(axes[i]) if i < len(axes) else 0.0

    def button(i):
        return bool(buttons[i]) if i < len(buttons) else False

    out_axes = [0.0, 0.0]
    out_axes[OUT_AXIS_LINEAR] = axis(WEB_AXIS_LINEAR) * WEB_AXIS_LINEAR_SIGN
    out_axes[OUT_AXIS_ANGULAR] = axis(WEB_AXIS_ANGULAR) * WEB_AXIS_ANGULAR_SIGN

    out_buttons = [0] * OUT_BUTTONS_LEN
    out_buttons[OUT_BUTTON_DEADMAN] = int(button(WEB_BUTTON_DEADMAN))
    out_buttons[OUT_BUTTON_LOCK] = int(button(WEB_BUTTON_LOCK))
    out_buttons[OUT_BUTTON_DELETE] = int(button(WEB_BUTTON_DELETE))
    out_buttons[OUT_BUTTON_DOOR] = int(button(WEB_BUTTON_LOCK))  # same source button as lock
    out_buttons[OUT_BUTTON_ESTOP] = int(button(WEB_BUTTON_ESTOP))
    return out_axes, out_buttons


def create_app(state: SharedState, ros_node: WebUiBridgeNode):
    app = FastAPI()

    # This UI is actively iterated on with the robot running live -- no
    # Cache-Control header (Starlette's default) lets browsers heuristically
    # cache index.html/app.js/etc, which caused a real live symptom (a
    # frontend fix silently not taking effect on a page reload).
    #
    # NOTE: a global @app.middleware('http') was tried here first and
    # caused a real outage (2026-07-25) -- Starlette's http middleware
    # decorator wraps EVERY route through BaseHTTPMiddleware, including the
    # /video/* MJPEG StreamingResponses, which never terminate. That
    # combination is a known Starlette footgun (BaseHTTPMiddleware buffers/
    # awaits response completion in a way that doesn't play well with
    # infinite streams) and hung the whole server. Fixed by setting the
    # header explicitly per-route instead of via middleware, so the MJPEG
    # routes are never touched by this at all.
    no_cache = {'Cache-Control': 'no-cache'}

    @app.get('/')
    async def index():
        return FileResponse(STATIC_DIR + '/index.html', headers=no_cache)

    @app.get('/static/{filename}')
    async def static_file(filename: str):
        # Small, fixed set of known files (index.html/app.js/
        # deck_schematic.js) -- explicit routes instead of StaticFiles so
        # each response gets the no-cache header without a middleware.
        safe_name = os.path.basename(filename)  # no path traversal
        path = os.path.join(STATIC_DIR, safe_name)
        if not os.path.isfile(path):
            return Response(status_code=404)
        media_type = 'text/javascript' if safe_name.endswith('.js') else None
        return FileResponse(path, headers=no_cache, media_type=media_type)

    async def _mjpeg_generator(get_jpeg):
        boundary = b'--frame\r\n'
        period = 1.0 / MJPEG_HZ
        while True:
            jpeg = get_jpeg()
            if jpeg is not None:
                yield (boundary + b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
            await asyncio.sleep(period)

    @app.get('/video/front')
    async def video_front():
        return StreamingResponse(
            _mjpeg_generator(lambda: state.latest_front_jpeg),
            media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/video/rear')
    async def video_rear():
        return StreamingResponse(
            _mjpeg_generator(lambda: state.latest_rear_jpeg),
            media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/video/costmap')
    async def video_costmap():
        return StreamingResponse(
            _mjpeg_generator(lambda: state.latest_costmap_jpeg),
            media_type='multipart/x-mixed-replace; boundary=frame')

    @app.websocket('/ws/telemetry')
    async def ws_telemetry(ws: WebSocket):
        await ws.accept()
        # See ros_bridge.py's publish_joy/_republish_joy for why this
        # matters: /joy only keeps flowing while at least one browser is
        # actually connected here -- this is the "real disconnect" signal
        # safety_watchdog's fallback should key off, not routine per-frame
        # gamepad jitter.
        ros_node.on_ws_connect()
        send_task = asyncio.create_task(_telemetry_sender(ws))
        try:
            while True:
                raw = await ws.receive_text()
                await _handle_client_message(raw, ros_node, state)
        except WebSocketDisconnect:
            pass
        finally:
            send_task.cancel()
            ros_node.on_ws_disconnect()

    async def _telemetry_sender(ws: WebSocket):
        last_slow = 0.0
        fast_period = 1.0 / FAST_HZ
        slow_period = 1.0 / SLOW_HZ
        try:
            while True:
                await ws.send_text(json.dumps({'type': 'fast', **state.snapshot_fast()}))
                now = time.monotonic()
                if now - last_slow >= slow_period:
                    last_slow = now
                    await ws.send_text(json.dumps({'type': 'slow', **state.snapshot_slow()}))
                await asyncio.sleep(fast_period)
        except (WebSocketDisconnect, asyncio.CancelledError):
            return

    return app


async def _handle_client_message(raw, ros_node: WebUiBridgeNode, state: SharedState):
    try:
        msg = json.loads(raw)
    except (ValueError, TypeError):
        return
    kind = msg.get('type')

    if kind == 'gamepad':
        axes, buttons = translate_gamepad_to_joy_arrays(
            msg.get('axes', []), msg.get('buttons', []))
        ros_node.publish_joy(axes, buttons)
        return

    if kind != 'command':
        return
    action = msg.get('action')
    value = msg.get('value')

    if action == 'drive_source' and value in ('manual', 'semi', 'auto'):
        ros_node.command_drive_source(value)
    elif action == 'track_mode' and value in ('follow', 'flee'):
        ros_node.command_track_mode(value)
    elif action == 'door' and value in ('open', 'close'):
        ros_node.command_door(value)
    elif action == 'estop':
        ros_node.command_estop()
    elif action == 'max_speed' and isinstance(value, (int, float)):
        ros_node.set_speed_scale(value)
    elif action == 'motor_min_speed' and isinstance(value, (int, float)):
        name = msg.get('name')
        ros_node.set_motor_min_speed(name, value)
    elif action == 'auto_enable_override' and isinstance(value, bool):
        ros_node.set_auto_enable_override(value)
    elif action == 'bearing_sign' and isinstance(value, bool):
        ros_node.set_bearing_sign(msg.get('camera'), value)
    elif action == 'reset_costmap':
        ros_node.command_reset_costmap()


def _spin_ros_in_background(node, executor):
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)


def main():
    rclpy.init()
    state = SharedState()
    node = WebUiBridgeNode(state)
    executor = SingleThreadedExecutor()

    ros_thread = threading.Thread(
        target=_spin_ros_in_background, args=(node, executor), daemon=True)
    ros_thread.start()

    app = create_app(state, node)

    try:
        uvicorn.run(app, host='0.0.0.0', port=8088, log_level='info')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
