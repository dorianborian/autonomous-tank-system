# ODIN 1 vendor driver

The front sensor (ODIN 1 spatial memory module — dToF depth + RGB + IMU +
onboard SLAM, USB-C) is driven by Manifold's vendor ROS 2 driver. It is
**not vendored into this repository** — it's third-party code with its own
license and upstream release cadence, and committing a fork of it here
would just drift out of sync with upstream fixes.

- **Upstream:** https://github.com/manifoldsdk/odin_ros_driver
- **Commit pinned for this robot:** see [`COMMIT_PINNED.txt`](COMMIT_PINNED.txt)
  (`388440c`, tagged "0.12.0 release", 2026-06-18)

To set it up:

```bash
mkdir -p ~/odin_ws/src && cd ~/odin_ws/src
git clone https://github.com/manifoldsdk/odin_ros_driver.git
cd odin_ros_driver
git checkout 388440c977af579ea81c3c242ab36fb958d3be82
git apply /path/to/this/repo/vendor/odin_ros_driver/patches/*.patch
cd ~/odin_ws && colcon build
```

## Local patch: `script/build_ros2.sh` `$WS_DIR` typo

[`patches/0001-fix-WS_DIR-typo-in-build_ros2.sh.patch`](patches/0001-fix-WS_DIR-typo-in-build_ros2.sh.patch)

The upstream `build_workspace()` function does:

```bash
cd $WS_DIR
rm -rf build install log
```

`WS_DIR` is never defined anywhere in the script — the variable that *is*
computed and intended for this purpose is `WORKSPACE_ROOT`. Because the
undefined `$WS_DIR` expands to nothing, `cd $WS_DIR` is equivalent to a bare
`cd`, which changes to `$HOME`. The very next line, `rm -rf build install
log`, then deletes `~/build`, `~/install`, and `~/log` — not the workspace's
build artifacts — if those directories happen to exist in the user's home
directory. This is a real, if narrow, footgun: worth reporting upstream. The
patch here simply changes `cd $WS_DIR` to `cd "$WORKSPACE_ROOT"`, matching
what the rest of the script already computes and quoting it for paths with
spaces.

**This has not yet been reported upstream** — do that (open an issue or PR
against `manifoldsdk/odin_ros_driver`) rather than only living as a local
patch.
