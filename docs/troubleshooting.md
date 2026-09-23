# Troubleshooting & gotchas

## The spawn-orientation / sensor-frame quirk

The robot is spawned with `base_init_quat = (cos45°, -sin45°, 0, 0)` — a
90° rotation about the X-axis, purely so it renders upright in the viewer.
Genesis reports velocities and projected gravity in the robot's *raw* base
frame though, and that frame is rotated 90° relative to what visually
looks like "forward" — so naively using the raw quaternion to compute
`base_lin_vel` / `base_ang_vel` / `projected_gravity` produces
readings that don't line up with the visible direction of motion.

`behaviors/env.py` fixes this by applying a second, compensating rotation
— `sensor_correction_quat = (cos45°, sin45°, 0, 0)` (+90° about X) — before
transforming velocities and gravity into the body frame (see `step()`).
Termination checks (roll/pitch) deliberately use the **uncorrected** raw
quaternion instead, since those need the robot's actual physical tilt, not
the sensor-corrected reading.

**Practical consequence:** in the corrected frame, "forward" maps to world
**-y**, not world x. That's why `forward_walk/train.py` commands
`lin_vel_y_range = [-0.65, -0.65]` with `lin_vel_x_range = [0, 0]` — despite
the name, the actual world-frame command is lateral, not what you'd
naively call "x-forward." This is intentional, not a leftover bug — don't
"fix" it by swapping the ranges back to x without re-verifying against the
robot's actual visible forward direction in the viewer.

This was originally diagnosed with a handful of one-off debug scripts,
now preserved in [`archive/`](../archive/README.md):
`q1_sensor_debug.py`, `visualize_coordinate_system.py`,
`visualize_robot_axes.py`. If a future orientation bug shows up again,
those are a reasonable starting point.

## Asset paths: URDF vs. MJCF, and why `genesis_asset_path()` exists

Genesis resolves a `morph.file` string via `os.path.join(<genesis package's
own assets dir>, morph.file)` — i.e. relative to wherever the `genesis`
Python package is installed, not relative to your current working
directory. The custom Q1 robot's URDF lives inside the **vendored** copy of
the engine at `Genesis/genesis/assets/urdf/q1/kutta.urdf` — it was added
directly into the vendored engine's own asset tree rather than kept
separate.

`behaviors/env.py` sidesteps any ambiguity about how `genesis-world` is
installed (editable from `Genesis/`, a separate pip install, etc.) by
computing an **absolute** path via `genesis_asset_path()`, anchored to
`__file__`:

```python
GENESIS_ROOT = pathlib.Path(__file__).resolve().parent.parent / "Genesis"

def genesis_asset_path(relative_path):
    return str(GENESIS_ROOT / relative_path)
```

Since `os.path.join(a, b)` simply returns `b` when `b` is already absolute,
passing an absolute path here makes asset loading work regardless of
invocation directory or install mode. If you add a new script that loads a
Genesis asset, import and reuse this helper (`tools/interactive_control.py`
does) rather than hardcoding a relative path.

Separately: a handful of archived debug scripts load
`genesis/assets/xml/q1/q1_mjx_full.xml` (MJCF format) — an older robot
model from before the pipeline switched to URDF. That file still exists
in the vendored asset tree, but nothing in the active `behaviors/`
pipeline uses it.

## `rsl-rl` vs. `rsl-rl-lib`

Every train/eval script starts with a version guard that raises
`ImportError` unless `rsl-rl-lib==2.2.4` is installed (and `rsl-rl`, a
different, incompatible package with a similar name, is *not*). If you hit
this on setup:

```bash
pip uninstall rsl_rl
pip install rsl-rl-lib==2.2.4
```

## `pynput` is an undeclared dependency

`tools/interactive_control.py` imports `pynput` for keyboard input. It's
not required by anything else in this repo and isn't pinned anywhere —
install it separately (`pip install pynput`) only if you need that tool.

## Working directory matters for logs, not for assets

Asset loading (`genesis_asset_path()`) is invocation-independent by
design. Checkpoint/log paths are **not** — `log_dir = f"logs/{exp_name}"`
is always relative to wherever you run the script from. Run training/eval/
monitor scripts from the repo root unless you deliberately want logs to
land somewhere else (see [training.md](training.md) for the old-vs-new
`logs/` layout this causes).

## Black/blank viewer window (`-v`, `eval.py`)

If `python behaviors/eval.py ...` (or any `-v`/`show_viewer=True` run)
opens a window titled "Genesis x.y.z" but its contents are **solid
black** — no robot, no ground plane, even though the log keeps printing
`Running at NN FPS` (i.e. the sim itself is stepping fine) — check for
`__NV_PRIME_RENDER_OFFLOAD` and `__GLX_VENDOR_LIBRARY_NAME` in your
shell environment:

```bash
echo $__NV_PRIME_RENDER_OFFLOAD $__GLX_VENDOR_LIBRARY_NAME
```

These two variables tell the NVIDIA driver to route OpenGL rendering
through **PRIME offload** — a mechanism for hybrid-GPU laptops (Intel
iGPU driving the display, NVIDIA dGPU doing the rendering, output
copied back to the iGPU's surface). On a single-GPU machine (check
`lspci | grep -i vga` — only one GPU listed), there's nothing to
"offload" to, but if these vars are set anyway (e.g. inherited from a
dotfile written for a different machine), the NVIDIA driver still takes
the offload code path, and its output silently fails to composite into
the actual visible window under GNOME/Mutter — producing exactly this
black-window symptom with no error anywhere in the log.

**Fix** — unset both for the launch:

```bash
env -u __NV_PRIME_RENDER_OFFLOAD -u __GLX_VENDOR_LIBRARY_NAME \
    python behaviors/eval.py -e forward_walk --ckpt 200
```

Confirmed root-caused by screenshotting the viewer window directly
(`import -window <id> out.png`, window id via `xdotool search --name
Genesis`) — file size jumped from a ~200-byte flat-black PNG to a real
~58KB rendered frame once these vars were unset, and consecutive
screenshots differed, confirming live rendering (not just a static
frame). Neither switching `PYOPENGL_PLATFORM` between `glx`/`egl`/`native`
nor anything else changed the result — it was specifically these two
PRIME vars.

## No live smoke test available in some environments

If you're validating changes to `behaviors/env.py` or the training
scripts and don't have `torch` + `genesis-world` installed locally, you
can still sanity-check path resolution and imports statically — e.g.
confirm `genesis_asset_path(...)` points at a file that actually exists on
disk, or grep for stale references to old module/file names — without
running the full simulation.
