# Environment reference (`behaviors/env.py`)

`Q1Env` (in `behaviors/env.py`) is the single environment class shared by
all three behaviors (`forward_walk`, `turn`, `diagonal_walk`). Each
behavior's `train.py` builds a different `env_cfg` / `reward_cfg` /
`command_cfg` and passes it into the same class — the physics, observation
layout, and reward *implementations* are identical across behaviors; only
the numbers differ.

## Robot

- Custom quadruped ("Q1"/"kutta"), URDF at
  `Genesis/genesis/assets/urdf/q1/kutta.urdf` (loaded via the
  `genesis_asset_path()` helper at the top of `env.py`, anchored to
  `__file__` so it resolves regardless of the invocation directory).
- 8 actuated joints, 2 per leg (thigh + calf only — **no hip joint**, unlike
  12-DOF quadrupeds such as Unitree Go2 this project was originally ported
  from).
- Joint order: `FL_thigh, FL_calf, FR_thigh, FR_calf, RL_thigh, RL_calf,
  RR_thigh, RR_calf`.
- Default standing pose: front legs `thigh=0.45 rad, calf=0.15 rad`; rear
  legs `thigh=0.25 rad, calf=0.15 rad`.
- PD control: `kp=100.0`, `kd=2.5` for all three behaviors.
- Spawns with `base_init_quat = (cos45°, -sin45°, 0, 0)` — a 90° rotation
  about the X-axis so the robot appears upright in the viewer. See
  [troubleshooting.md](troubleshooting.md) for why this matters and how
  `sensor_correction_quat` compensates for it.

## Observation space (25-dim)

Built in `Q1Env.step()`:

| Component | Dims | Scale |
|---|---|---|
| Base angular velocity | 3 | `obs_scales["ang_vel"]` = 0.25 |
| Projected gravity (body frame) | 3 | none |
| Velocity commands | 3 | `[lin_vel, lin_vel, ang_vel]` scales |
| Joint positions (relative to default) | 8 | `obs_scales["dof_pos"]` = 1.0 |
| Last actions | 8 | none |

**Note:** joint velocities are computed (`self.dof_vel`) but deliberately
excluded from the observation vector — the line is present in `step()` but
commented out. 3+3+3+8+8 = 25. (An older comment in one training script
said "33 (instead of 45 for Go2)" — that was stale from before `dof_vel`
was excluded; the actual, current dimension is 25.)

## Action space

- 8-dimensional continuous, one value per joint.
- Applied as `target_dof_pos = action * action_scale + default_dof_pos`
  (action_scale = 0.25 for all three behaviors).
- Clipped to `±clip_actions` (100.0) before use.
- `simulate_action_latency = True`: the action actually executed each step
  is the *previous* step's action (`last_actions`), not the current one —
  a simple one-step-latency model.

## Episode / termination

- `dt = 0.02` (50 Hz control), `episode_length_s = 20.0` → max 1000 steps.
- Commands resample every `resampling_time_s = 4.0` seconds.
- Episode terminates (and resets) when `|roll| > 15°` or `|pitch| > 15°`,
  computed from the **raw** (uncorrected) base quaternion — termination
  checks intentionally do not use the sensor-corrected frame.

## Reward terms

Nine `_reward_*` methods are implemented on `Q1Env`; which ones are
*active* (nonzero scale) depends on the behavior's `reward_cfg`:

| Reward | What it does |
|---|---|
| `tracking_lin_vel` | Exponential reward for matching commanded xy linear velocity |
| `tracking_ang_vel` | Exponential reward for matching commanded yaw rate |
| `lin_vel_z` | Penalizes vertical (bouncing) velocity |
| `action_rate` | Penalizes large changes between consecutive actions |
| `similar_to_default` | Penalizes joint poses far from the default stance |
| `base_height` | Penalizes deviation from a target base height |
| `upright` | Rewards `-projected_gravity_z` (≈1.0 when level) |
| `tilt_penalty` | Penalizes gravity's projection onto the body xy-plane (tilt) |
| `base_ang_vel` | Penalizes roll/pitch angular velocity (wobble) |

Five additional reward helpers (`ang_vel_xy`, `orientation`, `dof_vel`,
`dof_acc`, `collision`) are left commented out at the bottom of the class —
disabled scaffolding from the original Go2→Q1 port, kept as optional
starting points rather than deleted.

Per-behavior reward weights (see each `train.py`'s `get_cfgs()` for the
authoritative source):

| Reward | forward_walk | turn | diagonal_walk |
|---|---|---|---|
| `tracking_lin_vel` | 4.0 | 0.2 | 5.0 |
| `tracking_ang_vel` | 1.5 | 3.0 | 5.0 |
| `lin_vel_z` | -0.5 | -0.5 | -0.5 |
| `base_height` (target) | -50.0 (0.55m) | -50.0 (0.45m) | -30.0 (0.55m) |
| `action_rate` | -0.05 | -0.05 | -0.05 |
| `similar_to_default` | -0.2 | -0.2 | -0.1 |
| `upright` | 5.0 | — | 3.0 |
| `tilt_penalty` | -2.0 | — | -1.0 |
| `base_ang_vel` | -0.2 | — | -0.05 |

`turn` only enables 6 of the 9 reward terms (no `upright` / `tilt_penalty`
/ `base_ang_vel`) — it relies on the dominant `tracking_ang_vel` weight
instead.

## Velocity commands per behavior

| Behavior | `lin_vel_x_range` | `lin_vel_y_range` | `ang_vel_range` |
|---|---|---|---|
| `forward_walk` | `[0, 0]` | `[-0.65, -0.65]` (fixed) | `[0, 0]` |
| `turn` | `[0, 0]` | `[0, 0]` | `[0.8, 1.2]` (randomized each resample) |
| `diagonal_walk` | `[0.4, 0.4]` (fixed) | `[-0.4, -0.4]` (fixed) | `[0, 0]` |

`forward_walk`'s commanded velocity is along world **-y**, not x — this is
not a bug, it's a consequence of the sensor-frame correction described in
[troubleshooting.md](troubleshooting.md).

## Port history (Go2 → Q1)

This environment started as a port of a 12-DOF Unitree Go2 locomotion
environment. Key adaptations, for context if you're comparing against Go2
reference code: hip joints removed (12→8 DOF), observation size reduced
accordingly, PD gains raised substantially (early versions used kp≈20-25;
the current, working value is kp=100/kd=2.5), and the robot model was
originally loaded as MJCF (`genesis/assets/xml/q1/q1_mjx_full.xml`, still
present and used by a couple of archived debug scripts) before the active
pipeline switched to URDF (`genesis/assets/urdf/q1/kutta.urdf`).
