# Training, monitoring, and evaluation

All commands below assume you run them **from the repo root**
(`walk/`), since log paths (`logs/<exp_name>/...`) are resolved relative to
the current working directory, not to the script's own location.

## The three behaviors

| Behavior | Train script | Default `exp_name` | What it does |
|---|---|---|---|
| Forward walking | `behaviors/forward_walk/train.py` | `forward_walk` | Constant lateral velocity command, no turning |
| Turning | `behaviors/turn/train.py` | `turn` | Randomized yaw-rate command, dominant turning reward |
| Diagonal walking | `behaviors/diagonal_walk/train.py` | `diagonal_walk` | Constant combined forward+lateral (45°) command |

See [environment.md](environment.md) for exactly what each behavior
configures.

## 1. Train

```bash
# forward walking
python behaviors/forward_walk/train.py            # headless, 4096 envs
python behaviors/forward_walk/train.py -v -B 8     # visualized, 8 envs (debugging)

# turning
python behaviors/turn/train.py
python behaviors/turn/train.py -v -B 8

# diagonal walking
python behaviors/diagonal_walk/train.py
python behaviors/diagonal_walk/train.py -v -B 8
```

Common flags (all three scripts):

- `-e, --exp_name`: experiment/log directory name (defaults per table above)
- `-v, --vis`: show the viewer for environment 0 only (all environments
  still train in parallel; this is just visualization)
- `-B, --num_envs`: number of parallel environments (default 4096)
- `--max_iterations`: PPO iterations (default 201)

Each run writes to `logs/<exp_name>/`: a `cfgs.pkl` (the env/obs/reward/
command/train config, pickled — needed by `eval.py` later) and
`model_<iteration>.pt` checkpoints saved every 100 iterations
(`save_interval` in the PPO config).

**Starting a run with an existing `exp_name` wipes that log directory first**
(`shutil.rmtree` if it exists) — pick a new name if you want to keep an
old run around.

## 2. Monitor (optional, while training runs elsewhere)

```bash
python behaviors/monitor.py -e forward_walk --interval 5
```

Polls `logs/<exp_name>/` for new `model_*.pt` checkpoints and prints
progress. Independent of which behavior you're training — `-e` is
required (there's no meaningful default across three different
behaviors).

You can also use TensorBoard directly:

```bash
tensorboard --logdir logs/forward_walk
```

## 3. Evaluate

One shared script handles all three behaviors — pass the experiment name
you trained with:

```bash
python behaviors/eval.py -e forward_walk --ckpt 200
python behaviors/eval.py -e turn --ckpt 200
python behaviors/eval.py -e diagonal_walk --ckpt 200
```

Loads `logs/<exp_name>/cfgs.pkl` and `logs/<exp_name>/model_<ckpt>.pt`,
opens a viewer, and runs the policy indefinitely (zeroed reward scales —
this is pure inference/visualization, not further training). Stop it with
Ctrl+C.

## Checkpoint layout: old vs. new runs

Before this restructure, training scripts lived under `Genesis/examples/`
and were run with `Genesis/` as the working directory, so their logs ended
up at `Genesis/logs/<exp_name>/`. That directory still holds the existing
checkpoints and is left untouched:

```
Genesis/logs/
├── q1-walking(PERFECT)/     # forward walking, old exp_name
├── q4-back_walking/
├── q2-turn-left/
├── q2-turn-left_20251108-004108/
├── q2-turn-left_3(WIGGLE)/
├── q3-diagonal-left/
├── q3-diagonal-right/
├── q1-walking_2025-11-08/          # recovered from other/Quadruped, 2026-09-22
├── q1-walking_2025-11-07/          # recovered from Quadruped_backup_2025_11_07.zip
└── q1-walking-height_2025-11-07/   # recovered from the same zip; a separate
                                      # "height"-focused run — no cfgs.pkl, only
                                      # model_{0,100,200}.pt + the source scripts
                                      # it was trained with
```

The last three directories are **not duplicates** of `q1-walking(PERFECT)`
— three more genuinely distinct trained checkpoints for the same forward-
walking behavior, recovered on 2026-09-22 from the user's original working
copy of this project (`other/Quadruped/`) and its backup zip. Dates are in
the folder names since there are now multiple `q1-walking` variants with
no other way to tell them apart. None of these three have been evaluated
against each other or against `(PERFECT)` yet — that comparison (and
picking a winner) is still open — same deferred-decision bucket as
checkpoint retention generally (see the restructure plan's notes on
`Genesis/logs/` retention).

New runs launched from the relocated scripts (as shown above, from the
repo root) land at top-level `logs/<exp_name>/` instead, using the new
`exp_name` defaults (`forward_walk`, `turn`, `diagonal_walk`). If you want
to resume evaluating an *old* checkpoint with the new `behaviors/eval.py`,
either copy/move its folder from `Genesis/logs/<name>/` to `logs/<name>/`,
or point the script at it by passing the matching `-e <name>` and running
from inside `Genesis/` instead of the repo root — `log_dir` is always just
`f"logs/{exp_name}"` relative to wherever you invoke the script.

Consolidating the two `logs/` locations (and deciding which of the now 10
old runs are worth keeping) is a deferred cleanup, not part of this
restructure.

## Interactive joint control (debugging tool, not part of the RL pipeline)

```bash
python tools/interactive_control.py
```

Spawns the robot and lets you nudge individual joints with the keyboard
(1-8 to select a joint, +/-/W/S/A/D to move it, R to reset, P to print
state, H for help, Esc to quit). Useful for sanity-checking a default pose
or joint limits before training. Requires the optional `pynput` dependency
— see [troubleshooting.md](troubleshooting.md).
