# Quadruped Locomotion RL

Reinforcement-learning training for a custom 8-DOF quadruped robot ("Q1"),
built on the [Genesis](https://github.com/Genesis-Embodied-AI/Genesis)
physics engine and trained with PPO via `rsl-rl-lib`.

Three trained behaviors, sharing one environment implementation:

- **`forward_walk`** — walk in a fixed direction
- **`turn`** — turn in place / while moving, at a randomized yaw rate
- **`diagonal_walk`** — walk at a fixed 45° diagonal

## Quickstart

From the repo root:

```bash
# 1. train (headless, full-size run)
python behaviors/forward_walk/train.py

# 2. (optional, in another terminal) watch progress
python behaviors/monitor.py -e forward_walk

# 3. evaluate a checkpoint once training has produced one
python behaviors/eval.py -e forward_walk --ckpt 200
```

Swap `forward_walk` for `turn` or `diagonal_walk` to train/evaluate the
other behaviors — see [docs/training.md](docs/training.md) for the full
command reference, flags, and checkpoint layout.

## Repository layout

```
walk/
├── README.md              this file
├── docs/
│   ├── environment.md      observation/action space, PD gains, reward terms
│   ├── training.md         how to train/monitor/evaluate each behavior
│   └── troubleshooting.md  known gotchas (sensor frame, asset paths, deps)
├── behaviors/
│   ├── env.py               shared Q1Env environment class
│   ├── eval.py               shared evaluation/inference script
│   ├── monitor.py            training-progress monitor
│   ├── forward_walk/train.py
│   ├── turn/train.py
│   └── diagonal_walk/train.py
├── tools/
│   └── interactive_control.py   keyboard joint-control debugging tool
├── hardware/                 real-robot ODrive motor control + CPG gait —
│                               a separate concern from the sim pipeline
│                               above; see hardware/README.md
├── assets/
│   └── maze_simple.xml      custom scene asset (currently unused by any script)
├── archive/                 superseded/duplicate/dead code, preserved
│                             for reference — see archive/README.md
├── logs/                    new training runs land here (see docs/training.md)
└── Genesis/                 vendored genesis-world v0.3.4 engine, plus
                              the existing (pre-restructure) checkpoints
                              under Genesis/logs/
```

## Requirements

- Python ≥ 3.10, < 3.14
- `genesis-world` 0.3.4 (vendored in `Genesis/`; not yet decided whether to
  keep vendoring it or switch to a pinned pip dependency)
- PyTorch (a recent stable release)
- `rsl-rl-lib==2.2.4` — **not** `rsl-rl` (a differently-named, incompatible
  package); see [docs/troubleshooting.md](docs/troubleshooting.md)
- `pynput`, optional — only needed for `tools/interactive_control.py`

## Environment summary

8 actuated joints (2 per leg, no hip joint), 25-dimensional observation
space, PD control at `kp=100.0, kd=2.5`, PPO with a `[512, 256, 128]`
actor/critic. Full details, including the per-behavior reward weights and
velocity commands, are in [docs/environment.md](docs/environment.md).

## License

Builds on the Genesis physics engine, licensed under Apache 2.0 — see
`Genesis/LICENSE`.
