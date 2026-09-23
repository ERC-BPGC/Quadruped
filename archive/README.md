# Archive

Everything here is **retired, not deleted**. It was moved out of the active
tree during the 2026-09 repo restructure because it was superseded,
duplicated, or dead code — but nothing was thrown away, in case it's useful
as reference later.

## Contents

- **`legacy-docs/`** — the original six overlapping Markdown docs
  (`README.md`, `QUICKSTART.md`, `SUMMARY.md`, `MODIFICATIONS.md`,
  `TRAINING_CHECKLIST.md`, `CRITICAL_FIXES.md`) that used to live under
  `Genesis/examples/Q1/`. They contradicted each other's numbers and three
  of them hardcoded another developer's local machine path. Their accurate
  content was mined into the current root `README.md` and `docs/` instead —
  read those first. Kept here for historical reference only.

- **`legacy-q2-env.py`** — was `Genesis/examples/Q2/q2_env.py`, a
  byte-for-byte duplicate of what is now `behaviors/env.py` (the class
  inside was even still named `Q1Env`). Superseded by the single shared
  `behaviors/env.py` used by all three behaviors.

- **`legacy-q2-eval.py`** — was `Genesis/examples/Q2/q2_eval.py`. Nearly
  identical to the old `q1_eval.py`, including the same dead code path (it
  built a `data` list for CSV export that was never appended to, so the
  output file was always empty). Superseded by the single shared
  `behaviors/eval.py`, which every behavior now uses via `-e <exp_name>`.

- **`q1_eval_actions_200.csv`** — the empty (0-byte) output of that dead
  CSV-export code path. Kept rather than deleted, but it never contained
  data.

- **`q1_visualize.py`, `q1_sensor_debug.py`, `visualize_coordinate_system.py`,
  `visualize_robot_axes.py`, `test_gravity.py`** — one-off debug/diagnostic
  scripts from the investigation into the robot's 90-degree spawn-orientation
  sensor bug. The fix that came out of that investigation now lives
  permanently in `behaviors/env.py` (the `sensor_correction_quat` config).
  These scripts are self-contained (no imports from `behaviors/`) and were
  moved as-is; some load an MJCF model at
  `Genesis/genesis/assets/xml/q1/q1_mjx_full.xml`, which still exists in the
  vendored engine's asset tree, alongside the URDF model the active pipeline
  actually uses.

## Not archived

The vendored Genesis engine (`Genesis/genesis/`, `Genesis/tests/`, upstream
metadata) and all training checkpoints/logs (`logs/`, `Genesis/logs/`) are
deliberately left exactly where they are — cleaning those up is a separate,
deferred decision (see project memory for details), not part of this
restructure.
