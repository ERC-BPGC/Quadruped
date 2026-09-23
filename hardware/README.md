# Hardware: real-robot motor control

This directory holds the code for driving the **physical** Q1 quadruped via
its ODrive motor controllers, plus a standalone CPG (Central Pattern
Generator) gait algorithm. It's a separate concern from the rest of the
repo: everything under `behaviors/` trains and evaluates a policy in the
Genesis **simulator**; nothing here is imported by or imports from
`behaviors/`.

Recovered on 2026-09-22 from `Quadruped_backup_2025_11_07.zip` (the
original working repo's own backup, `other/Quadruped/`), where this code
lived at `Genesis/Quadruped_local/`. It was never part of `walk/`'s git
history.

## Layout

```
hardware/
├── odrive_control/       ODrive GUI/scripts for driving the real motors
│                          (quadruped_gui.py, quadruped_enhanced.py,
│                          quadruped_oop.py, Motor_script.py, calibration
│                          and plotting utilities — see its own README.md)
├── gait/
│   └── quadruped_cpg.py  CPG-based gait generator, independent of RL
├── robot_description/
│   ├── Assem1.urdf        SolidWorks-exported URDF of the physical robot
│   └── *.STL               the 9 link meshes it references (8 legs +
│                            base_link) — kept flat alongside the URDF
│                            since Assem1.urdf references them by bare
│                            filename (`filename="FL_calf.STL"`), not a
│                            `meshes/` subpath
└── model_200.pt           a trained policy checkpoint that was bundled
                            with this hardware code in the original repo —
                            distinct architecture/size from the
                            `Genesis/logs/` training checkpoints, looks
                            like the model actually deployed to the real
                            robot rather than a training-log artifact
```

## Notes on what was and wasn't recovered

- The backup zip's `Quadruped_local/` also had a `kkkkkk.urdf` (same
  robot, but referencing meshes via a ROS `package://kkkkkk/meshes/...`
  path — not usable standalone without a ROS package setup) and a
  `meshes/` folder / `meshes.zip` containing only 5 files (front legs +
  base_link, missing both rear legs entirely). **Neither `Assem1.urdf` nor
  `kkkkkk.urdf` will load correctly against that `meshes/` set** — both
  reference all 8 leg meshes. The complete, correct mesh set was actually
  in the folder named `meshes_old/` (all 8 legs + `base_link.STL`, despite
  the name suggesting it's superseded) — that's what's copied into
  `robot_description/` here. `kkkkkk.urdf`, `meshes/`, `meshes.zip`, and
  `base_link_old.STL` (an extra, unreferenced mesh variant) were left out
  as redundant/unusable; they still exist untouched in the original zip at
  `other/Quadruped/Quadruped_backup_2025_11_07.zip` if ever needed.
- This code has not been run or tested since recovery — treat it as
  archaeological recovery, not verified-working.
