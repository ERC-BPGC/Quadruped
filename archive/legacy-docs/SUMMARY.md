# Q1 Quadruped Training - Summary

## What We Did

Adapted the Unitree Go2 quadruped locomotion code for your custom Q1 robot with the following key modifications:

### 1. Reduced DOF: 12 → 8
- **Removed**: Hip joints (no abduction/adduction)
- **Kept**: Thigh and calf joints (2 per leg)
- This simpler configuration makes the problem easier but requires careful tuning

### 2. Changed Robot Model Format
- **From**: URDF (Go2)
- **To**: MuJoCo XML/MJCF (Q1)
- Your robot model is at: `genesis/assets/xml/q1/q1_mjx_full.xml`

### 3. Adjusted Configurations
- **Observation space**: 45 → 33 dimensions
- **Action space**: 12 → 8 dimensions  
- **Default joint angles**: Tuned for Q1's kinematics
- **PD gains**: Increased kp to compensate for fewer DOFs
- **Reward weights**: Emphasized forward tracking, adjusted penalties

### 4. Created Support Files

| File | Purpose |
|------|---------|
| `q1_env.py` | RL environment adapted for Q1 |
| `q1_train.py` | Training script with Q1 configs |
| `q1_eval.py` | Visualization of trained policy |
| `q1_visualize.py` | Quick robot model checker |
| `q1_monitor.py` | Training progress monitor |
| `README.md` | Complete documentation |
| `MODIFICATIONS.md` | Detailed code changes |
| `QUICKSTART.md` | Getting started guide |

## How to Use

### Quick Test (Recommended First)
```bash
# 1. Check robot loads
python examples/Q1/q1_visualize.py

# 2. Quick training test (50 iterations, ~2 min)
python examples/Q1/q1_train.py -e test -B 1024 --max_iterations 50

# 3. Check results
python examples/Q1/q1_eval.py -e test --ckpt 50
```

### Full Training
```bash
# Train for 500 iterations (~1 hour on GPU)
python examples/Q1/q1_train.py -e q1-walking -B 4096 --max_iterations 500

# Evaluate
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
```

## Key Configuration Values

These are the main parameters you might want to tune:

```python
# In q1_train.py, get_cfgs():

# Joint angles (most important!)
"FL_thigh_joint": 0.5,    # Adjust if robot doesn't stand
"FL_calf_joint": -0.3,    # Adjust if robot doesn't stand

# Base height
"base_init_pos": [0.0, 0.0, 0.45],  # Initial height
"base_height_target": 0.35,          # Desired standing height

# Control
"kp": 25.0,              # Joint stiffness
"kd": 0.5,               # Joint damping
"action_scale": 0.3,     # Movement range

# Rewards (higher = more important)
"tracking_lin_vel": 1.5,      # Forward movement
"base_height": -30.0,         # Height maintenance
"lin_vel_z": -2.0,           # Penalize bouncing
"action_rate": -0.01,         # Smooth movements
"similar_to_default": -0.2,   # Stay near default pose
```

## What to Expect

### Early Training (0-100 iterations)
- Robot learns to stand without falling
- Lots of failures and resets
- Reward slowly increasing

### Mid Training (100-300 iterations)
- Robot maintains stable posture
- Some forward movement starts
- Fewer falls

### Late Training (300-500 iterations)
- Consistent forward walking
- Smooth gait emerges
- Near-optimal velocity tracking

## Troubleshooting Decision Tree

```
Robot falls immediately?
├─ YES → Adjust default joint angles
│        Increase base_height penalty
│        Check base_init_pos height
└─ NO
   └─ Robot stands but doesn't move?
      ├─ YES → Increase tracking_lin_vel reward
      │        Decrease similar_to_default penalty
      └─ NO
         └─ Robot moves but tips over?
            ├─ YES → Add ang_vel_xy penalty
            │        Add orientation penalty
            └─ NO
               └─ Robot walks but jerky?
                  ├─ YES → Increase action_rate penalty
                  │        Reduce action_scale
                  └─ NO → Success! 🎉
```

## File Locations

```
/home/ritwik/NUS/Genesis/
├── examples/Q1/              # Your Q1 code (here)
│   ├── q1_env.py
│   ├── q1_train.py
│   ├── q1_eval.py
│   ├── q1_visualize.py
│   ├── q1_monitor.py
│   ├── README.md
│   ├── MODIFICATIONS.md
│   ├── QUICKSTART.md
│   └── logs/                 # Created during training
│       └── q1-walking/
│           ├── model_*.pt    # Checkpoints
│           └── cfgs.pkl      # Configuration
│
└── genesis/assets/xml/q1/    # Your robot model
    ├── q1_mjx_full.xml       # Main model file
    ├── scene_mjx_flat_terrain.xml
    └── meshes/               # STL files
```

## Differences from Go2

| Aspect | Go2 | Q1 |
|--------|-----|-----|
| Joints per leg | 3 (hip, thigh, calf) | 2 (thigh, calf) |
| Total DOF | 12 | 8 |
| Model format | URDF | MuJoCo XML |
| Observations | 45-dim | 33-dim |
| Actions | 12-dim | 8-dim |
| File format | `go2_*.py` | `q1_*.py` |

## Important Notes

1. **No Hip Joints**: Your Q1 robot cannot abduct/adduct legs sideways, only move them forward/backward
2. **Simpler Control**: Fewer joints means easier training but less maneuverability  
3. **Joint Limits**: Check your XML file for joint range limits - they affect training
4. **Base Height**: Critical parameter - if wrong, robot will always fall
5. **Training Time**: ~1 hour on GPU for 500 iterations with 4096 environments

## Next Steps for Improvement

1. **Tune default pose**: Get stable standing first
2. **Increase training time**: Try 1000-2000 iterations
3. **Add more rewards**: Enable optional rewards (ang_vel_xy, orientation, etc.)
4. **Curriculum learning**: Start with just standing, then add walking
5. **Terrain variation**: Add small height variations to improve robustness

## Support Resources

- **Genesis Documentation**: Check Genesis docs for API details
- **rsl_rl Documentation**: For PPO algorithm details
- **Your XML file**: Verify joint names, ranges, and inertias match reality
- **MODIFICATIONS.md**: See exactly what changed from Go2
- **README.md**: Full documentation with examples

## Success Criteria

Your training is successful when:
- ✅ Robot consistently stands for full episode (20 seconds)
- ✅ Forward velocity reaches ~0.4-0.5 m/s
- ✅ Base height maintained within ±0.05m of target
- ✅ No wild oscillations or jerky movements
- ✅ Reward converges and stabilizes

## Contact & Debugging

If you encounter issues:
1. Run `q1_visualize.py` to verify model loads
2. Check that joint names in XML match those in `q1_train.py`
3. Verify base_init_pos height is reasonable for your robot
4. Try reducing number of environments if OOM errors occur
5. Check Genesis and rsl-rl-lib versions

Good luck with your Q1 quadruped! 🚀
