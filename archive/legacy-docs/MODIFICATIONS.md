# Q1 Quadruped Code Modifications Summary

## Overview
This document summarizes the modifications made to adapt the Unitree Go2 locomotion code for the Q1 custom quadruped robot.

## Key Differences: Q1 vs Go2

| Feature | Go2 | Q1 |
|---------|-----|-----|
| **DOF per leg** | 3 (hip, thigh, calf) | 2 (thigh, calf) |
| **Total DOF** | 12 | 8 |
| **Action Space** | 12-dimensional | 8-dimensional |
| **Observation Space** | 45-dimensional | 33-dimensional |
| **Model Format** | URDF | MuJoCo XML (MJCF) |
| **Hip Joint** | Yes | No |

## Files Modified

### 1. `q1_env.py` (Environment)

**Class Name Change:**
- `Go2Env` → `Q1Env`

**Robot Loading:**
```python
# Old (Go2):
gs.morphs.URDF(file="urdf/go2/urdf/go2.urdf", ...)

# New (Q1):
gs.morphs.MJCF(file="xml/q1/q1_mjx_full.xml", ...)
```

**Joint Configuration:**
```python
# Old (Go2): 12 joints
joint_names = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

# New (Q1): 8 joints
joint_names = [
    "FL_thigh_joint", "FL_calf_joint",
    "FR_thigh_joint", "FR_calf_joint",
    "RL_thigh_joint", "RL_calf_joint",
    "RR_thigh_joint", "RR_calf_joint",
]
```

**Observation Space:**
```python
# Old (Go2): 45 dims = 3 + 3 + 3 + 12 + 12 + 12
# New (Q1): 33 dims = 3 + 3 + 3 + 8 + 8 + 8
self.obs_buf = torch.cat([
    self.base_ang_vel * self.obs_scales["ang_vel"],        # 3
    self.projected_gravity,                                 # 3
    self.commands * self.commands_scale,                    # 3
    (self.dof_pos - self.default_dof_pos) * scales["dof_pos"],  # 8 (was 12)
    self.dof_vel * self.obs_scales["dof_vel"],             # 8 (was 12)
    self.actions,                                           # 8 (was 12)
], axis=-1)
```

**Additional Reward Functions Added:**
- `_reward_ang_vel_xy()`: Penalizes roll/pitch rates
- `_reward_orientation()`: Penalizes tilting
- `_reward_dof_vel()`: Penalizes high joint velocities
- `_reward_dof_acc()`: Penalizes high joint accelerations
- `_reward_collision()`: Placeholder for collision penalties

### 2. `q1_train.py` (Training Script)

**Environment Import:**
```python
# Old: from go2_env import Go2Env
# New: from q1_env import Q1Env
```

**Configuration Changes:**

```python
env_cfg = {
    "num_actions": 8,  # Changed from 12
    
    # Default joint angles adjusted for Q1's kinematics
    "default_joint_angles": {
        "FL_thigh_joint": 0.5,   # vs Go2's hip:0.0, thigh:0.8
        "FL_calf_joint": -0.3,    # vs Go2's calf:-1.5
        # ... (8 joints instead of 12)
    },
    
    # Joint ordering changed to match Q1's structure
    "joint_names": [
        "FL_thigh_joint", "FL_calf_joint",
        "FR_thigh_joint", "FR_calf_joint",
        "RL_thigh_joint", "RL_calf_joint",
        "RR_thigh_joint", "RR_calf_joint",
    ],
    
    # PD gains adjusted for Q1
    "kp": 25.0,  # vs Go2's 20.0 (higher due to fewer DOFs)
    "kd": 0.5,   # Same
    
    # More tolerant termination thresholds
    "termination_if_roll_greater_than": 15,   # vs Go2's 10
    "termination_if_pitch_greater_than": 15,  # vs Go2's 10
    
    # Base height adjusted for Q1
    "base_init_pos": [0.0, 0.0, 0.45],  # vs Go2's 0.42
    
    # Action scale increased for more movement range
    "action_scale": 0.3,  # vs Go2's 0.25
}

obs_cfg = {
    "num_obs": 33,  # Changed from 45
    # obs_scales remain the same
}

reward_cfg = {
    "base_height_target": 0.35,  # vs Go2's 0.3
    "reward_scales": {
        "tracking_lin_vel": 1.5,      # vs 1.0 (emphasize forward motion)
        "tracking_ang_vel": 0.5,       # vs 0.2
        "lin_vel_z": -2.0,            # vs -1.0 (more penalty)
        "base_height": -30.0,         # vs -50.0 (less penalty)
        "action_rate": -0.01,         # vs -0.005
        "similar_to_default": -0.2,   # vs -0.1
    },
}

# Command configuration (same as Go2 for forward walking)
command_cfg = {
    "lin_vel_x_range": [0.5, 0.5],  # Forward at 0.5 m/s
    "lin_vel_y_range": [0, 0],       # No lateral
    "ang_vel_range": [0, 0],         # No turning
}
```

**Training Parameters:**
```python
parser.add_argument("-e", "--exp_name", default="q1-walking")  # vs "go2-walking"
parser.add_argument("--max_iterations", default=500)           # vs 101
```

### 3. `q1_eval.py` (Evaluation Script)

**Changes:**
- Import changed to `Q1Env`
- Default experiment name: `"q1-walking"`
- Default checkpoint: `500` (vs Go2's `100`)

## New Files Created

### 1. `q1_visualize.py`
A standalone script to visualize the Q1 robot in its default pose. Useful for:
- Verifying the robot model loads correctly
- Checking joint configurations
- Adjusting default joint angles before training

### 2. `README.md`
Comprehensive documentation including:
- Robot specifications
- Configuration details
- Training instructions
- Troubleshooting guide
- Tips for parameter tuning

## Rationale for Key Changes

### 1. Higher PD Gain (kp: 25.0 vs 20.0)
With fewer DOFs, each joint needs to be more responsive to achieve similar movement capabilities.

### 2. More Tolerant Termination (15° vs 10°)
Q1 has less ability to correct orientation without hip joints, so we allow slightly more tilt before termination.

### 3. Larger Action Scale (0.3 vs 0.25)
Without hip joints, the thigh and calf need a larger range of motion to achieve forward movement.

### 4. Higher Forward Velocity Reward (1.5 vs 1.0)
Emphasizes the primary objective of forward walking, compensating for the reduced DOFs.

### 5. Increased Vertical Velocity Penalty (-2.0 vs -1.0)
Q1 may have less stability without hip joints, so we penalize bouncing more heavily.

### 6. Adjusted Base Height (0.45 vs 0.42, target 0.35 vs 0.3)
Based on Q1's physical dimensions and standing height.

## Training Recommendations

### Initial Training
```bash
python examples/Q1/q1_train.py -e q1-test -B 2048 --max_iterations 200
```
Start with fewer environments and iterations to quickly verify the setup works.

### Full Training
```bash
python examples/Q1/q1_train.py -e q1-walking -B 4096 --max_iterations 500
```

### Monitoring
Watch for:
- Episode length increasing over time
- Tracking reward increasing
- Base height stabilizing

### Tuning
If the robot:
- **Falls immediately**: Increase `base_height` penalty, adjust `default_joint_angles`
- **Doesn't move forward**: Increase `tracking_lin_vel` reward, decrease `similar_to_default`
- **Is too jerky**: Increase `action_rate` penalty, reduce `action_scale`
- **Tips over**: Increase termination thresholds, add `ang_vel_xy` and `orientation` penalties

## Optional Reward Functions

The following reward functions are implemented but not enabled by default:
- `ang_vel_xy`: Penalize roll/pitch rates
- `orientation`: Penalize tilting from level
- `dof_vel`: Penalize high joint velocities
- `dof_acc`: Penalize joint accelerations

To enable, add to `reward_cfg["reward_scales"]` in `q1_train.py`:
```python
"ang_vel_xy": -0.05,
"orientation": -1.0,
"dof_vel": -0.01,
"dof_acc": -2.5e-7,
```

## Testing Workflow

1. **Visualize**: Run `q1_visualize.py` to verify robot loads correctly
2. **Quick Test**: Train for 200 iterations with fewer environments
3. **Adjust**: Tune parameters based on behavior
4. **Full Training**: Run for 500+ iterations with 4096 environments
5. **Evaluate**: Use `q1_eval.py` to visualize trained policy

## Differences from Go2 Code Structure

The code structure remains largely the same, with these key differences:
- Robot loading uses `MJCF` instead of `URDF`
- All arrays sized for 8 joints instead of 12
- Joint names exclude hip joints
- Default configurations adjusted for Q1's kinematics
