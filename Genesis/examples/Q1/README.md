# Q1 Quadruped Walking Task

This folder contains the training and evaluation code for a simple forward walking task for the Q1 custom quadruped robot.

## Robot Specifications

The Q1 quadruped has:
- **8 Degrees of Freedom (DOF)**: 2 per leg (thigh and calf joints only, no hip joint)
- **Joint Configuration**: 
  - Front Left (FL): thigh, calf
  - Front Right (FR): thigh, calf
  - Rear Left (RL): thigh, calf
  - Rear Right (RR): thigh, calf

This is different from robots like the Unitree Go2 which have 12 DOF (3 per leg: hip, thigh, and calf).

## Files

- `q1_env.py`: Environment class for Q1 robot with RL training interface
- `q1_train.py`: Training script using PPO algorithm
- `q1_eval.py`: Evaluation script to visualize trained policy
- `q1_visualize.py`: Standalone script to visualize robot in default pose
- `q1_monitor.py`: Real-time training progress monitor
- `README.md`: This file
- `MODIFICATIONS.md`: Detailed documentation of changes from Go2 code

## MuJoCo XML Files

The robot model is located at: `genesis/assets/xml/q1/q1_mjx_full.xml`

## Configuration

### Environment Configuration
- **Actions**: 8 continuous actions (one per joint)
- **Observations**: 33-dimensional vector containing:
  - Base angular velocity (3)
  - Projected gravity (3)
  - Velocity commands (3)
  - Joint positions relative to default (8)
  - Joint velocities (8)
  - Previous actions (8)

### Default Joint Angles
```python
"FL_thigh_joint": 0.5,
"FL_calf_joint": -0.3,
"FR_thigh_joint": 0.5,
"FR_calf_joint": -0.3,
"RL_thigh_joint": 0.5,
"RL_calf_joint": -0.3,
"RR_thigh_joint": 0.5,
"RR_calf_joint": -0.3,
```

### Reward Function
The reward function includes:
- **tracking_lin_vel** (1.5): Tracks desired forward velocity (0.5 m/s)
- **tracking_ang_vel** (0.5): Tracks desired angular velocity (0 rad/s)
- **lin_vel_z** (-2.0): Penalizes vertical movement
- **base_height** (-30.0): Maintains desired base height (0.35m)
- **action_rate** (-0.01): Encourages smooth actions
- **similar_to_default** (-0.2): Keeps joints near default pose

## Training

### Step 1: Visualize the Robot (Optional but Recommended)

First, verify that your robot model loads correctly:

```bash
cd /home/ritwik/NUS/Genesis
python examples/Q1/q1_visualize.py
```

This will open a viewer showing the Q1 robot in its default standing pose. Check that:
- The robot loads without errors
- All 8 joints are reported correctly
- The standing pose looks stable

### Step 2: Start Training

To train the Q1 robot for forward walking:

```bash
cd /home/ritwik/NUS/Genesis
python examples/Q1/q1_train.py
```

Optional arguments:
- `-e, --exp_name`: Experiment name (default: "q1-walking")
- `-B, --num_envs`: Number of parallel environments (default: 4096)
- `--max_iterations`: Maximum training iterations (default: 500)

Example:
```bash
python examples/Q1/q1_train.py -e q1-forward -B 2048 --max_iterations 1000
```

### Step 3: Monitor Training (Optional)

In a separate terminal, you can monitor training progress:

```bash
cd /home/ritwik/NUS/Genesis
python examples/Q1/q1_monitor.py -e q1-walking --interval 10
```

This will display updates about training checkpoints. You can also use TensorBoard:

```bash
tensorboard --logdir examples/Q1/logs/q1-walking
```

## Evaluation

To evaluate a trained policy:

```bash
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
```

Arguments:
- `-e, --exp_name`: Experiment name to load
- `--ckpt`: Checkpoint number to load (default: 500)

## Training Tips

1. **Initial Training**: Start with a lower number of iterations (500) to check if the robot learns basic stability
2. **Adjust Default Pose**: If the robot struggles to stand, adjust the default joint angles in `q1_train.py`
3. **Tuning PD Gains**: The gains are set to kp=25.0, kd=0.5. If joints oscillate, reduce kp. If joints are too sluggish, increase kp.
4. **Base Height**: The base_init_pos z-coordinate is set to 0.45m. Adjust this based on your robot's actual standing height.
5. **Reward Tuning**: 
   - Increase `tracking_lin_vel` weight if robot doesn't move forward enough
   - Increase `base_height` penalty if robot crouches too much
   - Adjust `action_rate` if actions are too jerky or too conservative

## Troubleshooting

### Robot Falls Immediately
- Check if default joint angles result in a stable standing pose
- Increase `base_height` penalty
- Verify base_init_pos height is appropriate

### Robot Doesn't Move Forward
- Increase `tracking_lin_vel` reward weight
- Decrease `similar_to_default` penalty
- Check if action_scale is appropriate (currently 0.3)

### Joint Oscillations
- Reduce PD gains (kp, kd)
- Increase `action_rate` penalty
- Check if joint limits in XML are appropriate

### Robot Tips Over
- Increase termination thresholds for roll/pitch
- Add more penalty for angular velocities
- Adjust base_height_target

## Next Steps

After achieving stable forward walking, you can:
1. Add turning capability by adjusting `ang_vel_range` in command_cfg
2. Add lateral movement by adjusting `lin_vel_y_range`
3. Add terrain variations
4. Implement velocity tracking for variable speed commands
5. Add additional reward terms for energy efficiency or foot clearance
