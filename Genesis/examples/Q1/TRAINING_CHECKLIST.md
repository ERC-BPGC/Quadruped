# Q1 Training Pre-Flight Checklist

## ✅ Configuration Review

### PD Gains (CRITICAL for stability)
- **kp: 100.0** (increased from 25.0) - High proportional gain for strong position tracking
- **kd: 2.0** (increased from 0.5) - High derivative gain for damping oscillations
- These gains match what works in the interactive control script
- ⚠️ If robot still collapses during training, increase kp to 150-200

### Joint Configuration
- **8 DOF total**: 2 per leg (thigh + calf only, no hip abduction)
- **Joint order**: FL_thigh, FL_calf, FR_thigh, FR_calf, RL_thigh, RL_calf, RR_thigh, RR_calf
- **Default angles** (standing pose):
  - Front legs: thigh=0.45 rad, calf=0.15 rad
  - Rear legs: thigh=0.25 rad, calf=0.15 rad
- **Joint limits**:
  - Thigh: -0.9 to 0.67 rad
  - Calf: -0.5 to 0.5 rad (check URDF if using different values)

### Observations (33 dims)
- Base angular velocity (3) - scaled by 0.25
- Projected gravity (3) - no scaling
- Commands (3) - lin_vel_x, lin_vel_y, ang_vel_z
- DOF positions (8) - difference from default, scaled by 1.0
- DOF velocities (8) - scaled by 0.05
- Last actions (8) - no scaling

### Rewards
- `tracking_lin_vel`: 2.5 (primary objective - forward movement)
- `tracking_ang_vel`: 0.5 (turning)
- `lin_vel_z`: -1.0 (penalize vertical movement)
- `base_height`: -50.0 (stay at target height)
- `action_rate`: -0.005 (smooth actions)
- `similar_to_default`: -0.1 (stay near default pose)

### Commands
- **Forward velocity**: 0.5 m/s (constant)
- **Lateral velocity**: 0 m/s
- **Angular velocity**: 0 rad/s
- Commands resample every 4 seconds

### Termination Conditions
- Episode length: 10 seconds (500 steps at 50Hz)
- Roll > 15 degrees
- Pitch > 15 degrees
- Base height issues handled by reward

## 🎮 Visualization During Training

**YES!** The `-v` flag is now supported:

```bash
# Train with visualization (shows environment 0 only)
python examples/Q1/q1_train.py -v -B 8

# Train without visualization (faster, more envs)
python examples/Q1/q1_train.py -B 2048

# Quick debug run
python examples/Q1/q1_train.py -v -B 4 --max_iterations 10
```

**Note**: Only the first environment (index 0) is visualized to save GPU memory and rendering overhead. The policy still trains on all environments in parallel.

## 📊 Sensor Readings

### Current Implementation
The environment uses these sensor readings (all from Genesis API):
- `robot.get_pos()` - base position (x, y, z)
- `robot.get_quat()` - base orientation (quaternion)
- `robot.get_vel()` - base linear velocity
- `robot.get_ang()` - base angular velocity  
- `robot.get_dofs_position()` - joint positions
- `robot.get_dofs_velocity()` - joint velocities

### Do You Need to Check Sensors?
**For basic locomotion: NO, these readings are sufficient.**

The current sensor setup matches what's used in successful locomotion policies (like Go2). However, you COULD add:

1. **Contact sensors** (foot contact detection):
   - Useful for gait learning
   - Can add `_reward_feet_contact` if you want to encourage proper stepping
   - Genesis supports this via collision detection

2. **IMU sensor** (if you want to match real robot more closely):
   - The MJCF file already has an IMU sensor defined
   - Currently we compute orientation from `get_quat()` which is essentially IMU data
   - Not necessary unless you want to add noise/latency simulation

3. **Torque sensors** (joint effort feedback):
   - Useful for more advanced control
   - Can help with terrain adaptation
   - Not needed for flat ground walking

**Recommendation**: Start training with current sensors. If the policy works in simulation but fails on the real robot, THEN add sensor noise/latency to close the sim-to-real gap.

## 🚀 Training Commands

### Recommended Training Workflow

1. **Quick sanity check** (1-2 minutes):
```bash
python examples/Q1/q1_train.py -v -B 4 --max_iterations 10
```
Watch the first environment. Robot should:
- Not collapse immediately
- Try to move forward
- Reset properly after falling

2. **Short training run** (5-10 minutes):
```bash
python examples/Q1/q1_train.py -B 512 --max_iterations 100
```
Check tensorboard logs:
```bash
tensorboard --logdir examples/Q1/logs
```
Look for:
- `tracking_lin_vel` reward increasing
- Episode length increasing over time
- Policy loss decreasing

3. **Full training** (1-2 hours):
```bash
python examples/Q1/q1_train.py -B 2048 --max_iterations 2000
```

4. **Evaluate trained policy**:
```bash
python examples/Q1/q1_eval.py -e q1-walking -c <iteration_number>
```

## ⚠️ Common Issues

### Robot collapses immediately
- **Cause**: PD gains too low or joint limits wrong
- **Fix**: Increase kp to 150-200, check joint limits in URDF
- **Test**: Run `q1_interactive_control.py` - if it stands there, training should work

### Robot moves but doesn't learn
- **Cause**: Reward scales or command velocity might be wrong
- **Fix**: Check that `tracking_lin_vel` reward is the dominant positive reward
- **Verify**: Plot rewards in tensorboard

### Training is very slow
- **Cause**: Too many environments or visualization enabled
- **Fix**: Start with `-B 512` without `-v`, increase envs if GPU has headroom

### Robot learns weird gaits
- **Cause**: Action scale too large or reward balance off
- **Fix**: Reduce `action_scale` from 0.25 to 0.15
- **Add**: `_reward_feet_air_time` to encourage proper stepping

## 📈 Expected Training Progress

| Iteration | Episode Length | Velocity Tracking | Notes |
|-----------|---------------|-------------------|-------|
| 0-50 | ~50 steps | Poor (~0.1) | Robot learning not to fall |
| 50-200 | ~200 steps | Improving (~0.4) | Basic forward motion emerging |
| 200-500 | ~400 steps | Good (~0.7) | Stable walking gait |
| 500+ | ~500 steps | Excellent (~0.9) | Optimizing efficiency |

## 🔧 Tuning Guide

If robot is learning but performance plateaus:

1. **For faster walking**: Increase `lin_vel_x_range` to [0.8, 0.8]
2. **For more stable gait**: Increase `action_rate` penalty to -0.01
3. **For smoother motion**: Add `_reward_dof_acc` with scale -0.0001
4. **For natural gait**: Add `_reward_feet_air_time` to encourage swing phase

## ✅ Pre-Training Verification

Before starting a long training run, verify:
- [ ] Interactive control works (`q1_interactive_control.py`)
- [ ] Visualization works (`q1_visualize.py`)  
- [ ] Robot doesn't collapse in first 10 iterations with `-v -B 4`
- [ ] Rewards are being computed (check terminal output)
- [ ] Tensorboard logs are being written to `logs/q1-walking/`

Good luck with training! 🚀
