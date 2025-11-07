# Q1 Quick Start Guide

## Prerequisites

1. Genesis simulator installed
2. `rsl-rl-lib==2.2.4` installed (not `rsl_rl`)
   ```bash
   pip uninstall rsl_rl  # if installed
   pip install rsl-rl-lib==2.2.4
   ```

## 5-Minute Quick Start

### 1. Visualize Robot (30 seconds)
```bash
cd /home/ritwik/NUS/Genesis
python examples/Q1/q1_visualize.py
```
✓ Verify robot loads and stands correctly

### 2. Quick Training Test (2 minutes)
```bash
python examples/Q1/q1_train.py -e q1-test -B 1024 --max_iterations 50
```
✓ Verify training starts without errors

### 3. Full Training (1-2 hours depending on GPU)
```bash
python examples/Q1/q1_train.py -e q1-walking -B 4096 --max_iterations 500
```
✓ Let it train to completion

### 4. Evaluate Trained Policy (1 minute)
```bash
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
```
✓ Watch your robot walk!

## Complete Workflow

```bash
# Terminal 1: Visualize
python examples/Q1/q1_visualize.py

# Terminal 1: Train
python examples/Q1/q1_train.py

# Terminal 2 (optional): Monitor
python examples/Q1/q1_monitor.py

# Terminal 3 (optional): TensorBoard
tensorboard --logdir examples/Q1/logs/q1-walking

# Terminal 1: Evaluate (after training)
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
```

## Expected Results

After 500 iterations:
- Robot should maintain stable standing posture
- Forward velocity should reach ~0.4-0.5 m/s
- Minimal roll/pitch oscillations
- Smooth joint movements

## Common Issues

### Issue: Robot falls immediately
**Solution**: Adjust default joint angles in `q1_train.py`
```python
"FL_thigh_joint": 0.6,   # Increase from 0.5
"FL_calf_joint": -0.4,   # Increase magnitude from -0.3
```

### Issue: Robot doesn't move forward
**Solution**: Increase forward velocity reward
```python
"tracking_lin_vel": 2.0,  # Increase from 1.5
```

### Issue: ImportError about rsl_rl
**Solution**: 
```bash
pip uninstall rsl_rl
pip install rsl-rl-lib==2.2.4
```

### Issue: Can't find XML file
**Solution**: Make sure you're in the Genesis root directory and the path is correct:
```
genesis/assets/xml/q1/q1_mjx_full.xml
```

## Parameters to Tune

Priority order for tuning:

1. **Default Joint Angles** - Most important for initial stability
2. **Base Height** - Adjust `base_init_pos[2]` and `base_height_target`
3. **Reward Weights** - Balance between tracking and stability
4. **PD Gains** - `kp` and `kd` for joint responsiveness
5. **Action Scale** - Range of joint movements

## File Reference

- `q1_env.py` - Environment (change robot behavior here)
- `q1_train.py` - Configuration (change parameters here)
- `q1_eval.py` - Evaluation (visualize results)
- `q1_visualize.py` - Standalone visualization
- `q1_monitor.py` - Training monitor

For detailed information, see:
- `README.md` - Complete documentation
- `MODIFICATIONS.md` - Changes from Go2 code

## Tips for Success

1. **Start small**: Use fewer environments (1024-2048) for quick tests
2. **Visualize first**: Always check robot loads correctly before training
3. **Monitor progress**: Use TensorBoard or monitor script
4. **Save checkpoints**: Training saves every 100 iterations
5. **Iterate**: Don't expect perfect results first try - tune and retrain

## Hardware Requirements

- **GPU**: Recommended for parallel simulation (4096 environments)
- **CPU**: Can work with fewer environments (512-1024)
- **RAM**: ~8GB minimum, 16GB recommended
- **Storage**: ~500MB for logs and checkpoints

## Expected Training Time

| Environments | Iterations | GPU (RTX 3090) | CPU (16 cores) |
|--------------|-----------|----------------|----------------|
| 1024         | 500       | ~30 min        | ~2 hours       |
| 4096         | 500       | ~1 hour        | ~8 hours       |

## Next Steps After Success

1. **Add turning**: Set `ang_vel_range: [-0.5, 0.5]`
2. **Add lateral movement**: Set `lin_vel_y_range: [-0.3, 0.3]`
3. **Variable speed**: Set `lin_vel_x_range: [0.0, 1.0]`
4. **Rough terrain**: Modify plane to heightfield
5. **Energy efficiency**: Add torque/power penalty rewards

Happy training! 🐕🤖
