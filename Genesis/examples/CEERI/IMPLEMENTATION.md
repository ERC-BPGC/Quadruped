# Implementation Summary - Indoor Drone Exploration

## ✅ Completed Implementation

### Files Created:
1. **`exploration_env.py`** - Main environment class
2. **`exploration_train.py`** - Training script  
3. **`exploration_eval.py`** - Evaluation script
4. **`exploration_teleop.py`** - Manual control testing script
5. **`/home/ritwik/NUS/Genesis/meshes/maze_simple.xml`** - MJCF maze definition
6. **`README.md`** - Updated comprehensive documentation

---

## 🎯 Key Features Implemented

### 1. Observation Space (15D)
```python
obs = [
    F,              # Forward LiDAR distance (1D)
    R,              # Right LiDAR distance (1D)
    L,              # Left LiDAR distance (1D)
    lin_vel_x,      # Linear velocity in drone frame (3D)
    lin_vel_y,
    lin_vel_z,
    euler_roll,     # Euler angles (3D)
    euler_pitch,
    euler_yaw,
    ang_vel_x,      # Angular velocity in drone frame (3D)
    ang_vel_y,
    ang_vel_z,
    last_action_1,  # Previous actions (3D)
    last_action_2,
    last_action_3,
]
```

**LiDAR Processing:**
- 128 rays spanning 180° horizontal ([-90°, +90°])
- Forward sector: [-10°, +10°] → minimum distance = F
- Right sector: [+10°, +90°] → minimum distance = R
- Left sector: [-90°, -10°] → minimum distance = L
- Max range: 6.0m

### 2. Action Space (3D)
```python
actions = [
    x_vel,      # Forward/backward velocity [-1.5, 1.5] m/s
    y_vel,      # Left/right velocity [-1.5, 1.5] m/s  
    yaw_rate,   # Rotation rate [-π/2, π/2] rad/s
]
```

**Velocity Controller:**
- Simple proportional controller converts velocity commands to rotor RPMs
- Gains: `kp_vel=3000`, `kp_yaw=5000`
- Hover RPM: 14468
- RPM clipping: [5000, 25000]

### 3. Reward Structure (from paper)

**R1: Variable Pitch**
```python
if F > 1.5:
    reward = β * (F - 1.5)²  # Encourage forward movement
else:
    reward = -η * exp(1/(F - 1.5))  # Penalize proximity
```

**R2: Obstacle Avoidance**
```python
Urep_R = λ * (1/R - 1/d0)² if R ≤ d0
Urep_L = λ * (1/L - 1/d0)² if L ≤ d0
rep_eq = |Urep_R - Urep_L|

if rep_eq < κ:
    if Δrep_eq < 0:
        reward = 4κ / rep_eq  # Reward improvement
    else:
        reward = κ / rep_eq - 1  # Penalize worsening
else:
    reward = -ω * exp(rep_eq - ρ)  # Large penalty
```

**R3: Center Alignment**
```python
Uattr = -μ * (R - L)² + δ
if ΔUattr > 0:
    reward = 2 * Uattr  # Reward centering
else:
    reward = Uattr  # Maintain
```

**R4: Exploration** (placeholder, set to 0)

**Additional Rewards:**
- Collision: -10.0
- Smoothness: -0.01 * ||actions - last_actions||²
- Angular velocity: -0.001 * ||ang_vel||

### 4. Maze Environment

**MJCF Structure:**
- 10m × 10m arena with boundary walls
- Internal walls creating corridors
- T-junctions and turns
- Cylindrical pillars as obstacles
- Wall height: 2m
- Wall thickness: 0.2m

**Fallback:** If MJCF fails to load, simple cuboid walls are created programmatically.

---

## 🚀 Quick Start Commands

### Test with Teleoperation:
```bash
cd /home/ritwik/NUS/Genesis/examples/CEERI
python exploration_teleop.py
```

**Controls:**
- Arrow keys: Move forward/back/left/right
- A/D: Rotate left/right
- Space: Stop
- R: Reset
- ESC: Quit

### Train:
```bash
# Fast training (headless)
python exploration_train.py -e drone-exploration -B 4096 --max_iterations 5001

# With visualization (slow)
python exploration_train.py -e drone-exploration -B 256 --max_iterations 501 -v
```

### Evaluate:
```bash
python exploration_eval.py -e drone-exploration --ckpt 5000
python exploration_eval.py -e drone-exploration --ckpt 5000 --record  # with video
```

### Monitor Training:
```bash
tensorboard --logdir logs/
```

---

## 🔧 Configuration Parameters

### Environment Config (`env_cfg`):
```python
{
    "num_actions": 3,
    "episode_length_s": 60.0,
    "collision_threshold": 0.3,       # meters
    "termination_if_pitch_greater_than": 45,  # degrees
    "termination_if_roll_greater_than": 45,
    "termination_if_close_to_ground": 0.15,
    "termination_if_too_high": 2.5,
    "base_init_pos": [0.0, 0.0, 1.0],
    "randomize_init_pos": True,       # for training
    "max_x_vel": 1.5,                 # m/s
    "max_y_vel": 1.5,
    "max_yaw_rate": 1.57,            # rad/s
    "kp_vel": 3000.0,
    "kp_yaw": 5000.0,
    "lidar_n_rays": 128,
    "lidar_max_range": 6.0,
}
```

### Training Config (`train_cfg`):
```python
{
    "algorithm": {
        "class_name": "PPO",
        "learning_rate": 3e-4,
        "entropy_coef": 0.01,
        "num_mini_batches": 8,
        "num_learning_epochs": 5,
    },
    "policy": {
        "actor_hidden_dims": [256, 256, 128],
        "critic_hidden_dims": [256, 256, 128],
    },
    "num_steps_per_env": 200,
    "max_iterations": 5001,
}
```

### Reward Scales:
```python
{
    "variable_pitch": 1.0,
    "obstacle_avoidance": 5.0,
    "center_alignment": 2.0,
    "exploration": 0.0,           # placeholder
    "collision": 10.0,
    "smooth": -0.01,
    "angular": -0.001,
}
```

---

## 📊 Expected Training Behavior

### Early Training (0-500 iterations):
- Drone learns to hover without crashing
- Random exploration, frequent collisions
- Rewards mostly negative

### Mid Training (500-2000 iterations):
- Learns basic obstacle avoidance
- Starts following corridors
- Forward movement increases

### Late Training (2000-5000 iterations):
- Smooth corridor navigation
- Good center alignment
- Efficient exploration
- Collision rate decreases

### Convergence:
- Success rate > 80%
- Average episode length > 50s
- Smooth trajectories

---

## 🐛 Common Issues & Solutions

### Issue 1: "pynput not installed"
```bash
pip install pynput
```

### Issue 2: Teleop not responding
- Make sure terminal window has focus
- Some keys might conflict with VS Code shortcuts
- Try in standalone terminal

### Issue 3: Drone crashes immediately
- Check initial position not inside wall
- Verify maze loaded correctly
- Adjust controller gains if oscillating

### Issue 4: Training very slow
- Reduce number of environments: `-B 256`
- Disable visualization: remove `-v`
- Check GPU utilization: `nvidia-smi`

### Issue 5: LiDAR not visualizing
- Enable in config: `visualize_lidar=True`
- Make sure `show_viewer=True`
- Check Genesis viewer is running

---

## 🔬 Hyperparameter Tuning Guide

### If drone is too aggressive:
- Decrease `max_x_vel`, `max_y_vel`
- Decrease reward scale for `variable_pitch`
- Increase `collision` penalty

### If drone is too conservative:
- Increase `variable_pitch` reward scale
- Decrease `obstacle_avoidance` scale
- Increase `max_x_vel`

### If drone wobbles/oscillates:
- Decrease controller gains: `kp_vel`, `kp_yaw`
- Increase `smooth` penalty
- Increase `angular` penalty

### If learning is slow:
- Increase learning rate
- Increase number of environments
- Adjust reward scales to be more balanced

---

## 📈 Next Steps / Future Work

1. **Add entropy-based exploration reward**
   - Track visited states/grid cells
   - Reward novel states
   - Implement curiosity-driven exploration

2. **Implement multi-critic DDPG**
   - Separate critics for pitch, yaw, roll
   - Independent Q-value estimation
   - As described in paper

3. **Add precision playback buffer**
   - Store high-quality experiences
   - Priority sampling based on loss
   - Faster convergence

4. **3D LiDAR**
   - Add vertical rays for ceiling/floor detection
   - Full 3D obstacle avoidance
   - More complex environments

5. **Coverage metrics**
   - Track explored area
   - Visualize heatmap
   - Reward based on coverage

6. **More complex mazes**
   - Multi-level environments
   - Dynamic obstacles
   - Different maze topologies

7. **Sim-to-real transfer**
   - Domain randomization
   - Real Crazyflie deployment
   - Reality gap analysis

---

## 📚 Code Architecture

```
exploration_env.py
├── __init__()
│   ├── Scene setup
│   ├── Maze loading (MJCF or fallback)
│   ├── Drone entity
│   ├── LiDAR sensor attachment
│   └── Buffer initialization
├── _build_maze()
│   └── MJCF or cuboid walls
├── _process_lidar()
│   └── Extract F, R, L from raw distances
├── _velocity_to_rpm()
│   └── Proportional controller
├── step()
│   ├── Action → RPM conversion
│   ├── Physics step
│   ├── LiDAR reading
│   ├── Termination check
│   ├── Reward computation
│   └── Observation update
├── reset_idx()
│   └── Reset specific environments
└── Reward functions
    ├── _reward_variable_pitch()
    ├── _reward_obstacle_avoidance()
    ├── _reward_center_alignment()
    ├── _reward_exploration()
    ├── _reward_collision()
    ├── _reward_smooth()
    └── _reward_angular()
```

---

## 🎓 Paper Implementation Notes

**What's implemented from paper:**
- ✅ LiDAR-based state representation [F, R, L]
- ✅ 3D action space [pitch, yaw, roll] → [x_vel, y_vel, yaw_rate]
- ✅ Reward 1: Variable pitch control
- ✅ Reward 2: Obstacle avoidance (repulsive potential)
- ✅ Reward 3: Center alignment (attractive potential)
- ✅ Maze environment

**What's different:**
- ❌ Multi-critic DDPG → Single-critic PPO (for now)
- ❌ Precision playback buffer → Standard replay (for now)
- ❌ Exploration reward → Placeholder (set to 0)
- ➕ Added smoothness and angular velocity penalties
- ➕ Extended observations with velocities and orientations

**Why PPO instead of DDPG:**
- Simpler to implement and debug
- More stable training
- Good baseline before multi-critic
- Extensible architecture for future DDPG implementation

---

## ✅ Testing Checklist

Before training:
- [ ] Run `exploration_teleop.py` and verify:
  - [ ] Keyboard controls work
  - [ ] LiDAR rays visualize correctly
  - [ ] Drone responds to commands
  - [ ] Collisions detected properly
  - [ ] Environment resets correctly

For training:
- [ ] Start with small number of envs (256)
- [ ] Monitor first 100 iterations for stability
- [ ] Check TensorBoard for reward trends
- [ ] Verify checkpoints are saving
- [ ] No NaN or Inf values in logs

For evaluation:
- [ ] Load trained checkpoint successfully
- [ ] Policy runs without errors
- [ ] Drone navigates corridors
- [ ] Success rate > 50%
- [ ] Video recording works (if enabled)

---

## 📞 Support

If you encounter issues:
1. Check this document's troubleshooting section
2. Review the main README.md
3. Verify all dependencies are installed
4. Test with teleop script first
5. Check Genesis documentation

---

**Created:** October 20, 2025  
**Environment:** Genesis Physics Simulator  
**RL Framework:** rsl-rl (PPO)  
**Drone Model:** Crazyflie 2.X
