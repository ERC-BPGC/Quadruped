# Indoor Drone Exploration with LiDAR

This directory contains a custom implementation of an autonomous indoor drone exploration system using reinforcement learning (PPO) with LiDAR-based navigation.

## 📋 Overview

This project implements a drone navigation system for indoor maze exploration based on the research paper:
**"Exploring Unknown Indoors with Drone using Multi-Critic DDPG Architecture"** (Kishore et al.)

The implementation uses:
- **2D LiDAR sensor** for obstacle detection (180° horizontal coverage)
- **Velocity-based control** (x_vel, y_vel, yaw_rate commands)
- **Hybrid reward structure** from the paper (variable pitch, obstacle avoidance, center alignment)
- **PPO algorithm** for training (single-critic, extensible to multi-critic later)
- **MJCF maze environment** with corridors, turns, and obstacles

## 🚀 Quick Start

### 0. Installation

Install required dependencies:

```bash
pip install --upgrade pip
pip install tensorboard rsl-rl-lib==2.2.4 pynput
```

### 1. Test Environment with Teleoperation

Before training, test the environment and visualize LiDAR sensor:

```bash
python exploration_teleop.py
```

**Keyboard Controls:**
- `↑/↓` : Forward/Backward
- `←/→` : Left/Right (lateral movement)
- `A/D` : Rotate Left/Right (yaw)
- `Space` : Stop (zero velocity)
- `R` : Reset drone
- `ESC` : Quit

The LiDAR rays will be visualized:
- **Green rays**: No obstacle detected
- **Red rays**: Obstacle hit

### 2. Train the Policy

Train the exploration policy using PPO:

```bash
# Basic training (4096 parallel environments)
python exploration_train.py -e drone-exploration -B 4096 --max_iterations 5001

# Training with visualization (slower but useful for debugging)
python exploration_train.py -e drone-exploration -B 4096 --max_iterations 5001 -v

# Quick test with fewer environments
python exploration_train.py -e test-exploration -B 256 --max_iterations 501
```

Monitor training progress with TensorBoard:
```bash
tensorboard --logdir logs/
```

### 3. Evaluate the Trained Policy

Evaluate the trained policy:

```bash
# Basic evaluation
python exploration_eval.py -e drone-exploration --ckpt 5000

# Evaluation with video recording
python exploration_eval.py -e drone-exploration --ckpt 5000 --record

# Evaluate multiple episodes
python exploration_eval.py -e drone-exploration --ckpt 5000 --num_episodes 20
```

## 📁 File Structure

```
CEERI/
├── exploration_env.py          # Main environment with LiDAR sensor
├── exploration_train.py        # Training script (PPO)
├── exploration_eval.py         # Evaluation script
├── exploration_teleop.py       # Manual control for testing
├── README.md                   # This file
└── logs/                       # Training logs and checkpoints
    └── drone-exploration/
        ├── model_*.pt          # Saved model checkpoints
        ├── cfgs.pkl            # Configuration file
        └── events.*            # TensorBoard logs
```

## 🎯 Environment Details

### Observation Space (15D)
- **LiDAR readings (3D)**: [F, R, L]
  - F: Minimum distance forward ([-10°, +10°])
  - R: Minimum distance right ([+10°, +90°])
  - L: Minimum distance left ([-90°, -10°])
- **Linear velocity (3D)**: [vx, vy, vz] in drone frame
- **Euler angles (3D)**: [roll, pitch, yaw]
- **Angular velocity (3D)**: [wx, wy, wz] in drone frame
- **Last actions (3D)**: Previous [x_vel, y_vel, yaw_rate] commands

### Action Space (3D)
- **x_vel**: Forward/backward velocity [-1.5, 1.5] m/s
- **y_vel**: Left/right velocity [-1.5, 1.5] m/s
- **yaw_rate**: Rotation rate [-π/2, π/2] rad/s

Actions are converted to rotor RPMs using a simple proportional controller.

### Reward Structure

Based on the research paper with 4 components:

1. **Variable Pitch** (forward movement encouragement)
   - Rewards faster movement when path is clear
   - Penalizes getting too close to obstacles

2. **Obstacle Avoidance** (repulsive potential)
   - Uses artificial potential hills around obstacles
   - Rewards decreasing repulsion equilibrium

3. **Center Alignment** (attractive potential)
   - Keeps drone centered in corridors
   - Minimizes difference between left/right distances

4. **Exploration** (placeholder)
   - Currently set to zero
   - Reserved for entropy-based exploration reward

Additional rewards:
- Collision penalty
- Smoothness penalty (action jerkiness)
- Angular velocity penalty

### Environment Configuration

Key parameters (see `exploration_env.py` and `exploration_train.py`):

```python
env_cfg = {
    "num_actions": 3,
    "episode_length_s": 60.0,
    "collision_threshold": 0.3,  # meters
    "lidar_n_rays": 128,
    "lidar_max_range": 6.0,  # meters
    "max_x_vel": 1.5,  # m/s
    "max_y_vel": 1.5,  # m/s
    "max_yaw_rate": 1.57,  # rad/s
}
```

## 🏗️ Maze Environment

The maze is defined in `/home/ritwik/NUS/Genesis/meshes/maze_simple.xml` (MJCF format).

Current maze features:
- 10m × 10m arena
- Corridors with varying widths
- Internal walls creating T-junctions
- Cylindrical obstacles/pillars

You can modify the maze by editing the XML file to add:
- More complex corridors
- Different obstacle shapes
- Dead ends and loops
- Multiple floors (for future 3D navigation)

## 🔧 Technical Details

### LiDAR Sensor
- **Type**: 2D (single horizontal plane)
- **FOV**: 180° horizontal ([-90°, +90°])
- **Resolution**: 128 rays (configurable)
- **Range**: 0.1m - 6.0m
- **Update rate**: 100 Hz (same as simulation)

### Velocity Controller
Simple proportional controller converts velocity commands to RPM:
- `kp_vel = 3000.0` for translational velocities
- `kp_yaw = 5000.0` for yaw rate
- Hover RPM: 14468
- RPM range: [5000, 25000]

### Drone Model
- Crazyflie 2.X (`urdf/drones/cf2x.urdf`)
- Quadcopter X-configuration
- Mass: ~0.027 kg
- 4 rotors with differential control

## 📊 Training Tips

1. **Start with fewer environments** (256-512) to verify setup
2. **Monitor TensorBoard** for reward trends and policy metrics
3. **Adjust reward scales** if one component dominates
4. **Use visualization** (`-v` flag) sparingly during training (slows down)
5. **Save checkpoints** regularly (default: every 100 iterations)

Typical training time:
- 256 envs: ~5-10 minutes per 100 iterations (CPU)
- 4096 envs: ~10-20 minutes per 100 iterations (GPU)

## 🎓 Future Extensions

- [ ] Add entropy-based exploration reward
- [ ] Implement multi-critic DDPG architecture
- [ ] Add precision playback buffer
- [ ] 3D LiDAR for vertical obstacle avoidance
- [ ] More complex maze environments
- [ ] Coverage tracking and visualization
- [ ] Real-world transfer (sim-to-real)

## 📚 References

**Paper**: "Exploring Unknown Indoors with Drone using Multi-Critic DDPG Architecture"  
**Authors**: Kaushal Kishore, Chimata Anudeep, Shahid Shaikh, Samarth Singh

Key concepts adapted:
- Hybrid reward structure (repulsive/attractive potentials)
- LiDAR-based state representation
- Multi-dimensional action space

## 🐛 Troubleshooting

**Issue**: Teleop script not responding to keyboard
- **Solution**: Install pynput: `pip install pynput`

**Issue**: MJCF maze not loading
- **Solution**: Check if `meshes/maze_simple.xml` exists. If not, the environment will create simple walls as fallback.

**Issue**: Training very slow
- **Solution**: Disable visualization (`-v` flag), reduce number of environments, or use GPU backend

**Issue**: Drone crashes immediately
- **Solution**: Check controller gains (`kp_vel`, `kp_yaw`), adjust collision threshold, or verify initial position is not inside a wall

## 📝 License

This project is part of the Genesis physics simulation framework research.