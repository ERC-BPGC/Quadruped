# Quadruped Robot Learning Project

This project implements reinforcement learning for quadrupedal locomotion using the Genesis physics simulation framework. It focuses on training a quadruped robot to perform various locomotion tasks including walking and turning maneuvers.

## Project Overview

The project consists of two main implementations:

### Q1: Forward Walking
- Basic walking gait implementation
- Focus on stable forward locomotion
- Conservative action parameters and reward structure
- Balanced emphasis on stability and forward progress

### Q2: Turning Maneuvers
- Enhanced turning capabilities
- Increased action scaling (0.35 vs 0.25)
- Modified reward structure prioritizing rotation
- Aggressive angular velocity targets (1.2 rad/s)
- Relaxed stability constraints to allow dynamic turning

## Key Features

- Reinforcement learning using PPO (Proximal Policy Optimization)
- Genesis physics engine integration for accurate simulation
- Real-time visualization capabilities
- Multi-environment parallel training
- Configurable reward structures
- Robust joint control using PD controllers
- Advanced sensor frame corrections for accurate IMU readings

## Technical Specifications

### Robot Configuration
- 8 actuated joints (2 per leg)
  - FL/FR/RL/RR thigh joints
  - FL/FR/RL/RR calf joints
- Default stance angles optimized for stability
- High-performance PD control (kp=100.0, kd=2.5)

### Observation Space
25-dimensional state space including:
- Base angular velocity (3D)
- Projected gravity vector (3D)
- Command signals (3D)
- Joint positions relative to default (8D)
- Last actions (8D)

### Action Space
- 8-dimensional continuous actions
- Scaled outputs (0.25 for walking, 0.35 for turning)
- Action clipping at ±100.0

## Usage

### Training

```bash
# Q1: Train walking gait
python examples/Q1/q1_train.py -v  # With visualization
python examples/Q1/q1_train.py -B 4096  # Full training

# Q2: Train turning maneuvers
python examples/Q2/q2_turn_train.py -v  # With visualization
python examples/Q2/q2_turn_train.py -B 4096  # Full training

# Debug with fewer environments
python examples/Q1/q1_train.py -v -B 8
python examples/Q2/q2_turn_train.py -v -B 8
```

### Evaluation

```bash
# Evaluate walking
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500

# Evaluate turning
python examples/Q2/q2_eval.py -e q2-turn-left --ckpt 500
```

## Training Parameters

### Environment
- Episode duration: 20 seconds
- Control frequency: 50 Hz
- Command resampling: Every 4 seconds
- Termination conditions:
  - Roll > 15 degrees
  - Pitch > 15 degrees

### PPO Configuration
- Learning rate: 0.001
- Clip parameter: 0.2
- Value loss coefficient: 1.0
- Mini-batch size: 4
- Training epochs: 5
- Discount factor (γ): 0.99
- GAE parameter (λ): 0.95

### Neural Network
- Actor: [512, 256, 128] hidden layers
- Critic: [512, 256, 128] hidden layers
- ELU activation function

## Reward Structure

### Q1 (Walking)
- Tracking linear velocity (2.0)
- Tracking angular velocity (0.5)
- Base height maintenance (-50.0)
- Action smoothness (-0.05)
- Pose similarity (-0.2)

### Q2 (Turning)
- Enhanced turning rewards (12.0)
- Reduced stability penalties
- Relaxed pose constraints
-允许更激进的运动范围

## Requirements

- Python ≥ 3.10, < 3.14
- genesis-world
- PyTorch (latest stable)
- rsl-rl-lib==2.2.4

## Project Structure

```
📦 Quadruped
├── Genesis/
│   ├── examples/
│   │   ├── Q1/                # Walking implementation
│   │   │   ├── q1_env.py     # Environment definition
│   │   │   ├── q1_eval.py    # Evaluation script
│   │   │   └── q1_train.py   # Training script
│   │   └── Q2/               # Turning implementation
│   │       ├── q2_env.py     # Enhanced environment
│   │       ├── q2_eval.py    # Turn evaluation
│   │       └── q2_turn_train.py  # Turn training
│   └── ...
└── logs/                     # Training logs & models
    ├── q1-walking/          # Walking checkpoints
    └── q2-turn-left/        # Turning checkpoints
```

## Advanced Features

### Sensor Frame Correction
- Quaternion-based IMU frame alignment
- Proper handling of base orientation
- Accurate velocity transformations

### DOF Management
- Robust joint-to-DOF mapping
- Proper floating base handling
- Automatic index correction for URDF compatibility

## License

This project builds upon the Genesis framework which is licensed under the Apache 2.0 License. See the LICENSE file in the Genesis directory for more details.

## Acknowledgments

- Genesis physics engine for simulation
- rsl-rl-lib for reinforcement learning
- PyTorch for deep learning framework Robot Learning Project

This project implements reinforcement learning for quadrupedal locomotion using the Genesis physics simulation framework. It focuses on training a quadruped robot (Q1) to perform stable walking gaits through PPO (Proximal Policy Optimization) algorithm.

## Project Structure

```
📦 Quadruped
├── Genesis/                      # Genesis framework integration
│   ├── examples/
│   │   └── Q1/                  # Q1 robot specific implementation
│   │       ├── q1_env.py        # Environment implementation
│   │       ├── q1_eval.py       # Evaluation script
│   │       └── q1_train.py      # Training script
│   └── ...                      # Other Genesis framework files
└── logs/
    └── q1-walking/              # Training logs and checkpoints
```

## Features

- Reinforcement learning implementation for quadrupedal locomotion
- Built on the Genesis physics engine for accurate robot simulation
- PPO-based training with customizable hyperparameters
- Support for both training and evaluation workflows
- Configurable robot parameters and reward functions
- Real-time visualization options

## Key Components

### Q1 Environment (`q1_env.py`)

- Custom environment implementation for the Q1 quadruped robot
- PD control for joint actuation with configurable gains
- Comprehensive state observation and reward computation
- Support for multiple parallel environments
- Sensor coordinate frame corrections for accurate IMU readings

### Training (`q1_train.py`)

- PPO implementation with configurable neural network architecture
- Multi-environment parallel training
- Customizable training parameters
- Checkpoint saving and loading
- Optional real-time visualization

### Evaluation (`q1_eval.py`)

- Model evaluation with saved checkpoints
- Action recording and analysis
- Visualization support for qualitative assessment

## Robot Configuration

### Joint Configuration
- 8 actuated joints (2 per leg: thigh and calf)
- Default joint angles optimized for stable standing
- High-gain PD control (kp=100.0, kd=2.5) for precise position control

### Observation Space
- 25-dimensional observation space including:
  - Base angular velocity (3D)
  - Projected gravity vector (3D)
  - Command signals (3D)
  - Joint positions relative to default (8D)
  - Last actions (8D)

### Action Space
- 8-dimensional continuous action space
- Action scaling of 0.25
- Action clipping at ±100.0

## Usage

### Training

```bash
# Basic training with default parameters
python examples/Q1/q1_train.py

# Training with visualization
python examples/Q1/q1_train.py -v

# Training with custom number of environments
python examples/Q1/q1_train.py -v -B 8  # for debugging with 8 environments
```

### Evaluation

```bash
# Evaluate a trained model (checkpoint 500)
python examples/Q1/q1_eval.py -e q1-walking --ckpt 500
```

## Requirements

- Python ≥ 3.10, < 3.14
- PyTorch (latest stable version)
- genesis-world
- rsl-rl-lib==2.2.4

## Training Parameters

### Environment
- Episode length: 20 seconds
- Control frequency: 50 Hz (dt = 0.02s)
- Command resampling every 4 seconds
- Termination conditions:
  - Roll > 15 degrees
  - Pitch > 15 degrees

### PPO Configuration
- Learning rate: 0.001
- Clip parameter: 0.2
- Entropy coefficient: 0.01
- Value loss coefficient: 1.0
- Number of epochs: 5
- Mini-batch size: 4
- Gamma (discount factor): 0.99
- Lambda: 0.95

### Neural Network Architecture
- Actor network: [512, 256, 128] hidden layers
- Critic network: [512, 256, 128] hidden layers
- ELU activation function

## Rewards

The reward function combines multiple components:

- Tracking linear velocity (xy plane)
- Tracking angular velocity (yaw)
- Penalize vertical motion
- Base height maintenance
- Action smoothness
- Pose similarity to default stance

## Directory Structure

The project follows a modular structure:

```
Genesis/
├── examples/Q1/           # Core robot implementation
├── genesis/              # Genesis framework core
├── tests/               # Test suite
└── meshes/              # Robot and environment meshes
```

## License

This project builds upon the Genesis framework which is licensed under the Apache 2.0 License. See the LICENSE file in the Genesis directory for more details.

## Acknowledgments

This project utilizes several key components and builds upon previous work:

- Genesis physics engine for simulation
- rsl-rl-lib for reinforcement learning implementation
- PyTorch for deep learning framework