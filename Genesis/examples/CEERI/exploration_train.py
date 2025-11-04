import argparse
import os
import pickle
import shutil
from importlib import metadata

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
from rsl_rl.runners import OnPolicyRunner

import genesis as gs

from exploration_env import ExplorationEnv


def get_train_cfg(exp_name, max_iterations):
    train_cfg_dict = {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,  # Higher entropy for exploration
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 3e-4,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 8,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [256, 256, 128],
            "critic_hidden_dims": [256, 256, 128],
            "init_noise_std": 1.0,
            "class_name": "ActorCritic",
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
        },
        "runner_class_name": "OnPolicyRunner",
        "num_steps_per_env": 200,
        "save_interval": 100,
        "empirical_normalization": None,
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs():
    env_cfg = {
        "num_actions": 3,  # x_vel, y_vel, yaw_rate
        
        # Episode settings
        "episode_length_s": 60.0,
        
        # Termination conditions
        "termination_if_pitch_greater_than": 45,  # degrees
        "termination_if_roll_greater_than": 45,
        "termination_if_close_to_ground": 0.15,
        "termination_if_too_high": 2.5,
        "collision_threshold": 0.3,  # meters
        
        # Initial pose - base position in open area, randomized during training
        "base_init_pos": [-3.0, 3.0, 1.0],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "randomize_init_pos": True,  # Randomize start position for training
        
        # Control parameters
        "max_x_vel": 1.5,  # m/s
        "max_y_vel": 1.5,  # m/s
        "max_yaw_rate": 1.57,  # rad/s (~90 deg/s)
        "clip_actions": 1.0,
        
        # Controller gains
        "kp_vel": 3000.0,
        "kp_yaw": 5000.0,
        
        # LiDAR settings
        "lidar_n_rays": 128,
        "lidar_max_range": 6.0,
        
        # Visualization
        "visualize_lidar": False,  # Disable for training speed
        "visualize_camera": False,
        "max_visualize_FPS": 30,
    }
    
    obs_cfg = {
        "num_obs": 15,  # F(1) + R(1) + L(1) + lin_vel(3) + euler(3) + ang_vel(3) + last_actions(3)
        "obs_scales": {
            "lidar": 1.0 / 6.0,  # normalize by max range
            "lin_vel": 1.0 / 3.0,
            "euler": 1.0 / 180.0,  # degrees to normalized
            "ang_vel": 1.0 / 3.14159,
        },
    }
    
    reward_cfg = {
        "reward_scales": {
            "variable_pitch": 1.0,
            "obstacle_avoidance": 5.0,
            "center_alignment": 2.0,
            "exploration": 0.0,  # Placeholder for future entropy-based reward
            "collision": 10.0,
            "smooth": -0.01,
            "angular": -0.001,
        },
    }

    return env_cfg, obs_cfg, reward_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="drone-exploration")
    parser.add_argument("-v", "--vis", action="store_true", default=False)
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=5001)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    if args.vis:
        env_cfg["visualize_lidar"] = True
        env_cfg["max_visualize_FPS"] = 30

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    env = ExplorationEnv(
        num_envs=args.num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        show_viewer=args.vis,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# Training commands:

# Basic training (headless, fast)
python exploration_train.py -e drone-exploration -B 4096 --max_iterations 5001

# Training with visualization (slower)
python exploration_train.py -e drone-exploration -B 4096 --max_iterations 5001 -v

# Quick test with fewer environments
python exploration_train.py -e test-exploration -B 256 --max_iterations 501

# Monitor training with TensorBoard:
tensorboard --logdir logs/
"""
