import argparse
import os
import pickle
import shutil
from importlib import metadata
import math

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

from q1_env import Q1Env

def get_train_cfg(exp_name, max_iterations):
    train_cfg_dict = {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
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
        "num_steps_per_env": 24,
        "save_interval": 100,
        "empirical_normalization": None,
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs():
    env_cfg = {
        "num_actions": 8,  # Q1 has 8 joints (2 per leg: thigh and calf)
        # joint/link names
        "default_joint_angles": {  # [rad]
            "FL_thigh_joint": 0.45,
            "FL_calf_joint": 0.15,
            "FR_thigh_joint": 0.45,
            "FR_calf_joint": 0.15,
            "RL_thigh_joint": 0.25,
            "RL_calf_joint": 0.15,
            "RR_thigh_joint": 0.25,
            "RR_calf_joint": 0.15,
        },
        "joint_names": [
            "FL_thigh_joint",
            "FL_calf_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ],
        # PD - Using Go2's proven gains for better stability
        "kp": 100.0,
        "kd": 2.5,
        # termination
        "termination_if_roll_greater_than": 15,  # degree - slightly more tolerant
        "termination_if_pitch_greater_than": 15,
        # base pose
        "base_init_pos": [0.0, 0.0, 0.55],  # Adjusted based on your robot's height
        "base_init_quat": [math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0],  # Visual upright (0.707, -0.707, 0, 0)
        # Sensor coordinate correction - rotate sensor frame to get correct IMU readings
        "sensor_correction_quat": [math.cos(math.pi/4), math.sin(math.pi/4), 0, 0],  # +90° around X-axis
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,  # Slightly larger action scale for more movement range
        "simulate_action_latency": True,
        "clip_actions": 100.0,
    }
    obs_cfg = {
        "num_obs": 25,  # 3 + 3 + 3 + 8 + 8 + 8 = 33 (instead of 45 for Go2)
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.55,  # Target base height when standing (not spawn height!)
        "feet_height_target": 0.125,
        "reward_scales": {
            "tracking_lin_vel": 4.0,  # Emphasize forward movement
            "tracking_ang_vel": 1.5,
            "lin_vel_z": -0.5,  # Penalize vertical movement
            "base_height": -50.0,
            "action_rate": -0.05,  # Encourage smooth actions
            "similar_to_default": -0.2,  # Encourage staying near default pose
            "upright": 5.0,
            "tilt_penalty": -2.0,
            "base_ang_vel": -0.2,
        },
    }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0, 0],  # Forward walking at 0.5 m/s
        "lin_vel_y_range": [0.65, 0.65],  # No lateral movement
        "ang_vel_range": [0, 0],  # No turning
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="q1-walking")
    parser.add_argument("-v", "--vis", action="store_true", default=False)
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=201)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    env = Q1Env(
        num_envs=args.num_envs, 
        env_cfg=env_cfg, 
        obs_cfg=obs_cfg, 
        reward_cfg=reward_cfg, 
        command_cfg=command_cfg,
        show_viewer=args.vis,  # Add visualization support
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# training without visualization
python examples/Q1/q1_train.py

# training with visualization (shows first environment only)
python examples/Q1/q1_train.py -v

# training with fewer environments for debugging
python examples/Q1/q1_train.py -v -B 8
"""