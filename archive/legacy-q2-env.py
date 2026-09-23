import torch
import math
import genesis as gs
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat


def gs_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower


class Q1Env:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False):
        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]
        self.device = gs.device

        self.simulate_action_latency = True  # there is a 1 step latency on real robot
        self.dt = 0.02  # control frequency on real robot is 50hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

        # create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(1))),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
                # for this locomotion policy there are usually no more than 30 collision pairs
                # set a low value can save memory
                max_collision_pairs=30,
            ),
            show_viewer=show_viewer,
        )

        # add ground plane
        self.scene.add_entity(gs.morphs.URDF(file="genesis/assets/urdf/plane/plane.urdf", fixed=True))

        # add robot
        self.base_init_pos = torch.tensor(self.env_cfg["base_init_pos"], device=gs.device)
        self.base_init_quat = torch.tensor(self.env_cfg["base_init_quat"], device=gs.device)
        self.inv_base_init_quat = inv_quat(self.base_init_quat)
        
        # Sensor coordinate correction - rotate sensor readings to correct frame
        # Robot spawns visually upright with quat (0.707, -0.707, 0, 0)
        # But we apply +90Â° X rotation to sensor readings for correct IMU frame
        if "sensor_correction_quat" in self.env_cfg:
            self.sensor_correction_quat = torch.tensor(
                self.env_cfg["sensor_correction_quat"], device=gs.device
            )
            print(f"[Q1Env] Using sensor coordinate correction: {self.env_cfg['sensor_correction_quat']}")
        else:
            self.sensor_correction_quat = None

        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="genesis/assets/urdf/q1/kutta.urdf",
                pos=self.base_init_pos.cpu().numpy(),
                quat=self.base_init_quat.cpu().numpy(),
            ),
        )

        # build
        self.scene.build(n_envs=num_envs)

        # names to indices - properly handle DOF indices
        self.motors_dof_idx = []
        base_dof_count = 6  # floating base has 6 DOFs (3 for position, 3 for rotation)
        dof_offset = 6  # Start actuated joints after floating base DOFs
        
        # First, get all raw indices
        raw_indices = []
        for name in self.env_cfg["joint_names"]:
            joint = self.robot.get_joint(name)
            raw_indices.append(joint.dof_start)
        
        # Find the minimum index to calculate offset
        min_idx = min(raw_indices)
        
        # Map indices to proper range starting from dof_offset
        for i, name in enumerate(self.env_cfg["joint_names"]):
            # Map the raw index to start after base DOFs
            dof_idx = dof_offset + (raw_indices[i] - min_idx)
            self.motors_dof_idx.append(dof_idx)
            print(f"[Q1Env] Joint {name} mapped to DOF index {dof_idx} (raw index: {raw_indices[i]})")
            
        print("[Q1Env] Joint names:", self.env_cfg["joint_names"])
        print("[Q1Env] Remapped DOF indices:", self.motors_dof_idx)
        print("[Q1Env] DOF indices:", self.motors_dof_idx)

        # PD control parameters - using high gains for stable control
        # If kp/kd are None, use Genesis/URDF defaults (same as q1_visualize.py)
        if self.env_cfg["kp"] is not None and self.env_cfg["kd"] is not None:
            self.robot.set_dofs_kp([self.env_cfg["kp"]] * self.num_actions, self.motors_dof_idx)
            self.robot.set_dofs_kv([self.env_cfg["kd"]] * self.num_actions, self.motors_dof_idx)
            print(f"[Q1Env] Using custom PD gains: kp={self.env_cfg['kp']}, kd={self.env_cfg['kd']}")
        else:
            print(f"[Q1Env] Using Genesis default PD gains (same as q1_visualize.py)")
        
        print(f"[Q1Env] Initialized with {num_envs} environments")
        print(f"[Q1Env] Default joint positions: {[self.env_cfg['default_joint_angles'][name] for name in self.env_cfg['joint_names']]}")
        print(f"[Q1Env] Visualization: {'enabled (env 0 only)' if show_viewer else 'disabled'}")

        # prepare reward functions and multiply reward scales by dt
        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            # multiply in-place by dt so that rewards are per-step not per-second
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)

        # initialize buffers
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.projected_gravity = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], device=gs.device, dtype=gs.tc_float).repeat(
            self.num_envs, 1
        )
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=gs.device, dtype=gs.tc_float)
        self.rew_buf = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)
        self.reset_buf = torch.ones((self.num_envs,), device=gs.device, dtype=gs.tc_int)
        self.episode_length_buf = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_int)
        self.commands = torch.zeros((self.num_envs, self.num_commands), device=gs.device, dtype=gs.tc_float)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=gs.device,
            dtype=gs.tc_float,
        )
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=gs.device, dtype=gs.tc_float)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.zeros_like(self.actions)
        self.dof_vel = torch.zeros_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.base_pos = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.base_quat = torch.zeros((self.num_envs, 4), device=gs.device, dtype=gs.tc_float)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["joint_names"]],
            device=gs.device,
            dtype=gs.tc_float,
        )
        self.extras = dict()  # extra information for logging
        self.extras["observations"] = dict()

    def _resample_commands(self, envs_idx):
        self.commands[envs_idx, 0] = gs_rand_float(*self.command_cfg["lin_vel_x_range"], (len(envs_idx),), gs.device)
        self.commands[envs_idx, 1] = gs_rand_float(*self.command_cfg["lin_vel_y_range"], (len(envs_idx),), gs.device)
        self.commands[envs_idx, 2] = gs_rand_float(*self.command_cfg["ang_vel_range"], (len(envs_idx),), gs.device)

    def step(self, actions):
        from genesis.utils.geom import transform_quat_by_quat
        
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos
        self.robot.control_dofs_position(target_dof_pos, self.motors_dof_idx)
        self.scene.step()

        # update buffers
        self.episode_length_buf += 1
        self.base_pos[:] = self.robot.get_pos()
        self.base_quat[:] = self.robot.get_quat()
        
        # Compute euler angles from RAW quaternion (for termination checks)
        # Don't apply sensor correction here - we want to know the actual physical orientation
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(torch.ones_like(self.base_quat) * self.inv_base_init_quat, self.base_quat),
            rpy=True,
            degrees=True,
        )
        
        # Apply sensor coordinate correction for IMU/velocity readings
        if self.sensor_correction_quat is not None:
            # Correct the quaternion by applying sensor rotation
            corrected_quat = transform_quat_by_quat(
                self.base_quat,
                self.sensor_correction_quat.unsqueeze(0).expand(self.num_envs, -1)
            )
        else:
            corrected_quat = self.base_quat
        
        # Use corrected quaternion for velocity transformations and projected gravity
        inv_base_quat = inv_quat(corrected_quat)
        self.base_lin_vel[:] = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel[:] = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)
        self.dof_pos[:] = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel[:] = self.robot.get_dofs_velocity(self.motors_dof_idx)

        # resample commands
        envs_idx = (
            (self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)
            .nonzero(as_tuple=False)
            .reshape((-1,))
        )
        self._resample_commands(envs_idx)

        # check termination and reset
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]

        time_out_idx = (self.episode_length_buf > self.max_episode_length).nonzero(as_tuple=False).reshape((-1,))
        self.extras["time_outs"] = torch.zeros_like(self.reset_buf, device=gs.device, dtype=gs.tc_float)
        self.extras["time_outs"][time_out_idx] = 1.0

        self.reset_idx(self.reset_buf.nonzero(as_tuple=False).reshape((-1,)))

        # compute reward
        self.rew_buf[:] = 0.0
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # compute observations
        self.obs_buf = torch.cat(
            [
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 8
                # self.dof_vel * self.obs_scales["dof_vel"],  # 8
                self.actions,  # 8
            ],
            axis=-1,
        )

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]

        self.extras["observations"]["critic"] = self.obs_buf

        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.extras

    def get_privileged_observations(self):
        return None

    def reset_idx(self, envs_idx):
        if len(envs_idx) == 0:
            return

        # reset dofs
        self.dof_pos[envs_idx] = self.default_dof_pos
        self.dof_vel[envs_idx] = 0.0
        self.robot.set_dofs_position(
            position=self.dof_pos[envs_idx],
            dofs_idx_local=self.motors_dof_idx,
            zero_velocity=True,
            envs_idx=envs_idx,
        )

        # reset base
        self.base_pos[envs_idx] = self.base_init_pos
        self.base_quat[envs_idx] = self.base_init_quat.reshape(1, -1)
        self.robot.set_pos(self.base_pos[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.robot.set_quat(self.base_quat[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.base_lin_vel[envs_idx] = 0
        self.base_ang_vel[envs_idx] = 0
        self.robot.zero_all_dofs_velocity(envs_idx)

        # reset buffers
        self.last_actions[envs_idx] = 0.0
        self.last_dof_vel[envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = True

        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]["rew_" + key] = (
                torch.mean(self.episode_sums[key][envs_idx]).item() / self.env_cfg["episode_length_s"]
            )
            self.episode_sums[key][envs_idx] = 0.0

        self._resample_commands(envs_idx)

    def reset(self):
        self.reset_buf[:] = True
        self.reset_idx(torch.arange(self.num_envs, device=gs.device))
        return self.obs_buf, None

    # ------------ reward functions----------------
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        # Penalize joint poses far away from default pose
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    def _reward_base_height(self):
        # Penalize base height away from target
        return torch.square(self.base_pos[:, 2] - self.reward_cfg["base_height_target"])

    def _reward_upright(self):
        """
        Reward proportional to how upright the robot is.
        We use projected_gravity (gravity in body frame). For a perfectly upright robot,
        projected_gravity[:, 2] ~= -1.0. We return positive reward near 1.0 when upright.
        """
        # -projected_gravity_z is ~1 when upright
        up_score = -self.projected_gravity[:, 2]
        # clamp to [0, 1.0] to keep reward bounded
        up_score = torch.clamp(up_score, min=0.0, max=1.0)
        return up_score

    def _reward_tilt_penalty(self):
        """
        Penalize tilt: magnitude of gravity projection on the body x/y axes.
        This is near zero when upright and grows as robot tilts.
        """
        tilt_mag_sq = torch.sum(self.projected_gravity[:, :2] ** 2, dim=1)
        # optionally take sqrt to get magnitude, but squared is fine and differentiable
        return tilt_mag_sq

    def _reward_base_ang_vel(self):
        """
        Penalize high roll/pitch angular velocity (wobble). Ignore yaw (z).
        """
        return torch.sum(self.base_ang_vel[:, :2] ** 2, dim=1)

    # Optional commented-out reward helpers you might want later:
    # def _reward_ang_vel_xy(self):
    #     return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    # def _reward_orientation(self):
    #     return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    # def _reward_dof_vel(self):
    #     return torch.sum(torch.square(self.dof_vel), dim=1)

    # def _reward_dof_acc(self):
    #     return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    # def _reward_collision(self):
    #     return torch.zeros(self.num_envs, device=gs.device, dtype=gs.tc_float)