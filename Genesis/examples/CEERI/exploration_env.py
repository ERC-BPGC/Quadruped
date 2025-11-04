import torch
import math
import copy
import numpy as np
import genesis as gs
from genesis.utils.geom import (
    quat_to_xyz,
    transform_by_quat,
    inv_quat,
    transform_quat_by_quat,
)


def gs_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower


class ExplorationEnv:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, show_viewer=False):
        self.num_envs = num_envs
        self.rendered_env_num = min(10, self.num_envs)
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.device = gs.device

        self.dt = 0.01  # run in 100hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = copy.deepcopy(reward_cfg["reward_scales"])

        # LiDAR configuration
        self.lidar_n_rays = env_cfg.get("lidar_n_rays", 128)
        self.lidar_max_range = env_cfg.get("lidar_max_range", 6.0)
        
        # Action space: [x_vel, y_vel, yaw_rate]
        self.max_x_vel = env_cfg.get("max_x_vel", 1.5)  # m/s
        self.max_y_vel = env_cfg.get("max_y_vel", 1.5)  # m/s
        self.max_yaw_rate = env_cfg.get("max_yaw_rate", 1.57)  # rad/s (~90 deg/s)
        
        # Velocity control gains (simple proportional controller)
        self.kp_vel = env_cfg.get("kp_vel", 3000.0)
        self.kp_yaw = env_cfg.get("kp_yaw", 5000.0)
        self.hover_rpm = 14468.429183500699

        # create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=env_cfg["max_visualize_FPS"],
                camera_pos=(3.0, -3.0, 3.0),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=50,
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(self.rendered_env_num))),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
            ),
            show_viewer=show_viewer,
        )

        # add plane
        self.scene.add_entity(gs.morphs.Plane())

        # Load maze environment
        self._build_maze()

        # add drone
        self.base_init_pos = torch.tensor(env_cfg["base_init_pos"], device=gs.device)
        self.base_init_quat = torch.tensor(env_cfg["base_init_quat"], device=gs.device)
        self.inv_base_init_quat = inv_quat(self.base_init_quat)
        self.drone = self.scene.add_entity(gs.morphs.Drone(file="urdf/drones/cf2x.urdf"))

        # Add 2D LiDAR sensor to drone
        # Use SphericalPattern with only horizontal scan (single vertical line)
        self.lidar_sensor = self.scene.add_sensor(
            gs.sensors.Lidar(
                pattern=gs.sensors.raycaster.SphericalPattern(
                    fov=(180.0, 1.0),  # 180° horizontal, 1° vertical (essentially 2D)
                    n_points=(self.lidar_n_rays, 1),  # n_rays horizontal, 1 vertical
                ),
                entity_idx=self.drone.idx,
                pos_offset=(0.0, 0.0, 0.0),  # at drone center
                euler_offset=(0.0, 0.0, 0.0),
                max_range=self.lidar_max_range,
                min_range=0.15,  # Set to 15cm to avoid detecting the drone itself (~6cm radius)
                return_world_frame=False,  # in drone frame
                draw_debug=env_cfg.get("visualize_lidar", False),
            )
        )

        # build scene
        self.scene.build(n_envs=num_envs)

        # prepare reward functions and multiply reward scales by dt
        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)

        # initialize buffers
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=gs.device, dtype=gs.tc_float)
        self.rew_buf = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)
        self.reset_buf = torch.ones((self.num_envs,), device=gs.device, dtype=gs.tc_int)
        self.episode_length_buf = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_int)

        self.actions = torch.zeros((self.num_envs, self.num_actions), device=gs.device, dtype=gs.tc_float)
        self.last_actions = torch.zeros_like(self.actions)

        self.base_pos = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.base_quat = torch.zeros((self.num_envs, 4), device=gs.device, dtype=gs.tc_float)
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=gs.device, dtype=gs.tc_float)
        
        # LiDAR processed data buffers
        self.lidar_data = torch.zeros((self.num_envs, self.lidar_n_rays), device=gs.device, dtype=gs.tc_float)
        self.F = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)  # Forward
        self.R = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)  # Right
        self.L = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)  # Left
        
        # Reward computation buffers (for temporal rewards)
        self.rep_eq_last = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)
        self.Uattr_last = torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)

        self.extras = dict()  # extra information for logging
        self.extras["observations"] = dict()

    def _build_maze(self):
        """Build a simple maze environment using cuboids."""
        # Try to load MJCF maze if it exists, otherwise create simple walls
        try:
            self.maze = self.scene.add_entity(
                gs.morphs.MJCF(
                    file="meshes/maze_simple.xml",
                    pos=(0, 0, 0),
                )
            )
        except:
            # Fallback: Create simple corridor maze with cuboids
            wall_height = 2.0
            wall_thickness = 0.2
            
            # Outer walls (10x10 arena)
            arena_size = 10.0
            
            # North wall
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(arena_size, wall_thickness, wall_height),
                    pos=(0.0, arena_size/2, wall_height/2),
                    fixed=True,
                )
            )
            # South wall
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(arena_size, wall_thickness, wall_height),
                    pos=(0.0, -arena_size/2, wall_height/2),
                    fixed=True,
                )
            )
            # East wall
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(wall_thickness, arena_size, wall_height),
                    pos=(arena_size/2, 0.0, wall_height/2),
                    fixed=True,
                )
            )
            # West wall
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(wall_thickness, arena_size, wall_height),
                    pos=(-arena_size/2, 0.0, wall_height/2),
                    fixed=True,
                )
            )
            
            # Internal walls to create corridors
            # Vertical divider
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(wall_thickness, 4.0, wall_height),
                    pos=(0.0, 0.0, wall_height/2),
                    fixed=True,
                )
            )
            
            # Horizontal wall segments
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(3.0, wall_thickness, wall_height),
                    pos=(-2.5, 2.0, wall_height/2),
                    fixed=True,
                )
            )
            self.scene.add_entity(
                gs.morphs.Box(
                    size=(3.0, wall_thickness, wall_height),
                    pos=(2.5, -2.0, wall_height/2),
                    fixed=True,
                )
            )

    def _process_lidar(self, raw_distances):
        """
        Process raw LiDAR distances to extract F, R, L values.
        
        LiDAR spans [-90°, 90°] (180° total)
        - Forward (F): [-10°, 10°] → middle ~1/9 of rays
        - Right (R): [10°, 90°] → right ~4/9 of rays
        - Left (L): [-90°, -10°] → left ~4/9 of rays
        """
        n_rays = raw_distances.shape[-1]
        
        # Indices for each sector (assuming rays go from left -90° to right +90°)
        # Index 0 = -90°, Index n_rays-1 = +90°
        forward_start = int(n_rays * 4/9)  # ~-10°
        forward_end = int(n_rays * 5/9)    # ~+10°
        
        # Extract minimum distances in each sector
        if len(raw_distances.shape) == 2:  # (n_envs, n_rays)
            F = torch.min(raw_distances[:, forward_start:forward_end], dim=1)[0]
            R = torch.min(raw_distances[:, forward_end:], dim=1)[0]
            L = torch.min(raw_distances[:, :forward_start], dim=1)[0]
        else:  # single env case (n_rays,)
            F = torch.min(raw_distances[forward_start:forward_end])
            R = torch.min(raw_distances[forward_end:])
            L = torch.min(raw_distances[:forward_start])
        
        return F, R, L

    def _update_state_and_observations(self):
        """Update drone state, LiDAR readings, and compute observations."""
        # Update drone state
        self.base_pos[:] = self.drone.get_pos()
        self.base_quat[:] = self.drone.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(
                torch.ones_like(self.base_quat) * self.inv_base_init_quat,
                self.base_quat,
            ),
            rpy=True,
            degrees=True,
        )
        inv_base_quat = inv_quat(self.base_quat)
        self.base_lin_vel[:] = transform_by_quat(self.drone.get_vel(), inv_base_quat)
        self.base_ang_vel[:] = transform_by_quat(self.drone.get_ang(), inv_base_quat)

        # Read and process LiDAR data
        lidar_reading = self.lidar_sensor.read()
        if self.num_envs > 0:
            # Multi-env: distances shape is (n_envs, n_rays, 1)
            self.lidar_data = lidar_reading.distances.squeeze(-1)
        else:
            # Single env: distances shape is (n_rays, 1)
            self.lidar_data = lidar_reading.distances.squeeze(-1)
        
        # Filter out readings below minimum range (to ignore drone body)
        # Replace close readings with max_range
        min_detection_range = 0.15  # 15cm - larger than drone radius (~6cm)
        self.lidar_data = torch.where(
            self.lidar_data < min_detection_range,
            torch.full_like(self.lidar_data, self.lidar_max_range),
            self.lidar_data
        )
        
        self.F, self.R, self.L = self._process_lidar(self.lidar_data)

        # Compute observations
        # Observation: [F, R, L, lin_vel(3), euler(3), ang_vel(3), last_actions(3)]
        self.obs_buf = torch.cat(
            [
                (self.F.unsqueeze(1) * self.obs_scales["lidar"]).clip(-1, 1),
                (self.R.unsqueeze(1) * self.obs_scales["lidar"]).clip(-1, 1),
                (self.L.unsqueeze(1) * self.obs_scales["lidar"]).clip(-1, 1),
                (self.base_lin_vel * self.obs_scales["lin_vel"]).clip(-1, 1),
                (self.base_euler * self.obs_scales["euler"]).clip(-1, 1),
                (self.base_ang_vel * self.obs_scales["ang_vel"]).clip(-1, 1),
                self.last_actions,
            ],
            axis=-1,
        )
        self.extras["observations"]["critic"] = self.obs_buf

    def _velocity_to_rpm(self, x_vel_cmd, y_vel_cmd, yaw_rate_cmd):
        """
        Convert velocity commands to rotor RPMs using simple differential control.
        
        Args:
            x_vel_cmd: Forward velocity (m/s) in drone frame
            y_vel_cmd: Lateral velocity (m/s) in drone frame  
            yaw_rate_cmd: Yaw rate (rad/s)
            
        Returns:
            rpm: (n_envs, 4) tensor of rotor RPMs
        """
        # Get current velocities in drone frame
        inv_base_quat = inv_quat(self.base_quat)
        vel_drone_frame = transform_by_quat(self.drone.get_vel(), inv_base_quat)
        ang_drone_frame = transform_by_quat(self.drone.get_ang(), inv_base_quat)
        
        # Velocity errors
        x_vel_error = x_vel_cmd - vel_drone_frame[:, 0]
        y_vel_error = y_vel_cmd - vel_drone_frame[:, 1]
        yaw_error = yaw_rate_cmd - ang_drone_frame[:, 2]
        
        # Simple proportional control
        # Base thrust (hover)
        base_thrust = torch.ones((self.num_envs,), device=gs.device) * self.hover_rpm
        
        # Control deltas
        pitch_delta = self.kp_vel * x_vel_error  # forward/backward
        roll_delta = self.kp_vel * y_vel_error   # left/right
        yaw_delta = self.kp_yaw * yaw_error      # rotation
        
        # Differential mixing for quadcopter
        # Standard X configuration: [front-right, rear-left, front-left, rear-right]
        rpm = torch.zeros((self.num_envs, 4), device=gs.device, dtype=gs.tc_float)
        
        rpm[:, 0] = base_thrust + pitch_delta - roll_delta - yaw_delta  # Front-right
        rpm[:, 1] = base_thrust - pitch_delta + roll_delta - yaw_delta  # Rear-left  
        rpm[:, 2] = base_thrust + pitch_delta + roll_delta + yaw_delta  # Front-left
        rpm[:, 3] = base_thrust - pitch_delta - roll_delta + yaw_delta  # Rear-right
        
        # Clip to safe RPM range
        rpm = torch.clip(rpm, 5000, 25000)
        
        return rpm

    def step(self, actions):
        # Clip and scale actions to velocity ranges
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        
        # Scale actions to velocity commands
        x_vel_cmd = self.actions[:, 0] * self.max_x_vel
        y_vel_cmd = self.actions[:, 1] * self.max_y_vel
        yaw_rate_cmd = self.actions[:, 2] * self.max_yaw_rate
        
        # Convert to rotor RPMs
        rpm = self._velocity_to_rpm(x_vel_cmd, y_vel_cmd, yaw_rate_cmd)
        self.drone.set_propellels_rpm(rpm)
        
        # Step simulation
        self.scene.step()

        # Update buffers
        self.episode_length_buf += 1
        
        # Update state and observations
        self._update_state_and_observations()

        # Check termination conditions
        self.collision_condition = (
            (self.F < self.env_cfg["collision_threshold"])
            | (self.R < self.env_cfg["collision_threshold"])
            | (self.L < self.env_cfg["collision_threshold"])
        )
        
        self.crash_condition = (
            self.collision_condition
            | (torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"])
            | (torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"])
            | (self.base_pos[:, 2] < self.env_cfg["termination_if_close_to_ground"])
            | (self.base_pos[:, 2] > self.env_cfg.get("termination_if_too_high", 2.5))
        )
        
        self.reset_buf = (self.episode_length_buf > self.max_episode_length) | self.crash_condition

        time_out_idx = (self.episode_length_buf > self.max_episode_length).nonzero(as_tuple=False).reshape((-1,))
        self.extras["time_outs"] = torch.zeros_like(self.reset_buf, device=gs.device, dtype=gs.tc_float)
        self.extras["time_outs"][time_out_idx] = 1.0

        self.reset_idx(self.reset_buf.nonzero(as_tuple=False).reshape((-1,)))

        # Compute rewards
        self.rew_buf[:] = 0.0
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # Update last actions for next observation
        self.last_actions[:] = self.actions[:]

        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.extras

    def get_privileged_observations(self):
        return None

    def reset_idx(self, envs_idx):
        if len(envs_idx) == 0:
            return

        # Reset drone position and orientation
        if self.env_cfg.get("randomize_init_pos", False):
            # Random positions in a small area
            random_offset = gs_rand_float(-1.0, 1.0, (len(envs_idx), 3), gs.device)
            random_offset[:, 2] = 0  # Keep same height
            self.base_pos[envs_idx] = self.base_init_pos + random_offset
        else:
            self.base_pos[envs_idx] = self.base_init_pos
            
        self.base_quat[envs_idx] = self.base_init_quat.reshape(1, -1)
        self.drone.set_pos(self.base_pos[envs_idx], zero_velocity=True, envs_idx=envs_idx)
        self.drone.set_quat(self.base_quat[envs_idx], zero_velocity=True, envs_idx=envs_idx)
        self.base_lin_vel[envs_idx] = 0
        self.base_ang_vel[envs_idx] = 0
        self.drone.zero_all_dofs_velocity(envs_idx)

        # Reset buffers
        self.last_actions[envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = True
        
        # Reset reward computation buffers
        self.rep_eq_last[envs_idx] = 0.0
        self.Uattr_last[envs_idx] = 0.0

        # Fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]["rew_" + key] = (
                torch.mean(self.episode_sums[key][envs_idx]).item() / self.env_cfg["episode_length_s"]
            )
            self.episode_sums[key][envs_idx] = 0.0

    def reset(self):
        self.reset_buf[:] = True
        self.reset_idx(torch.arange(self.num_envs, device=gs.device))
        
        # Step once to update sensors and get valid observations
        self.scene.step()
        self._update_state_and_observations()
        
        return self.obs_buf, None

    # ------------ Reward functions (based on paper) ----------------
    
    def _reward_variable_pitch(self):
        """Reward1: Encourages forward movement when path is clear."""
        Ft = 1.5  # threshold distance
        beta = 1.0
        eta = 5.0
        eps = 1e-6
        
        reward = torch.where(
            self.F > Ft,
            beta * (self.F - Ft) ** 2,
            -eta * torch.exp(1.0 / (self.F - Ft + eps))
        )
        return reward

    def _reward_obstacle_avoidance(self):
        """Reward2: Avoids obstacles using repulsive potential."""
        d0 = 2.0  # influence distance
        lambda_param = 1.0
        kappa = 0.5
        omega = 2.0
        rho = 1.0
        
        # Compute repulsion from right and left
        Urep_R = torch.where(
            self.R <= d0,
            lambda_param * (1.0/torch.clamp(self.R, min=0.1) - 1.0/d0) ** 2,
            torch.zeros_like(self.R)
        )
        
        Urep_L = torch.where(
            self.L <= d0,
            lambda_param * (1.0/torch.clamp(self.L, min=0.1) - 1.0/d0) ** 2,
            torch.zeros_like(self.L)
        )
        
        # Repulsion equilibrium
        rep_eq_current = torch.abs(Urep_R - Urep_L)
        delta_rep_eq = rep_eq_current - self.rep_eq_last
        self.rep_eq_last = rep_eq_current.clone()
        
        # Three-part reward based on repulsion equilibrium
        reward = torch.where(
            rep_eq_current < kappa,
            torch.where(
                delta_rep_eq < 0,
                4 * kappa / torch.clamp(rep_eq_current, min=0.1),
                kappa / torch.clamp(rep_eq_current, min=0.1) - 1
            ),
            -omega * torch.exp(rep_eq_current - rho)
        )
        
        return reward

    def _reward_center_alignment(self):
        """Reward3: Encourages staying centered in corridor."""
        mu = 0.5
        delta_param = 0.1
        
        Uattr = -mu * (self.R - self.L) ** 2 + delta_param
        delta_attr = Uattr - self.Uattr_last
        self.Uattr_last = Uattr.clone()
        
        reward = torch.where(
            delta_attr > 0,
            2 * Uattr,
            Uattr
        )
        return reward

    def _reward_exploration(self):
        """Reward4: Exploration reward (placeholder, set to zero)."""
        return torch.zeros((self.num_envs,), device=gs.device, dtype=gs.tc_float)

    def _reward_collision(self):
        """Penalty for collision."""
        return -self.collision_condition.float()

    def _reward_smooth(self):
        """Penalty for jerky actions."""
        return -torch.sum(torch.square(self.actions - self.last_actions), dim=1)

    def _reward_angular(self):
        """Penalty for excessive angular velocity."""
        return -torch.norm(self.base_ang_vel / 3.14159, dim=1)
