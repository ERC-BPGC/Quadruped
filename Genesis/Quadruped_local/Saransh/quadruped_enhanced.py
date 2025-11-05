#!/usr/bin/env python3
"""
Enhanced Quadruped Control System with Config File Support
This is an enhanced version of Quadruped_final_legs.py that works with the new GUI system
"""

from __future__ import print_function
import time
import math
import yaml
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import threading

try:
    import odrive
    from odrive.enums import *
    ODRIVE_AVAILABLE = True
except ImportError:
    print("ODrive not available - simulation mode only")
    ODRIVE_AVAILABLE = False
    # Create dummy classes for simulation
    class AxisState:
        IDLE = 1
        MOTOR_CALIBRATION = 3
        ENCODER_OFFSET_CALIBRATION = 6
        CLOSED_LOOP_CONTROL = 8
    
    class MotorType:
        HIGH_CURRENT = 0
    
    class EncoderMode:
        INCREMENTAL = 0
        HALL = 1
    
    class ControlMode:
        POSITION_CONTROL = 3
    
    class InputMode:
        TRAP_TRAJ = 5

try:
    import Quadruped_leg_test_algo as qlt
except ImportError:
    print("Warning: Quadruped_leg_test_algo not found")
    # Create dummy functions
    class qlt:
        @staticmethod
        def solve_M(L1, L2, x, height):
            # Simple inverse kinematics approximation
            distance = np.sqrt(x**2 + height**2)
            if distance > L1 + L2:
                raise ValueError("Position unreachable")
            
            alpha = np.arccos((L1**2 + distance**2 - L2**2) / (2 * L1 * distance))
            beta = np.arccos((L1**2 + L2**2 - distance**2) / (2 * L1 * L2))
            
            return alpha, beta


@dataclass
class LegConfig:
    """Configuration parameters for a leg"""
    L1: float = 25.0  # Upper leg length (hip to knee)
    L2: float = 33.0  # Lower leg length (knee to foot)
    hip_pole_pairs: int = 20
    knee_pole_pairs: int = 7
    hip_encoder_cpr: int = 2400
    knee_encoder_cpr: int = 42
    is_mirrored: bool = False  # True for right legs, False for left legs

    @classmethod
    def from_config(cls, config_dict, leg_specific_config=None):
        """Create LegConfig from configuration dictionary"""
        leg_config = config_dict.get('leg_config', {})
        
        # Start with base config
        kwargs = {
            'L1': leg_config.get('L1', 25.0),
            'L2': leg_config.get('L2', 33.0),
            'hip_pole_pairs': leg_config.get('hip_pole_pairs', 20),
            'knee_pole_pairs': leg_config.get('knee_pole_pairs', 7),
            'hip_encoder_cpr': leg_config.get('hip_encoder_cpr', 2400),
            'knee_encoder_cpr': leg_config.get('knee_encoder_cpr', 42),
            'is_mirrored': False
        }
        
        # Apply leg-specific overrides
        if leg_specific_config:
            kwargs.update(leg_specific_config)
        
        return cls(**kwargs)


class Motor:
    """Class to handle ODrive motor configuration and control"""
    def __init__(self, axis, config: dict, simulation_mode=False):
        self.axis = axis
        self.simulation_mode = simulation_mode
        if not simulation_mode:
            self.configure(config)
        
    def configure(self, config: dict):
        """Configure motor parameters"""
        if self.simulation_mode:
            return
            
        print(f"Configuring motor with parameters: {config}")
            
        # Convert string motor types and modes to enums
        enum_mappings = {
            'HIGH_CURRENT': MotorType.HIGH_CURRENT,
            'INCREMENTAL': EncoderMode.INCREMENTAL,
            'HALL': EncoderMode.HALL,
            'POSITION_CONTROL': ControlMode.POSITION_CONTROL,
            'TRAP_TRAJ': InputMode.TRAP_TRAJ
        }
        
        for param, value in config.items():
            # Convert enum strings to actual enums
            if isinstance(value, str) and value in enum_mappings:
                value = enum_mappings[value]
                
            print(f"Setting {param} = {value}")
            
            try:
                # Navigate nested attributes
                attrs = param.split('.')
                obj = self.axis
                for attr in attrs[:-1]:
                    obj = getattr(obj, attr)
                setattr(obj, attrs[-1], value)
                
                # Verify the setting was applied (for important parameters)
                if any(key in param for key in ['current_lim', 'calibration_current', 'calibration_lockin']):
                    final_obj = self.axis
                    for attr in attrs[:-1]:
                        final_obj = getattr(final_obj, attr)
                    actual_value = getattr(final_obj, attrs[-1])
                    print(f"✓ Verification: {param} = {actual_value}")
                    
            except Exception as e:
                print(f"✗ Error setting {param}: {e}")
        
        print("Motor configuration completed")
    
    def calibrate(self):
        """Perform motor calibration sequence"""
        if self.simulation_mode:
            print("Simulation: Motor calibration")
            time.sleep(0.1)  # Quick simulation
            return
            
        # Motor calibration
        self.axis.requested_state = AxisState.MOTOR_CALIBRATION
        time.sleep(7.5)
        self.axis.motor.config.pre_calibrated = True
        
        # Encoder calibration
        self.axis.encoder.config.calib_range = 0.1
        self.axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
        time.sleep(17)
        self.axis.encoder.config.pre_calibrated = True
        
    def set_closed_loop_control(self):
        """Set motor to closed loop control mode"""
        if not self.simulation_mode:
            self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        
    def set_position(self, position: float):
        """Set motor position"""
        if not self.simulation_mode:
            self.axis.controller.input_pos = float(position)
    
    def get_current(self):
        """Get motor current"""
        if self.simulation_mode:
            return 2.0 + 0.5 * np.random.randn()
        try:
            return self.axis.motor.current_control.Iq_measured
        except:
            return 0.0
    
    def get_position(self):
        """Get motor position"""
        if self.simulation_mode:
            return 10.0 * np.random.randn()
        try:
            return self.axis.encoder.pos_estimate
        except:
            return 0.0
    
    def get_velocity(self):
        """Get motor velocity"""
        if self.simulation_mode:
            return 5.0 * np.random.randn()
        try:
            return self.axis.encoder.vel_estimate
        except:
            return 0.0


class Leg:
    """Class to handle a single leg of the quadruped"""
    def __init__(self, odrv, config: LegConfig, motor_config: dict, simulation_mode=False):
        self.odrv = odrv
        self.config = config
        self.motor_config = motor_config
        self.simulation_mode = simulation_mode
        self.hip_motor = None
        self.knee_motor = None
        self.zero_hip = 0
        self.zero_knee = 0
        
    def setup_motors(self, hip_axis, knee_axis):
        """Initialize and configure motors"""
        # Get motor configurations from config
        hip_config = self._build_motor_config('hip')
        knee_config = self._build_motor_config('knee')
        
        self.hip_motor = Motor(hip_axis, hip_config, self.simulation_mode)
        self.knee_motor = Motor(knee_axis, knee_config, self.simulation_mode)
        
        # Save configuration to make it persistent
        if not self.simulation_mode:
            print("Saving motor configuration...")
            try:
                hip_axis.motor.config.save_configuration()
                knee_axis.motor.config.save_configuration()
                print("Motor configuration saved successfully")
            except Exception as e:
                print(f"Warning: Could not save motor configuration: {e}")
                # Try the alternative save method
                try:
                    self.odrv.save_configuration()
                    print("ODrive configuration saved using alternative method")
                except Exception as e2:
                    print(f"Warning: Alternative save method also failed: {e2}")
    
    def _build_motor_config(self, motor_type):
        """Build motor configuration from config dictionary"""
        if motor_type not in self.motor_config:
            return {}
        
        motor_cfg = self.motor_config[motor_type]
        config = {}
        
        # Map config keys to ODrive parameter paths
        param_mapping = {
            'motor_type': 'motor.config.motor_type',
            'pole_pairs': 'motor.config.pole_pairs',
            'resistance_calib_max_voltage': 'motor.config.resistance_calib_max_voltage',
            'calibration_current': 'motor.config.calibration_current',
            'current_lim': 'motor.config.current_lim',
            'requested_current_range': 'motor.config.requested_current_range',
            'current_control_bandwidth': 'motor.config.current_control_bandwidth',
            'calibration_lockin_current': 'config.calibration_lockin.current',
            'encoder_bandwidth': 'encoder.config.bandwidth',
            'encoder_mode': 'encoder.config.mode',
            'control_mode': 'controller.config.control_mode',
            'input_mode': 'controller.config.input_mode',
            'pos_gain': 'controller.config.pos_gain',
            'vel_gain': 'controller.config.vel_gain',
            'vel_integrator_gain': 'controller.config.vel_integrator_gain',
            'vel_limit': 'controller.config.vel_limit',
            'vel_limit_tolerance': 'controller.config.vel_limit_tolerance',
            'enable_vel_limit': 'controller.config.enable_vel_limit',
            'torque_ramp_rate': 'controller.config.torque_ramp_rate',
            'trap_traj_vel_limit': 'trap_traj.config.vel_limit',
            'trap_traj_accel_limit': 'trap_traj.config.accel_limit',
            'trap_traj_decel_limit': 'trap_traj.config.decel_limit'
        }
        
        # Add CPR based on motor type and leg config
        if motor_type == 'hip':
            config['encoder.config.cpr'] = self.config.hip_encoder_cpr
            config['motor.config.pole_pairs'] = self.config.hip_pole_pairs
        else:
            config['encoder.config.cpr'] = self.config.knee_encoder_cpr
            config['motor.config.pole_pairs'] = self.config.knee_pole_pairs
        
        # Build config from motor_cfg with debugging
        print(f"Building {motor_type} motor config from: {motor_cfg}")
        for key, value in motor_cfg.items():
            if key in param_mapping:
                config[param_mapping[key]] = value
                print(f"Mapped {key} ({value}) -> {param_mapping[key]}")
            else:
                print(f"Warning: Unknown parameter {key} in motor config")
        
        print(f"Final {motor_type} motor config: {config}")
        return config
    
    def calibrate(self):
        """Calibrate both motors"""
        print("Calibrating hip motor...")
        self.hip_motor.calibrate()
        print("Calibrating knee motor...")
        self.knee_motor.calibrate()
        
        self.hip_motor.set_closed_loop_control()
        self.knee_motor.set_closed_loop_control()
    
    def set_zero_position(self, interactive=True):
        """Set current position as zero reference"""
        if interactive and not self.simulation_mode:
            # Interactive calibration as in original code
            Calib_pos_hip = 0
            Calib_pos_knee = 0

            while Calib_pos_hip != 69.0:
                self.hip_motor.set_position(Calib_pos_hip)
                self.knee_motor.set_position(Calib_pos_knee)

                Calib_pos_hip = float(input("Enter Calib_pos_hip Pweaseee... : "))
                Calib_pos_knee = float(input("Enter Calib_Calib_pos_knee Pweaseee... : "))

            self.zero_hip = self.hip_motor.get_position()
            self.zero_knee = self.knee_motor.get_position()
        else:
            # Non-interactive or simulation mode
            self.zero_hip = self.hip_motor.get_position()
            self.zero_knee = self.knee_motor.get_position()
        
    def move_to_angles(self, hip_angle: float, knee_angle: float):
        """Move leg to specified joint angles"""
        # Apply mirroring if needed
        if self.config.is_mirrored:
            hip_angle = -hip_angle
            
        self.hip_motor.set_position(self.zero_hip + hip_angle)
        self.knee_motor.set_position(self.zero_knee + knee_angle)
        
    def move_to_point(self, x: float, height: float) -> bool:
        """Move end effector to specified point using inverse kinematics"""
        try:
            alpha, beta = qlt.solve_M(
                self.config.L1, 
                self.config.L2,
                x,
                height
            )
            
            # Convert to motor angles
            hip_angle = (((1.57 - alpha) * 1) / 6.28) * 10.5
            knee_angle = ((1.57 - alpha + beta - 1.12) * 4.58)
            
            self.move_to_angles(hip_angle, knee_angle)
            return True
        except (ValueError, AttributeError) as e:
            print(f"Invalid position: {e}")
            return False
    
    def get_joint_data(self):
        """Get current joint data for monitoring"""
        return {
            'hip': {
                'current': self.hip_motor.get_current(),
                'position': self.hip_motor.get_position(),
                'velocity': self.hip_motor.get_velocity()
            },
            'knee': {
                'current': self.knee_motor.get_current(),
                'position': self.knee_motor.get_position(),
                'velocity': self.knee_motor.get_velocity()
            }
        }


class QuadrupedEnhanced:
    """Enhanced class to manage all four legs of the quadruped with config file support"""
    def __init__(self, config_file=None, simulation_mode=None):
        self.config = self.load_config(config_file)
        
        # Determine simulation mode
        if simulation_mode is None:
            self.simulation_mode = self.config.get('system', {}).get('simulation_mode', True)
        else:
            self.simulation_mode = simulation_mode
        
        self.odrives = []
        self.legs = {}
        self.odrive_leg_map = {}
        self.connected = False
        
        # Build leg configs from config file
        self._build_leg_configs()
        
    def load_config(self, config_file=None):
        """Load configuration from file"""
        if config_file is None:
            config_file = "config.yaml"
        
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            print(f"Config file {config_file} not found, using defaults")
            return self._get_default_config()
        except yaml.YAMLError as e:
            print(f"Error parsing config file: {e}")
            return self._get_default_config()
    
    def _get_default_config(self):
        """Get default configuration"""
        return {
            'system': {'simulation_mode': True},
            'odrive_mapping': {},
            'leg_config': {'L1': 25.0, 'L2': 33.0},
            'motor_config': {'hip': {}, 'knee': {}},
            'leg_specific': {}
        }
    
    def _build_leg_configs(self):
        """Build leg configurations from config file"""
        self.leg_configs = {}
        leg_specific = self.config.get('leg_specific', {})
        
        for leg_name in ['front_left', 'front_right', 'back_left', 'back_right']:
            specific_config = leg_specific.get(leg_name, {})
            self.leg_configs[leg_name] = LegConfig.from_config(self.config, specific_config)
        
    def connect(self):
        """Connect to ODrives by serial and initialize"""
        if self.simulation_mode:
            print("Running in simulation mode - no ODrive connection")
            self.connected = True
            return
        
        if not ODRIVE_AVAILABLE:
            raise RuntimeError("ODrive library not available")
        
        print("Connecting to ODrives by serial...")
        serial_leg_map = self.config.get('odrive_mapping', {})
        
        self.odrives = []
        self.odrive_leg_map = {}
        
        for serial, leg_name in serial_leg_map.items():
            print(f"Connecting to ODrive for {leg_name} (serial: {serial})...")
            odrv = odrive.find_any(serial_number=serial)
            print(f"Connected to ODrive for {leg_name}: {odrv.serial_number}")
            self.odrives.append(odrv)
            self.odrive_leg_map[leg_name] = odrv
            
            # Configure ODrive
            odrive_config = self.config.get('odrive_config', {})
            odrv.config.brake_resistance = odrive_config.get('brake_resistance', 0.5)
            odrv.config.dc_max_positive_current = odrive_config.get('dc_max_positive_current', 70)
            odrv.config.dc_max_negative_current = odrive_config.get('dc_max_negative_current', -2.0)
            odrv.config.max_regen_current = odrive_config.get('max_regen_current', 0)
        
        self.connected = True
        
    def setup_leg(self, name: str, odrv, hip_axis, knee_axis, config: LegConfig):
        """Setup a single leg"""
        motor_config = self.config.get('motor_config', {})
        leg = Leg(odrv, config, motor_config, self.simulation_mode)
        leg.setup_motors(hip_axis, knee_axis)
        self.legs[name] = leg
        
    def setup_all_legs(self):
        """Setup all four legs with appropriate configurations"""
        if self.simulation_mode:
            # Create dummy legs for simulation
            for leg_name, config in self.leg_configs.items():
                leg = Leg(None, config, self.config.get('motor_config', {}), True)
                leg.setup_motors(None, None)
                self.legs[leg_name] = leg
                print(f"Set up {leg_name} leg in simulation mode")
        else:
            for leg_name, odrv in self.odrive_leg_map.items():
                config = self.leg_configs[leg_name]
                self.setup_leg(leg_name, odrv, odrv.axis0, odrv.axis1, config)
                print(f"Set up {leg_name} leg with ODrive serial {odrv.serial_number}")
        
    def calibrate_leg_thread(self, name, leg):
        """Calibrate a leg in a separate thread"""
        print(f"Calibrating {name}...")
        leg.calibrate()

    def calibrate_all_legs(self, simultaneous=True):
        """Calibrate all legs, either simultaneously (default) or sequentially."""
        if simultaneous:
            threads = []
            for name, leg in self.legs.items():
                t = threading.Thread(target=self.calibrate_leg_thread, args=(name, leg))
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
        else:
            for name, leg in self.legs.items():
                self.calibrate_leg_thread(name, leg)
            
    def set_all_zero_positions(self, interactive=True):
        """Set zero positions for all legs"""
        for name, leg in self.legs.items():
            print(f"Setting zero position for {name}")
            leg.set_zero_position(interactive and not self.simulation_mode)
    
    def get_all_joint_data(self):
        """Get joint data from all legs"""
        data = {'timestamp': time.time(), 'motors': {}}
        for leg_name, leg in self.legs.items():
            data['motors'][leg_name] = leg.get_joint_data()
        return data
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        if self.simulation_mode:
            print("Simulation: Emergency stop")
            return
        
        for odrv in self.odrives:
            try:
                odrv.axis0.requested_state = AxisState.IDLE
                odrv.axis1.requested_state = AxisState.IDLE
            except:
                pass
    
    def disconnect(self):
        """Disconnect and idle all motors"""
        self.emergency_stop()
        self.connected = False


def main_enhanced():
    """Enhanced main function with config file support"""
    print("Quadruped Enhanced Control System")
    print("=" * 40)
    
    # Create quadruped instance with config
    quad = QuadrupedEnhanced(config_file="config.yaml")
    
    try:
        # Connect to all ODrives (or run in simulation)
        quad.connect()
        
        # Setup all legs
        quad.setup_all_legs()
        
        if not quad.simulation_mode:
            # Ask user if they want simultaneous or sequential calibration
            print("\nCalibration mode:")
            print("1. Simultaneous (default)")
            print("2. Sequential")
            mode = input("Select calibration mode (1/2, default 1): ").strip()
            simultaneous = (mode != "2")
            
            # Calibrate all legs
            print("\nStarting calibration of all legs...")
            quad.calibrate_all_legs(simultaneous=simultaneous)

            print("\nSetting zero positions for all legs...")
            quad.set_all_zero_positions()
        else:
            print("\nSimulation mode - skipping calibration")
            quad.set_all_zero_positions(interactive=False)

        # Test movement of each leg
        print("\nReady for control commands!")
        while True:
            try:
                # Select leg to move
                print("\nAvailable legs:")
                print("1. Front Left")
                print("2. Front Right")
                print("3. Back Left")
                print("4. Back Right")
                print("5. Show joint data")
                print("0. Exit")

                choice = int(input("\nSelect option (0-5): "))
                if choice == 0:
                    break
                elif choice == 5:
                    # Show joint data
                    data = quad.get_all_joint_data()
                    print("\nCurrent joint data:")
                    for leg_name, leg_data in data['motors'].items():
                        print(f"{leg_name}:")
                        for joint, joint_data in leg_data.items():
                            print(f"  {joint}: current={joint_data['current']:.2f}A, "
                                 f"pos={joint_data['position']:.2f}, vel={joint_data['velocity']:.2f}")
                    continue

                leg_map = {
                    1: 'front_left',
                    2: 'front_right',
                    3: 'back_left',
                    4: 'back_right'
                }

                if choice not in leg_map:
                    print("Invalid choice!")
                    continue

                leg = quad.legs[leg_map[choice]]

                # Get movement coordinates
                x = float(input("Enter x position (-20 to 20): "))
                height = float(input("Enter height (0 to 40): "))

                # Move leg
                success = leg.move_to_point(x, height)
                if success:
                    print("Movement completed")
                else:
                    print("Movement failed - position may be unreachable")

            except ValueError as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                break
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Disconnect and cleanup
        print("\nDisconnecting...")
        quad.disconnect()


if __name__ == "__main__":
    main_enhanced()