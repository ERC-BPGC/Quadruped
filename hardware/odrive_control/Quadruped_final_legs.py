from __future__ import print_function
import time
import math
import odrive
from odrive.enums import *
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import threading
import Quadruped_leg_test_algo as qlt

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

class Motor:
    """Class to handle ODrive motor configuration and control"""
    def __init__(self, axis, config: dict):
        self.axis = axis
        self.configure(config)
        
    def configure(self, config: dict):
        """Configure motor parameters"""
        # for param, value in config.items():
        #     setattr(self.axis, param, value)
        for param, value in config.items():
            attrs = param.split('.')
            obj = self.axis
            for attr in attrs[:-1]:
                obj = getattr(obj, attr)
            setattr(obj, attrs[-1], value)
    
    def calibrate(self):
        """Perform motor calibration sequence"""
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
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        
    def set_position(self, position: float):
        """Set motor position"""
        self.axis.controller.input_pos = float(position)

class Leg:
    """Class to handle a single leg of the quadruped"""
    def __init__(self, odrv, config: LegConfig):
        self.odrv = odrv
        self.config = config
        self.hip_motor = None
        self.knee_motor = None
        self.zero_hip = 0
        self.zero_knee = 0
        
    def setup_motors(self, hip_axis, knee_axis):
        """Initialize and configure motors"""
        # Hip motor configuration
        hip_config = {
            'motor.config.pole_pairs': self.config.hip_pole_pairs,
            'motor.config.motor_type': MotorType.HIGH_CURRENT,
            'motor.config.resistance_calib_max_voltage': 12,
            'motor.config.calibration_current': 18,
            'motor.config.current_lim': 20.0,
            'motor.config.requested_current_range': 25.0,
            'config.calibration_lockin.current': 20,
            'encoder.config.bandwidth': 750,
            'encoder.config.mode': EncoderMode.INCREMENTAL,
            'encoder.config.cpr': self.config.hip_encoder_cpr,
            'controller.config.control_mode': ControlMode.POSITION_CONTROL,
            'controller.config.pos_gain': 50.0,
            'controller.config.vel_gain': 0.05,
            'controller.config.vel_integrator_gain': 0.0,
            'trap_traj.config.vel_limit': 10,
            'trap_traj.config.accel_limit': 5,
            'trap_traj.config.decel_limit': 5,
            'controller.config.vel_limit': 15.0,
            'controller.config.input_mode': InputMode.TRAP_TRAJ
        }
        
        # Knee motor configuration
        knee_config = {
            'motor.config.pole_pairs': self.config.knee_pole_pairs,
            'motor.config.motor_type': MotorType.HIGH_CURRENT,
            'motor.config.resistance_calib_max_voltage': 12,
            'motor.config.calibration_current': 22,
            'motor.config.current_lim': 60.0,
            'motor.config.requested_current_range': 70.0,
            'config.calibration_lockin.current': 33,
            'encoder.config.bandwidth': 100,
            'encoder.config.mode': EncoderMode.HALL,
            'encoder.config.cpr': self.config.knee_encoder_cpr,
            'controller.config.control_mode': ControlMode.POSITION_CONTROL,
            'controller.config.pos_gain': 20,
            'controller.config.vel_gain': 0.02,
            'controller.config.vel_integrator_gain': 0.1,
            'trap_traj.config.vel_limit': 20,
            'trap_traj.config.accel_limit': 10,
            'trap_traj.config.decel_limit': 10,
            'controller.config.vel_limit': 20.0,
            'controller.config.input_mode': InputMode.TRAP_TRAJ
        }
        
        self.hip_motor = Motor(hip_axis, hip_config)
        self.knee_motor = Motor(knee_axis, knee_config)
    
    def calibrate(self):
        """Calibrate both motors"""
        print("Calibrating hip motor...")
        self.hip_motor.calibrate()
        print("Calibrating knee motor...")
        self.knee_motor.calibrate()
        
        self.hip_motor.set_closed_loop_control()
        self.knee_motor.set_closed_loop_control()
    
    def set_zero_position(self):
        """Set current position as zero reference"""
        self.Calib_pos_hip = 0
        self.Calib_pos_knee = 0

        while self.Calib_pos_hip != 69.0:
            self.hip_motor.set_position(self.Calib_pos_hip)
            self.knee_motor.set_position(self.Calib_pos_knee)

            self.Calib_pos_hip = float(input("Enter Calib_pos_hip Pweaseee... : "))
            self.Calib_pos_knee = float(input("Enter Calib_Calib_pos_knee Pweaseee... : "))

        self.zero_hip = self.hip_motor.axis.encoder.pos_estimate
        self.zero_knee = self.knee_motor.axis.encoder.pos_estimate
        
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
        except ValueError as e:
            print(f"Invalid position: {e}")
            return False

class Quadruped:
    """Class to manage all four legs of the quadruped"""
    def __init__(self):
        self.odrives = []
        self.legs = {}
        self.leg_configs = {
            'front_left': LegConfig(is_mirrored=False),
            'front_right': LegConfig(is_mirrored=True),
            'back_left': LegConfig(is_mirrored=False),
            'back_right': LegConfig(is_mirrored=True)
        }
        
    def connect(self):
        """Connect to ODrives by serial and initialize"""
        print("Connecting to ODrives by serial...")
        serial_leg_map = {
            '3670314C3333': 'front_left',
            '366B315F3333': 'front_right',
            '3669315C3333': 'back_left',
            '367031553333': 'back_right'
        }
        self.odrives = []
        self.odrive_leg_map = {}
        for serial, leg_name in serial_leg_map.items():
            print(f"Connecting to ODrive for {leg_name} (serial: {serial})...")
            odrv = odrive.find_any(serial_number=serial)
            print(f"Connected to ODrive for {leg_name}: {odrv.serial_number}")
            self.odrives.append(odrv)
            self.odrive_leg_map[leg_name] = odrv
            # Configure each ODrive
            odrv.config.brake_resistance = 0.5
            odrv.config.dc_max_positive_current = 70
            odrv.config.dc_max_negative_current = -2.0
            odrv.config.max_regen_current = 0
        
    def setup_leg(self, name: str, odrv, hip_axis, knee_axis, config: LegConfig):
        """Setup a single leg"""
        leg = Leg(odrv, config)
        leg.setup_motors(hip_axis, knee_axis)
        self.legs[name] = leg
        
    def setup_all_legs(self):
        """Setup all four legs with appropriate configurations using explicit serial mapping"""
        for leg_name, odrv in self.odrive_leg_map.items():
            config = self.leg_configs[leg_name]
            self.setup_leg(leg_name, odrv, odrv.axis0, odrv.axis1, config)
            # if leg_name == 'front_right':
            #     odrv.axis1.encoder.config.cpr = 28  # Specific adjustment for front right knee
            print(f"Set up {leg_name} leg with ODrive serial {odrv.serial_number}")
        
    def calibrate_leg_thread(self, name, leg):
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
            
    def set_all_zero_positions(self):
        """Set zero positions for all legs"""
        for leg in self.legs.values():
            leg.set_zero_position()

def main():
    # Create quadruped instance
    quad = Quadruped()
    
    try:
        # Connect to all ODrives
        quad.connect()

        # Setup all legs
        quad.setup_all_legs()

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

        # Test movement of each leg
        while True:
            try:
                # Select leg to move
                print("\nAvailable legs:")
                print("1. Front Left")
                print("2. Front Right")
                print("3. Back Left")
                print("4. Back Right")
                print("0. Exit")

                choice = int(input("\nSelect leg to move (0-4): "))
                if choice == 0:
                    break

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
                leg.move_to_point(x, height)

            except ValueError as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Set all motors to idle state
        print("\nSetting all motors to idle state...")
        for odrv in quad.odrives:
            odrv.axis0.requested_state = AxisState.IDLE
            odrv.axis1.requested_state = AxisState.IDLE

if __name__ == "__main__":
    main()