from __future__ import print_function
import time
import math
import odrive
from odrive.enums import (
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    MOTOR_TYPE_HIGH_CURRENT,
    ENCODER_MODE_INCREMENTAL,
    ENCODER_MODE_HALL,
    CONTROL_MODE_POSITION_CONTROL,
    InputMode,
)
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import Quadruped_leg_test_algo as qlt

@dataclass
class LegConfig:
    """Configuration parameters for a leg"""
    L1: float = 24.0  # Upper leg length (hip to knee)
    L2: float = 37.5  # Lower leg length (knee to foot)
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
        for param, value in config.items():
            setattr(self.axis, param, value)
    
    def calibrate(self):
        """Perform motor calibration sequence"""
        # Motor calibration
        self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(7.5)
        self.axis.motor.config.pre_calibrated = True
        
        # Encoder calibration
        self.axis.encoder.config.calib_range = 0.1
        self.axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(17)
        self.axis.encoder.config.pre_calibrated = True
        
    def set_closed_loop_control(self):
        """Set motor to closed loop control mode"""
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def set_position(self, position: float):
        """Set motor position"""
        self.axis.controller.input_pos = float(position)

class Leg:
    """Class to handle a single leg of the quadruped"""
    def __init__(self, odrv: odrive.ODrive, config: LegConfig):
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
            'motor.config.motor_type': MOTOR_TYPE_HIGH_CURRENT,
            'motor.config.resistance_calib_max_voltage': 6,
            'motor.config.calibration_current': 12,
            'motor.config.current_lim': 20.0,
            'motor.config.requested_current_range': 25.0,
            'encoder.config.mode': ENCODER_MODE_INCREMENTAL,
            'encoder.config.cpr': self.config.hip_encoder_cpr,
            'controller.config.control_mode': CONTROL_MODE_POSITION_CONTROL,
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
            'motor.config.motor_type': MOTOR_TYPE_HIGH_CURRENT,
            'motor.config.resistance_calib_max_voltage': 6,
            'motor.config.calibration_current': 12,
            'motor.config.current_lim': 60.0,
            'motor.config.requested_current_range': 70.0,
            'encoder.config.mode': ENCODER_MODE_HALL,
            'encoder.config.cpr': self.config.knee_encoder_cpr,
            'controller.config.control_mode': CONTROL_MODE_POSITION_CONTROL,
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
                x if not self.config.is_mirrored else -x,
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
        self.odrv0 = None
        self.legs = {}
        
    def connect(self):
        """Connect to ODrive and initialize"""
        print("Looking for ODrive...")
        self.odrv0 = odrive.find_any()
        
        # Configure ODrive parameters
        self.odrv0.config.brake_resistance = 0.5
        self.odrv0.config.dc_max_positive_current = 70
        self.odrv0.config.dc_max_negative_current = -2.0
        self.odrv0.config.max_regen_current = 0
        
    def setup_leg(self, name: str, hip_axis, knee_axis, config: LegConfig):
        """Setup a single leg"""
        leg = Leg(self.odrv0, config)
        leg.setup_motors(hip_axis, knee_axis)
        self.legs[name] = leg
        
    def setup_all_legs(self):
        """Setup all four legs with appropriate configurations"""
        # Front left leg
        fl_config = LegConfig(is_mirrored=False)
        self.setup_leg('front_left', self.odrv0.axis0, self.odrv0.axis1, fl_config)
        
        # Add other legs here when you have more ODrives connected
        # Front right leg would use is_mirrored=True
        # fr_config = LegConfig(is_mirrored=True)
        # self.setup_leg('front_right', other_odrive.axis0, other_odrive.axis1, fr_config)
        
    def calibrate_all_legs(self):
        """Calibrate all legs"""
        for name, leg in self.legs.items():
            print(f"Calibrating {name}...")
            leg.calibrate()
            
    def set_all_zero_positions(self):
        """Set zero positions for all legs"""
        for leg in self.legs.values():
            leg.set_zero_position()

def main():
    # Create quadruped instance
    quad = Quadruped()
    
    try:
        # Connect to ODrive
        quad.connect()
        print("Connected to ODrive:", quad.odrv0.serial_number)
        
        # Setup all legs
        quad.setup_all_legs()
        
        # Calibrate all legs
        quad.calibrate_all_legs()
        
        # Example of moving front left leg
        fl_leg = quad.legs['front_left']
        
        # Set zero position
        input("Press Enter to set zero position...")
        fl_leg.set_zero_position()
        
        # Test movement
        while True:
            try:
                x = float(input("Enter x position (-20 to 20, 69 to exit): "))
                if x == 69.0:
                    break
                    
                height = float(input("Enter height (0 to 40): "))
                fl_leg.move_to_point(x, height)
                
            except ValueError as e:
                print(f"Invalid input: {e}")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Set motors to idle state
        if quad.odrv0:
            quad.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            quad.odrv0.axis1.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__":
    main()