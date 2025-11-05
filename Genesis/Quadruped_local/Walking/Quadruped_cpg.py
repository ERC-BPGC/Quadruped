import numpy as np
import time

class QuadrupedCPG:
    """
    A Simple Sine-Wave Based Central Pattern Generator (CPG) for a Quadruped.
    This class generates rhythmic end-effector (foot) positions for a walking gait.
    """
    def __init__(self, stride_length=7.5, step_height=10.0, frequency=1.0, default_height=35.0):
        """
        Initializes the CPG with default gait parameters.

        Args:
            stride_length (float): The forward/backward distance covered in one step (cm).
            step_height (float): The maximum height a foot is lifted during a step (cm).
            frequency (float): The frequency of the walking cycle (Hz).
            default_height (float): The neutral standing height of the feet (cm).
        """
        self.stride_length = stride_length
        self.step_height = step_height
        self.frequency = frequency
        self.default_height = default_height

        # Gait parameters for a walk: legs move in sequence (one at a time).
        # Phases are defined in radians for a walking pattern.
        self.phase_offsets = {
            'front_left': 0,
            'front_right': np.pi,
            'back_left': np.pi/2,
            'back_right': 3*np.pi/2
        }
        self.start_time = time.time()

    def update_parameters(self, stride_length, step_height, frequency, default_height):
        """Updates the CPG gait parameters."""
        self.stride_length = stride_length
        self.step_height = step_height
        self.frequency = frequency
        self.default_height = default_height

    def get_leg_positions(self, t, front_offset=0.0, back_offset=0.0, front_height_offset=0.0, back_height_offset=0.0):
        """
        Calculates the target (x, height) end-effector position for each leg at a given time t.

        Args:
            t (float): The elapsed time since the start of the CPG cycle.
            front_offset (float): X-axis offset for front legs (cm). Default: 0.0.
            back_offset (float): X-axis offset for back legs (cm). Default: 0.0.
            front_height_offset (float): Height offset for front legs (cm). Default: 0.0.
            back_height_offset (float): Height offset for back legs (cm). Default: 0.0.

        Returns:
            dict: A dictionary mapping leg names to their target (x, height) tuples.
        """
        leg_positions = {}
        angular_freq = 2 * np.pi * self.frequency
        
        for leg_name, phase_offset in self.phase_offsets.items():
            # The continuous phase for this leg
            phase = angular_freq * t + phase_offset
            
            # X position (forward/backward movement) based on a cosine wave.
            # This creates smooth forward and backward motion.
            x_pos = self.stride_length / 2 * np.cos(phase)  # Removed negative sign for forward motion
            
            # Z position (height) - INVERTED: higher values = foot down, lower values = foot up
            height = self.default_height
            # The leg is lifted (reduced height) during the "swing phase" (when sin(phase) is positive).
            # This subtracts from height to lift the foot UP.
            if np.sin(phase) > 0:
                height -= self.step_height * np.sin(phase)  # This SUBTRACTS height (lifts foot UP)
            
            # Adjust x position and height for front/back legs using provided offsets
            if 'front' in leg_name:
                x_pos += front_offset
                height += front_height_offset
            else: 
                x_pos += back_offset
                height += back_height_offset

            leg_positions[leg_name] = (x_pos, height)
            
        return leg_positions

    def reset_time(self):
        """Resets the internal timer for the CPG cycle."""
        self.start_time = time.time()
