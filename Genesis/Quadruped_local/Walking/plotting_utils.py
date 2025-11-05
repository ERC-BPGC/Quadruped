#!/usr/bin/env python3
"""
Enhanced plotting and data collection module for the Quadruped GUI
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import queue
from collections import deque


class QuadrupedDataCollector:
    """Handles data collection from ODrives in a separate thread"""
    
    def __init__(self, quadruped=None, simulation_mode=True):
        self.quadruped = quadruped
        self.simulation_mode = simulation_mode
        self.data_queue = queue.Queue()
        self.running = False
        self.thread = None
        
        # Simulation data generation
        self.sim_time = 0
        self.sim_frequency = 0.1  # Simulated frequency for data generation
        
    def start_collection(self):
        """Start data collection in a separate thread"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._collect_data, daemon=True)
        self.thread.start()
    
    def stop_collection(self):
        """Stop data collection"""
        self.running = False
        if self.thread:
            self.thread.join()
    
    def _collect_data(self):
        """Main data collection loop"""
        while self.running:
            try:
                if self.simulation_mode:
                    data = self._generate_simulation_data()
                else:
                    data = self._collect_real_data()
                
                self.data_queue.put(data)
                time.sleep(0.05)  # 20 Hz data collection
                
            except Exception as e:
                print(f"Data collection error: {e}")
                time.sleep(0.1)
    
    def _generate_simulation_data(self):
        """Generate simulated ODrive data"""
        self.sim_time += 0.05
        
        # Generate realistic-looking data with some noise
        data = {
            'timestamp': time.time(),
            'time': self.sim_time,
            'motors': {}
        }
        
        legs = ['front_left', 'front_right', 'back_left', 'back_right']
        motors = ['hip', 'knee']
        
        for leg in legs:
            data['motors'][leg] = {}
            for motor in motors:
                # Simulate different waveforms for different parameters
                base_freq = 0.5 if motor == 'hip' else 0.7
                
                data['motors'][leg][motor] = {
                    'current': 2.0 + 1.5 * np.sin(2 * np.pi * base_freq * self.sim_time) + 0.2 * np.random.randn(),
                    'position': 10 * np.sin(2 * np.pi * base_freq * self.sim_time) + 0.1 * np.random.randn(),
                    'velocity': 20 * np.pi * base_freq * np.cos(2 * np.pi * base_freq * self.sim_time) + 0.5 * np.random.randn(),
                    'voltage': 12.0 + 0.5 * np.sin(2 * np.pi * base_freq * self.sim_time) + 0.1 * np.random.randn(),
                    'temperature': 25.0 + 5.0 * np.abs(np.sin(2 * np.pi * 0.1 * self.sim_time)) + 0.5 * np.random.randn()
                }
        
        return data
    
    def _collect_real_data(self):
        """Collect real data from ODrives"""
        if not self.quadruped or not self.quadruped.connected:
            return None
        
        data = {
            'timestamp': time.time(),
            'time': time.time(),
            'motors': {}
        }
        
        try:
            for leg_name, leg in self.quadruped.legs.items():
                # Helper function to safely get temperature
                def get_temperature(motor):
                    try:
                        # Try different possible temperature attributes
                        if hasattr(motor.axis.motor, 'temperature'):
                            return motor.axis.motor.temperature
                        elif hasattr(motor.axis, 'motor') and hasattr(motor.axis.motor, 'inverter_temperature'):
                            return motor.axis.motor.inverter_temperature
                        elif hasattr(motor.axis, 'inverter') and hasattr(motor.axis.inverter, 'temperature'):
                            return motor.axis.inverter.temperature
                        elif hasattr(motor.axis, 'fet_thermistor') and hasattr(motor.axis.fet_thermistor, 'temperature'):
                            return motor.axis.fet_thermistor.temperature
                        else:
                            return 0.0  # Default if no temperature available
                    except:
                        return 0.0  # Default on any error
                
                data['motors'][leg_name] = {
                    'hip': {
                        'current': leg.hip_motor.axis.motor.current_control.Iq_measured,
                        'position': leg.hip_motor.axis.encoder.pos_estimate,
                        'velocity': leg.hip_motor.axis.encoder.vel_estimate,
                        'voltage': leg.hip_motor.axis.motor.current_control.v_current_control_integral_q,
                        'temperature': get_temperature(leg.hip_motor)
                    },
                    'knee': {
                        'current': leg.knee_motor.axis.motor.current_control.Iq_measured,
                        'position': leg.knee_motor.axis.encoder.pos_estimate,
                        'velocity': leg.knee_motor.axis.encoder.vel_estimate,
                        'voltage': leg.knee_motor.axis.motor.current_control.v_current_control_integral_q,
                        'temperature': get_temperature(leg.knee_motor)
                    }
                }
        except Exception as e:
            print(f"Error collecting real data: {e}")
            return None
        
        return data
    
    def get_latest_data(self):
        """Get the latest data from the queue"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


class LivePlotter:
    """Handles live plotting of ODrive data"""
    
    def __init__(self, ax, canvas, max_points=1000):
        self.ax = ax
        self.canvas = canvas
        self.max_points = max_points
        
        # Data storage
        self.data_buffer = {
            'time': deque(maxlen=max_points),
            'front_left_hip': deque(maxlen=max_points),
            'front_left_knee': deque(maxlen=max_points),
            'front_right_hip': deque(maxlen=max_points),
            'front_right_knee': deque(maxlen=max_points),
            'back_left_hip': deque(maxlen=max_points),
            'back_left_knee': deque(maxlen=max_points),
            'back_right_hip': deque(maxlen=max_points),
            'back_right_knee': deque(maxlen=max_points),
        }
        
        # Plot lines
        self.lines = {}
        self.parameter = 'current'
        self.colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        
        self.setup_plot()
    
    def setup_plot(self):
        """Setup the plot with initial styling"""
        self.ax.clear()
        self.ax.set_title(f'Live {self.parameter.title()} Data')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel(self.get_unit())
        self.ax.grid(True, alpha=0.3)
        
        # Create lines for each motor
        motors = ['front_left_hip', 'front_left_knee', 'front_right_hip', 'front_right_knee',
                 'back_left_hip', 'back_left_knee', 'back_right_hip', 'back_right_knee']
        
        for i, motor in enumerate(motors):
            line, = self.ax.plot([], [], label=motor.replace('_', ' ').title(), 
                               color=self.colors[i % len(self.colors)], linewidth=1.5)
            self.lines[motor] = line
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.canvas.draw()
    
    def get_unit(self):
        """Get the unit for the current parameter"""
        units = {
            'current': 'A',
            'position': 'turns',
            'velocity': 'turns/s',
            'voltage': 'V',
            'temperature': '°C'
        }
        return units.get(self.parameter, '')
    
    def set_parameter(self, parameter):
        """Set the parameter to plot"""
        self.parameter = parameter
        self.setup_plot()
    
    def update_data(self, data):
        """Update the plot with new data"""
        if not data or 'motors' not in data:
            return
        
        try:
            # Add new data points
            self.data_buffer['time'].append(data['time'])
            
            for leg_name, leg_data in data['motors'].items():
                for motor_name, motor_data in leg_data.items():
                    key = f"{leg_name}_{motor_name}"
                    if key in self.data_buffer and self.parameter in motor_data:
                        self.data_buffer[key].append(motor_data[self.parameter])
            
            # Update plot lines
            time_data = list(self.data_buffer['time'])
            
            for motor, line in self.lines.items():
                if motor in self.data_buffer:
                    y_data = list(self.data_buffer[motor])
                    if len(y_data) == len(time_data):
                        line.set_data(time_data, y_data)
            
            # Adjust plot limits
            if time_data:
                self.ax.set_xlim(max(0, time_data[-1] - 30), time_data[-1] + 1)  # Show last 30 seconds
                
                # Auto-scale y-axis
                all_y_data = []
                for motor in self.data_buffer:
                    if motor != 'time' and self.data_buffer[motor]:
                        all_y_data.extend(list(self.data_buffer[motor])[-100:])  # Last 100 points
                
                if all_y_data:
                    y_min, y_max = min(all_y_data), max(all_y_data)
                    y_range = y_max - y_min
                    self.ax.set_ylim(y_min - 0.1 * y_range, y_max + 0.1 * y_range)
            
            self.canvas.draw_idle()
            
        except Exception as e:
            print(f"Error updating plot: {e}")
    
    def clear_data(self):
        """Clear all data from the plot"""
        for buffer in self.data_buffer.values():
            buffer.clear()
        
        for line in self.lines.values():
            line.set_data([], [])
        
        self.canvas.draw()


class ParameterTuner:
    """Handles live parameter tuning for ODrive motors"""
    
    def __init__(self, quadruped=None, simulation_mode=True, logger=None):
        self.quadruped = quadruped
        self.simulation_mode = simulation_mode
        self.logger = logger
        
        # Parameter definitions - comprehensive control parameters
        self.parameters = {
            'PID Control': {
                'pos_gain': {'min': 0, 'max': 100, 'default': 30, 'step': 0.1},  # Increased max from 100 to 500
                'vel_gain': {'min': 0, 'max': 0.1, 'default': 0.025, 'step': 0.0001},  # Decreased max from 1 to 0.2, increased resolution (step from 0.001 to 0.0001)
                'vel_integrator_gain': {'min': 0, 'max': 0.01, 'default': 0.0, 'step': 0.00001},
                'vel_limit': {'min': 0, 'max': 50, 'default': 20.0, 'step': 0.1}
            },
            'Trajectory Control': {
                'trap_traj_vel_limit': {'min': 0, 'max': 50, 'default': 20, 'step': 1},
                'trap_traj_accel_limit': {'min': 0, 'max': 100, 'default': 10, 'step': 1},
                'trap_traj_decel_limit': {'min': 0, 'max': 100, 'default': 10, 'step': 1}
            },
            'Current Limits': {
                'current_lim': {'min': 0, 'max': 80, 'default': 40, 'step': 1},
                'current_lim_tolerance': {'min': 0, 'max': 20, 'default': 2, 'step': 0.1},
                'requested_current_range': {'min': 0, 'max': 100, 'default': 50, 'step': 1}
            }
        }
    
    def log(self, message):
        """Log a message using the logger if available"""
        if self.logger:
            self.logger(message)
        else:
            print(message)
    
    def set_parameter(self, leg_name, motor_name, parameter, value):
        """Set a parameter on the specified motor"""
        if self.simulation_mode:
            self.log(f"Simulation: Setting {leg_name}.{motor_name}.{parameter} = {value}")
            return True
        
        if not self.quadruped or leg_name not in self.quadruped.legs:
            self.log(f"Error: Quadruped not connected or leg '{leg_name}' not found")
            return False
        
        try:
            leg = self.quadruped.legs[leg_name]
            motor = leg.hip_motor if motor_name == 'hip' else leg.knee_motor
            
            # Map parameter names to ODrive attributes with correct paths
            param_mapping = {
                'pos_gain': 'controller.config.pos_gain',
                'vel_gain': 'controller.config.vel_gain', 
                'vel_integrator_gain': 'controller.config.vel_integrator_gain',
                'vel_limit': 'controller.config.vel_limit',
                'trap_traj_vel_limit': 'trap_traj.config.vel_limit',
                'trap_traj_accel_limit': 'trap_traj.config.accel_limit',
                'trap_traj_decel_limit': 'trap_traj.config.decel_limit',
                'current_lim': 'motor.config.current_lim',
                'current_lim_tolerance': 'motor.config.current_lim_tolerance',
                'requested_current_range': 'motor.config.requested_current_range'
            }
            
            if parameter in param_mapping:
                # Navigate to the correct attribute and set value
                attrs = param_mapping[parameter].split('.')
                obj = motor.axis
                for attr in attrs[:-1]:
                    obj = getattr(obj, attr)
                
                # Set the value
                setattr(obj, attrs[-1], float(value))
                
                self.log(f"Set {leg_name}.{motor_name}.{parameter} = {value}")
                return True
            else:
                self.log(f"Error: Unknown parameter '{parameter}'")
                return False
            
        except AttributeError as e:
            self.log(f"Error: ODrive attribute not found for {parameter}: {e}")
            return False
        except Exception as e:
            self.log(f"Error setting {parameter}: {e}")
            return False
    
    def get_parameter(self, leg_name, motor_name, parameter):
        """Get current value of a parameter from ODrive"""
        # First check if parameter exists in our definitions
        for category in self.parameters.values():
            if parameter in category:
                if self.simulation_mode:
                    # Return default values for simulation
                    default_value = category[parameter]['default']
                    self.log(f"Simulation: Getting {leg_name}.{motor_name}.{parameter} = {default_value}")
                    return default_value
                break
        else:
            self.log(f"Error: Unknown parameter '{parameter}'")
            return None
        
        if not self.quadruped or leg_name not in self.quadruped.legs:
            self.log(f"Error: Quadruped not connected or leg {leg_name} not found")
            return None
        
        try:
            leg = self.quadruped.legs[leg_name]
            motor = leg.hip_motor if motor_name == 'hip' else leg.knee_motor
            
            # Map parameter names to ODrive attributes
            param_mapping = {
                'pos_gain': 'controller.config.pos_gain',
                'vel_gain': 'controller.config.vel_gain',
                'vel_integrator_gain': 'controller.config.vel_integrator_gain',
                'vel_limit': 'controller.config.vel_limit',
                'trap_traj_vel_limit': 'trap_traj.config.vel_limit',
                'trap_traj_accel_limit': 'trap_traj.config.accel_limit',
                'trap_traj_decel_limit': 'trap_traj.config.decel_limit',
                'current_lim': 'motor.config.current_lim',
                'current_lim_tolerance': 'motor.config.current_lim_tolerance',
                'requested_current_range': 'motor.config.requested_current_range'
            }
            
            if parameter in param_mapping:
                # Navigate to the correct attribute and get value
                attrs = param_mapping[parameter].split('.')
                obj = motor.axis
                for attr in attrs:
                    obj = getattr(obj, attr)
                value = float(obj)
                self.log(f"Hardware: Getting {leg_name}.{motor_name}.{parameter} = {value}")
                return value
            else:
                self.log(f"Error: Parameter mapping not found for '{parameter}'")
                return None
            
        except AttributeError as e:
            self.log(f"Error: ODrive attribute not found for {parameter}: {e}")
            return None
        except Exception as e:
            self.log(f"Error getting {parameter}: {e}")
            return None
    
    def save_preset(self, name, parameters):
        """Save current parameters as a preset"""
        self.log(f"Saving preset '{name}' with parameters: {parameters}")
        # Implementation would save to config file
    
    def load_preset(self, name):
        """Load a parameter preset"""
        self.log(f"Loading preset '{name}'")
        # Implementation would load from config file
        return {}
    
    def read_all_parameters(self, leg_name, motor_name):
        """Read all current parameter values from ODrive"""
        current_values = {}
        
        # Get all parameter names from all categories
        all_params = []
        for category in self.parameters.values():
            all_params.extend(category.keys())
        
        for param in all_params:
            value = self.get_parameter(leg_name, motor_name, param)
            if value is not None:
                current_values[param] = value
        
        return current_values