# Quadruped Control System

A comprehensive control system for a quadruped robot with ODrive motor controllers, featuring a GUI interface, live plotting, parameter tuning, and 3D visualization.

## Features

- **Configuration File System**: Easy-to-modify YAML configuration for all ODrive and system parameters
- **3D Visualization**: Interactive skeletal model with joint angle and foot position controls
- **Live Plotting**: Real-time plotting of motor currents, positions, velocities, and other parameters
- **Parameter Tuning**: Live modification of PID constants, current limits, and other ODrive parameters
- **Simulation Mode**: Test all functionality without hardware connection
- **Calibration System**: Simultaneous or sequential motor calibration
- **Emergency Controls**: Emergency stop and safety features

## File Structure

```
Quadruped/Saransh/
├── config.yaml                 # Main configuration file
├── quadruped_gui.py            # Main GUI application
├── quadruped_enhanced.py       # Enhanced quadruped control with config support
├── plotting_utils.py           # Plotting and data collection utilities
├── Quadruped_final_legs.py     # Original quadruped control (backward compatibility)
├── Quadruped_leg_test_algo.py  # Inverse kinematics algorithms
├── requirements.txt            # Python package requirements
└── README.md                   # This file
```

## Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Install ODrive (if using real hardware):**
   ```bash
   pip install odrive
   ```
   Follow the [ODrive setup guide](https://docs.odriverobotics.com/) for hardware setup.

3. **Verify installation:**
   ```bash
   python quadruped_gui.py
   ```

## Configuration

### Main Configuration File (config.yaml)

The `config.yaml` file contains all system parameters:

- **System Settings**: Simulation mode, calibration preferences
- **ODrive Mapping**: Serial number to leg assignment
- **Motor Parameters**: PID gains, current limits, encoder settings
- **Leg Geometry**: Link lengths and physical parameters
- **GUI Settings**: Window size, update rates, visualization options

### Key Configuration Sections

#### Motor Configuration
```yaml
motor_config:
  hip:
    pos_gain: 50.0
    vel_gain: 0.05
    current_lim: 20.0
    # ... other parameters
  knee:
    pos_gain: 20.0
    vel_gain: 0.02
    current_lim: 60.0
    # ... other parameters
```

#### ODrive Serial Mapping
```yaml
odrive_mapping:
  "3670314C3333": "front_left"
  "366B315F3333": "front_right"
  "367031553333": "back_left"
  "3669315C3333": "back_right"
```

## Usage

### GUI Application

1. **Start the GUI:**
   ```bash
   python quadruped_gui.py
   ```

2. **Simulation Mode:**
   - Set `simulation_mode: true` in config.yaml
   - Or toggle via the Connection menu
   - Test all features without hardware

3. **Hardware Mode:**
   - Set `simulation_mode: false` in config.yaml
   - Connect ODrives via "Connection" → "Connect to ODrives"

### GUI Tabs

#### 1. Visualization & Pose Control
- **3D Skeleton**: Interactive 3D model of the quadruped
- **Joint Controls**: Sliders for hip and knee angles
- **Foot Position**: X/Y position controls with inverse kinematics
- **Send to Dog**: Apply current pose to physical robot
- **Emergency Stop**: Immediate motor shutdown

#### 2. Motor Control
- Direct motor control interfaces
- Individual leg selection
- Position and velocity commands

#### 3. Live Plotting
- **Real-time Data**: Plot motor currents, positions, velocities
- **Parameter Selection**: Dropdown to choose what to plot
- **Multi-motor Display**: All 8 motors plotted simultaneously
- **Data Export**: Save plot data for analysis

#### 4. Parameter Tuning
- **Live PID Tuning**: Adjust gains in real-time
- **Current Limits**: Modify motor current limits
- **Velocity Limits**: Set speed constraints
- **Presets**: Save and load parameter configurations

#### 5. Calibration & Setup
- **Motor Calibration**: Simultaneous or sequential calibration
- **Home Position**: Set reference positions
- **Motor Testing**: Verify motor functionality

### Command Line Interface

1. **Original Interface:**
   ```bash
   python Quadruped_final_legs.py
   ```

2. **Enhanced Interface with Config:**
   ```bash
   python quadruped_enhanced.py
   ```

## Hardware Setup

### ODrive Configuration

1. **Serial Numbers**: Update `odrive_mapping` in config.yaml with your ODrive serial numbers
2. **Motor Parameters**: Adjust motor configurations for your specific motors
3. **Encoder Settings**: Configure for your encoder types (incremental for hips, hall for knees)

### Physical Connections

- **Front Left**: ODrive 1 (axis0: hip, axis1: knee)
- **Front Right**: ODrive 2 (axis0: hip, axis1: knee)
- **Back Left**: ODrive 3 (axis0: hip, axis1: knee)
- **Back Right**: ODrive 4 (axis0: hip, axis1: knee)

## Safety Features

### Emergency Stop
- **GUI Button**: Red emergency stop button in visualization tab
- **Keyboard**: Ctrl+C to interrupt operations
- **Automatic**: Safety limits in configuration

### Parameter Limits
```yaml
safety:
  max_current: 80.0
  max_velocity: 25.0
  max_position_error: 5.0
  emergency_stop_enabled: true
```

## Troubleshooting

### Common Issues

1. **ODrive Connection Failed**
   - Check USB connections
   - Verify serial numbers in config.yaml
   - Use `odrivetool` to test individual connections

2. **Calibration Errors**
   - Ensure motors can rotate freely
   - Check encoder wiring
   - Verify motor parameters

3. **GUI Not Starting**
   - Install missing dependencies: `pip install -r requirements.txt`
   - Check Python version (3.7+ required)

4. **Plotting Issues**
   - Reduce update rate in config.yaml
   - Check simulation_mode setting
   - Verify data collection parameters

### Debug Mode

Enable verbose output by modifying the configuration:
```yaml
system:
  debug_mode: true
  log_level: "DEBUG"
```

## Development

### Adding New Features

1. **New Parameters**: Add to `motor_config` in config.yaml
2. **GUI Widgets**: Extend the respective tab classes
3. **Plotting**: Add new parameters to `plotting_utils.py`

### Testing

1. **Simulation Mode**: Test without hardware
2. **Unit Tests**: Run individual components
3. **Hardware Testing**: Use with actual ODrives

## Support

- **Documentation**: See inline code comments
- **Configuration**: All parameters documented in config.yaml
- **Issues**: Check troubleshooting section above

## Version History

- **v1.0**: Initial GUI implementation with all major features
- **v0.9**: Enhanced quadruped control with config file support
- **v0.8**: Original command-line interface

## License

This project is part of the ERC Quadruped development effort.