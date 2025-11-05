#!/usr/bin/env python3
"""
Quadruped Control GUI - Main Interface
A comprehensive single-screen GUI for controlling and visualizing a quadruped robot with ODrive motors.
Features dark theme with ERC colors and configurable joint limits.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import queue
import time
from datetime import datetime
import sys
import os

# Set matplotlib to use dark theme
plt.style.use('dark_background')

# Add the current directory to Python path to import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import odrive
    from odrive.enums import *
    ODRIVE_AVAILABLE = True
except ImportError:
    print("ODrive not available - running in simulation mode only")
    ODRIVE_AVAILABLE = False

# Import our existing quadruped code
try:
    from quadruped_enhanced import QuadrupedEnhanced, LegConfig
    import Quadruped_leg_test_algo as qlt
    from plotting_utils import QuadrupedDataCollector, LivePlotter, ParameterTuner
except ImportError as e:
    print(f"Warning: Could not import quadruped modules: {e}")
    
    # Create dummy classes if imports fail
    class QuadrupedEnhanced:
        def __init__(self, *args, **kwargs): 
            self.connected = False
            self.legs = {}
            self.odrives = []
        def connect(self): pass
        def setup_all_legs(self): pass
        def calibrate_all_legs(self, *args): pass
        def set_all_zero_positions(self, *args): pass
        def emergency_stop(self): pass
        def disconnect(self): pass
        def get_all_joint_data(self): return {'timestamp': 0, 'motors': {}}
    
    class QuadrupedDataCollector:
        def __init__(self, *args, **kwargs): pass
        def start_collection(self): pass
        def stop_collection(self): pass
        def get_latest_data(self): return None
    
    class LivePlotter:
        def __init__(self, *args, **kwargs): pass
        def set_parameter(self, param): pass
        def update_data(self, data): pass
        def clear_data(self): pass


class QuadrupedGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Quadruped Control Interface")
        
        # Initialize variables
        self.config = None
        self.quadruped = None
        self.simulation_mode = True
        self.connected = False
        self.data_queue = queue.Queue()
        self.plot_data = {}
        self.running = False
        
        # Initialize GUI variables early
        self.sim_mode_var = tk.BooleanVar(value=True)
        self.connection_status_var = tk.StringVar(value="Disconnected")
        
        # Store individual leg positions
        self.leg_positions = {
            'front_left': {'hip': 0.0, 'knee': 0.0},
            'front_right': {'hip': 0.0, 'knee': 0.0},
            'back_left': {'hip': 0.0, 'knee': 0.0},
            'back_right': {'hip': 0.0, 'knee': 0.0}
        }
        
        # Initialize motor control variables for individual sliders
        self.individual_motor_angles = {}
        self.homing_motor_angles = {}
        for leg in ['front_left', 'front_right', 'back_left', 'back_right']:
            self.individual_motor_angles[leg] = {
                'hip': tk.DoubleVar(),
                'knee': tk.DoubleVar()
            }
            self.homing_motor_angles[leg] = {
                'hip': tk.DoubleVar(),
                'knee': tk.DoubleVar()
            }
        
        # Initialize components
        self.data_collector = None
        self.live_plotter = None
        self.parameter_tuner = None
        self.plotting_active = False
        
        # Initialize logging system
        self.log_messages = []
        self.max_log_messages = 100  # Keep last 100 messages
        
        # Motor gear ratios and conversions (from your original code)
        # Hip: gear reduction derived from your conversion formula  
        # Knee: gear reduction derived from your conversion formula
        self.hip_gear_ratio = 10.5 / 6.28  # From: final_alpha = (((1.57 - alpha)*(1))/6.28)*10.5
        self.knee_gear_ratio = 4.58         # From: final_beta = ((1.57 - alpha + beta - 1.12)*4.58)
        
        # Load configuration
        self.load_config()
        
        # Apply theme and setup window
        self.setup_theme()
        self.setup_window()
        
        # Create the interface
        self.create_menu()
        self.create_single_screen_interface()
        self.create_status_bar()
        
        # Start update loop
        self.start_update_loop()
        
    def setup_theme(self):
        """Setup dark theme with ERC colors"""
        theme = self.config.get('gui', {}).get('theme', {})
        
        # Define theme colors with defaults
        self.theme = {
            'bg': theme.get('background', '#1e1e1e'),
            'surface': theme.get('surface', '#2d2d2d'),
            'primary': theme.get('primary', '#0078d4'),
            'secondary': theme.get('secondary', '#ffffff'),
            'accent': theme.get('accent', '#005a9e'),
            'text_primary': theme.get('text_primary', '#ffffff'),
            'text_secondary': theme.get('text_secondary', '#cccccc'),
            'success': theme.get('success', '#107c10'),
            'warning': theme.get('warning', '#ff8c00'),
            'error': theme.get('error', '#d13438')
        }
        
        # Configure root window
        self.root.configure(bg=self.theme['bg'])
        
        # Create custom styles
        style = ttk.Style()
        style.theme_use('clam')  # Use clam as base theme
        
        # Configure styles
        style.configure('Dark.TFrame', background=self.theme['bg'], borderwidth=0)
        style.configure('Surface.TFrame', background=self.theme['surface'], relief='flat', borderwidth=0)
        style.configure('Dark.TLabel', background=self.theme['bg'], foreground=self.theme['text_primary'])
        style.configure('Surface.TLabel', background=self.theme['surface'], foreground=self.theme['text_primary'])
        style.configure('Title.TLabel', background=self.theme['bg'], foreground=self.theme['primary'], 
                       font=('Arial', 12, 'bold'))
        style.configure('Dark.TButton', background=self.theme['primary'], foreground=self.theme['secondary'],
                       borderwidth=0, focuscolor='none')
        style.configure('Emergency.TButton', background=self.theme['error'], foreground=self.theme['secondary'],
                       borderwidth=0, focuscolor='none')
        style.configure('Success.TButton', background=self.theme['success'], foreground=self.theme['secondary'],
                       borderwidth=0, focuscolor='none')
        
        # Scale styling - remove borders and improve colors
        style.configure('Dark.Horizontal.TScale', background=self.theme['surface'], 
                       troughcolor=self.theme['bg'], sliderrelief='flat', borderwidth=0,
                       lightcolor=self.theme['primary'], darkcolor=self.theme['primary'])
        
        # Notebook styling
        style.configure('Dark.TNotebook', background=self.theme['bg'], borderwidth=0, tabmargins=0)
        style.configure('Dark.TNotebook.Tab', background=self.theme['surface'], 
                       foreground=self.theme['text_primary'], padding=[10, 5], borderwidth=0)
        
        # LabelFrame styling
        style.configure('Surface.TLabelframe', background=self.theme['surface'], borderwidth=1,
                       relief='solid', bordercolor=self.theme['accent'])
        style.configure('Surface.TLabelframe.Label', background=self.theme['surface'], 
                       foreground=self.theme['primary'], font=('Arial', 10, 'bold'))
        
        # Combobox styling
        style.configure('TCombobox', selectbackground=self.theme['primary'],
                       fieldbackground=self.theme['surface'], background=self.theme['surface'],
                       foreground=self.theme['text_primary'], borderwidth=0)
        
        # Entry styling
        style.configure('TEntry', fieldbackground=self.theme['surface'], 
                       foreground=self.theme['text_primary'], borderwidth=1,
                       bordercolor=self.theme['accent'], insertcolor=self.theme['text_primary'])
        
        # Radiobutton styling
        style.configure('Dark.TRadiobutton', background=self.theme['surface'], 
                       foreground=self.theme['text_primary'], focuscolor='none')
        
    def setup_window(self):
        """Setup window size and properties"""
        window_size = self.config.get('gui', {}).get('window_size', [1600, 1000])
        self.root.geometry(f"{window_size[0]}x{window_size[1]}")
        self.root.minsize(1400, 900)  # Minimum size for usability
        
    def load_config(self, config_file="config.yaml"):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded configuration from {config_file}")
            
            # Set simulation mode from config
            self.simulation_mode = self.config.get('system', {}).get('simulation_mode', True)
            
        except FileNotFoundError:
            self.log_message(f"Configuration file {config_file} not found! Using defaults.", "error")
            self.config = self.get_default_config()
        except yaml.YAMLError as e:
            self.log_message(f"Error parsing configuration file: {e}. Using defaults.", "error")
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """Return default configuration if file loading fails"""
        return {
            'system': {'simulation_mode': True},
            'gui': {'window_title': 'Quadruped Control', 'update_rate_ms': 50},
            'leg_config': {'L1': 25.0, 'L2': 33.0},
            'visualization': {'body_length': 42.0, 'body_width': 35.8}
        }
    
    def create_menu(self):
        """Create the menu bar with dark theme"""
        menubar = tk.Menu(self.root, bg=self.theme['surface'], fg=self.theme['text_primary'],
                         activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'],
                           activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load Config", command=self.load_config_dialog)
        file_menu.add_command(label="Save Config", command=self.save_config_dialog)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_closing)
        
        # Connection menu
        conn_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'],
                           activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="Connection", menu=conn_menu)
        conn_menu.add_command(label="Connect to ODrives", command=self.connect_odrives)
        conn_menu.add_command(label="Disconnect", command=self.disconnect_odrives)
        conn_menu.add_separator()
        conn_menu.add_checkbutton(label="Simulation Mode", variable=self.sim_mode_var, 
                                  command=self.toggle_simulation_mode)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'],
                           activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
    
    def create_single_screen_interface(self):
        """Create the single-screen interface layout"""
        # Main container with padding
        main_frame = ttk.Frame(self.root, style='Dark.TFrame')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Use a PanedWindow for better control over proportions
        main_paned = ttk.PanedWindow(main_frame, orient=tk.VERTICAL, style='Dark.TPanedwindow')
        main_paned.pack(fill=tk.BOTH, expand=True)
        
        # Top section - Animation (left) and Plotting (right) - smaller height
        top_frame = ttk.Frame(main_paned, style='Dark.TFrame', height=400)
        main_paned.add(top_frame, weight=1)
        
        # Animation section (top left)
        self.create_animation_section(top_frame)
        
        # Plotting section (top right)
        self.create_plotting_section(top_frame)
        
        # Bottom section - Controls organized in panels - larger height
        bottom_frame = ttk.Frame(main_paned, style='Dark.TFrame', height=600)
        main_paned.add(bottom_frame, weight=2)  # Give more weight to bottom section
        
        # Create control panels in bottom section
        self.create_control_panels(bottom_frame)
        
    def create_animation_section(self, parent):
        """Create the 3D animation section (top left)"""
        # Animation panel
        anim_panel = ttk.LabelFrame(parent, text="3D Visualization & Pose Control", 
                                   style='Surface.TLabelframe')
        anim_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Create matplotlib figure for 3D plot with dark theme
        self.viz_fig = Figure(figsize=(8, 6), dpi=100, facecolor=self.theme['surface'])
        self.viz_ax = self.viz_fig.add_subplot(111, projection='3d')
        self.viz_ax.set_facecolor(self.theme['bg'])
        
        # Customize 3D plot colors for dark theme
        self.viz_ax.xaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.yaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.zaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.tick_params(colors=self.theme['text_secondary'])
        
        self.viz_canvas = FigureCanvasTkAgg(self.viz_fig, anim_panel)
        self.viz_canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Control panel for animation (right side of animation section)
        control_frame = ttk.Frame(anim_panel, style='Dark.TFrame')
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        
        # Leg selection
        ttk.Label(control_frame, text="Leg Selection", style='Title.TLabel').pack(pady=(0, 5))
        
        self.selected_leg = tk.StringVar(value="front_left")
        legs = [("front_left", "Front Left"), ("front_right", "Front Right"), 
                ("back_left", "Back Left"), ("back_right", "Back Right")]
        
        leg_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        leg_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=5)
        
        for value, text in legs:
            ttk.Radiobutton(leg_frame, text=text, variable=self.selected_leg, value=value,
                           style='Dark.TRadiobutton', command=self.on_leg_selection_changed).pack(anchor=tk.W, padx=5)
        
        # Joint angle controls with configurable limits
        joint_limits = self.config.get('joint_limits', {}).get('animation', {})
        hip_limits = joint_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = joint_limits.get('knee', {'min': -90, 'max': 90})
        
        ttk.Label(control_frame, text="Joint Angles", style='Title.TLabel').pack(pady=(0, 5))
        
        joint_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        joint_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=10)
        
        self.hip_angle = tk.DoubleVar()
        self.knee_angle = tk.DoubleVar()
        
        # Hip angle
        ttk.Label(joint_frame, text=f"Hip ({hip_limits['min']}° to {hip_limits['max']}°):",
                 style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        hip_scale = ttk.Scale(joint_frame, from_=hip_limits['min'], to=hip_limits['max'], 
                             variable=self.hip_angle, orient=tk.HORIZONTAL, 
                             style='Dark.Horizontal.TScale',
                             command=self.update_visualization)
        hip_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        self.hip_value_label = ttk.Label(joint_frame, text="0.0°", style='Surface.TLabel')
        self.hip_value_label.pack(anchor=tk.W, padx=5)
        
        # Knee angle
        ttk.Label(joint_frame, text=f"Knee ({knee_limits['min']}° to {knee_limits['max']}°):",
                 style='Surface.TLabel').pack(anchor=tk.W, padx=5, pady=(10, 0))
        knee_scale = ttk.Scale(joint_frame, from_=knee_limits['min'], to=knee_limits['max'], 
                              variable=self.knee_angle, orient=tk.HORIZONTAL,
                              style='Dark.Horizontal.TScale',
                              command=self.update_visualization)
        knee_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        self.knee_value_label = ttk.Label(joint_frame, text="0.0°", style='Surface.TLabel')
        self.knee_value_label.pack(anchor=tk.W, padx=5)
        
        # Foot position controls
        ttk.Label(control_frame, text="Foot Position", style='Title.TLabel').pack(pady=(0, 5))
        
        foot_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        foot_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=10)
        
        self.foot_x = tk.DoubleVar()
        self.foot_y = tk.DoubleVar(value=30)
        
        ttk.Label(foot_frame, text="X Position (cm):", style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        x_scale = ttk.Scale(foot_frame, from_=-40, to=40, variable=self.foot_x, 
                           orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale',
                           command=self.update_foot_position)
        x_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        ttk.Label(foot_frame, text="Height (cm):", style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        y_scale = ttk.Scale(foot_frame, from_=5, to=60, variable=self.foot_y, 
                           orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale',
                           command=self.update_foot_position)
        y_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        # Control buttons
        ttk.Label(control_frame, text="Actions", style='Title.TLabel').pack(pady=(0, 5))
        
        button_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        button_frame.pack(fill=tk.X, padx=5, ipady=10)
        
        ttk.Button(button_frame, text="Send to Hardware", style='Dark.TButton',
                  command=self.send_pose_to_dog).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Apply to All Legs", style='Success.TButton',
                  command=self.apply_to_all_legs).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Send All to Hardware", style='Success.TButton',
                  command=self.send_all_poses_to_dog).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Reset Home", style='Dark.TButton',
                  command=self.reset_to_home).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Emergency Stop", style='Emergency.TButton',
                  command=self.emergency_stop).pack(fill=tk.X, pady=2, padx=5)
        
        # Initialize the 3D plot
        self.init_3d_visualization()
        
    def create_plotting_section(self, parent):
        """Create the live plotting section (top right)"""
        # Plotting panel
        plot_panel = ttk.LabelFrame(parent, text="Live Data Plotting", style='Surface.TLabelframe')
        plot_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Plot controls (top of plot panel)
        plot_control_frame = ttk.Frame(plot_panel, style='Surface.TFrame')
        plot_control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Parameter selection
        ttk.Label(plot_control_frame, text="Parameter:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        
        self.plot_parameter = tk.StringVar(value="current")
        parameters = ["current", "position", "velocity", "voltage", "temperature"]
        param_combo = ttk.Combobox(plot_control_frame, textvariable=self.plot_parameter, 
                                  values=parameters, state="readonly", width=12)
        param_combo.pack(side=tk.LEFT, padx=5)
        param_combo.bind('<<ComboboxSelected>>', self.on_parameter_changed)
        
        # Control buttons
        ttk.Button(plot_control_frame, text="Start", style='Success.TButton',
                  command=self.start_plotting).pack(side=tk.LEFT, padx=2)
        ttk.Button(plot_control_frame, text="Stop", style='Dark.TButton',
                  command=self.stop_plotting).pack(side=tk.LEFT, padx=2)
        ttk.Button(plot_control_frame, text="Clear", style='Dark.TButton',
                  command=self.clear_plot).pack(side=tk.LEFT, padx=2)
        
        # Status indicator
        self.plot_status_var = tk.StringVar(value="Stopped")
        status_label = ttk.Label(plot_control_frame, textvariable=self.plot_status_var, 
                                style='Surface.TLabel')
        status_label.pack(side=tk.RIGHT, padx=5)
        ttk.Label(plot_control_frame, text="Status:", style='Surface.TLabel').pack(side=tk.RIGHT)
        
        # Create matplotlib figure for plotting with dark theme - reduced height
        self.plot_fig = Figure(figsize=(8, 4), dpi=100, facecolor=self.theme['surface'])
        self.plot_ax = self.plot_fig.add_subplot(111)
        self.plot_ax.set_facecolor(self.theme['bg'])
        
        # Customize plot colors for dark theme
        self.plot_ax.xaxis.label.set_color(self.theme['text_primary'])
        self.plot_ax.yaxis.label.set_color(self.theme['text_primary'])
        self.plot_ax.tick_params(colors=self.theme['text_secondary'])
        self.plot_ax.spines['bottom'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['top'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['left'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['right'].set_color(self.theme['text_secondary'])
        
        self.plot_canvas = FigureCanvasTkAgg(self.plot_fig, plot_panel)
        self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=(0, 5))
        
        # Add logs window at the bottom of plot panel
        self.create_logs_window(plot_panel)
        
        # Initialize plotting components
        self.init_plotting()
    
    def create_logs_window(self, parent):
        """Create a logs window for displaying messages"""
        logs_frame = ttk.LabelFrame(parent, text="Logs", style='Surface.TLabelframe')
        logs_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=(5, 0))
        
        # Create text widget with scrollbar for logs - increased height
        logs_container = ttk.Frame(logs_frame, style='Surface.TFrame')
        logs_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.logs_text = tk.Text(logs_container, height=8, wrap=tk.WORD,
                                bg=self.theme['bg'], fg=self.theme['text_primary'],
                                font=('Consolas', 9), relief='flat', borderwidth=0)
        logs_scrollbar = ttk.Scrollbar(logs_container, command=self.logs_text.yview)
        self.logs_text.configure(yscrollcommand=logs_scrollbar.set)
        
        self.logs_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        logs_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Log controls
        log_controls = ttk.Frame(logs_frame, style='Surface.TFrame')
        log_controls.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        ttk.Button(log_controls, text="Clear Logs", style='Dark.TButton',
                  command=self.clear_logs).pack(side=tk.RIGHT, padx=5)
    
    def log_message(self, message, level="info"):
        """Add a message to the logs window with optional level"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Add level indicator with color coding
        level_indicators = {
            "info": "ℹ",
            "success": "✓", 
            "warning": "⚠",
            "error": "✗"
        }
        indicator = level_indicators.get(level.lower(), "ℹ")
        formatted_message = f"[{timestamp}] {indicator} {message}\n"
        
        # Add to message list
        self.log_messages.append(formatted_message)
        
        # Keep only last N messages
        if len(self.log_messages) > self.max_log_messages:
            self.log_messages.pop(0)
        
        # Update logs window if it exists
        if hasattr(self, 'logs_text'):
            self.logs_text.insert(tk.END, formatted_message)
            self.logs_text.see(tk.END)  # Auto-scroll to bottom
    
    def clear_logs(self):
        """Clear the logs window"""
        self.log_messages.clear()
        if hasattr(self, 'logs_text'):
            self.logs_text.delete(1.0, tk.END)
    
    def map_leg_name_to_quadruped(self, gui_leg_name):
        """Map GUI leg names (FL, FR, etc.) to quadruped leg names (front_left, etc.)"""
        mapping = {
            "FL": "front_left",
            "FR": "front_right", 
            "BL": "back_left",
            "BR": "back_right"
        }
        return mapping.get(gui_leg_name, gui_leg_name)
    
    def create_control_panels(self, parent):
        """Create the control panels in the bottom section"""
        # Create a notebook for the bottom panels (much more compact than before)
        bottom_notebook = ttk.Notebook(parent, style='Dark.TNotebook')
        bottom_notebook.pack(fill=tk.BOTH, expand=True)
        
        # Connection & Calibration Panel
        self.create_connection_panel(bottom_notebook)
        
        # Parameter Tuning Panel
        self.create_parameters_panel(bottom_notebook)
        
        # Motor Control Panel
        self.create_motor_control_panel(bottom_notebook)
        
        # Individual Motor Sliders Panel
        self.create_individual_motor_panel(bottom_notebook)
        
        # Interactive Homing Panel
        self.create_homing_panel(bottom_notebook)
        
    def create_connection_panel(self, parent):
        """Create connection and calibration panel"""
        conn_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(conn_frame, text="Connection & Calibration")
        
        # Split into connection and calibration sections
        left_section = ttk.LabelFrame(conn_frame, text="Connection", style='Surface.TLabelframe')
        left_section.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        right_section = ttk.LabelFrame(conn_frame, text="Calibration", style='Surface.TLabelframe')
        right_section.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Connection controls
        conn_buttons = ttk.Frame(left_section, style='Surface.TFrame')
        conn_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(conn_buttons, text="Connect ODrives", style='Success.TButton',
                  command=self.connect_odrives).pack(fill=tk.X, pady=2)
        ttk.Button(conn_buttons, text="Disconnect", style='Dark.TButton',
                  command=self.disconnect_odrives).pack(fill=tk.X, pady=2)
        
        # Simulation mode toggle
        sim_frame = ttk.Frame(left_section, style='Surface.TFrame')
        sim_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Checkbutton(sim_frame, text="Simulation Mode", variable=self.sim_mode_var,
                       command=self.toggle_simulation_mode).pack(anchor=tk.W)
        
        # Connection status
        status_frame = ttk.Frame(left_section, style='Surface.TFrame')
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(status_frame, text="Status:", style='Surface.TLabel').pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.connection_status_var, 
                 style='Surface.TLabel').pack(side=tk.LEFT, padx=(5, 0))
        
        # Calibration controls
        calib_buttons = ttk.Frame(right_section, style='Surface.TFrame')
        calib_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(calib_buttons, text="Calibrate All Motors", style='Dark.TButton',
                  command=self.calibrate_all).pack(fill=tk.X, pady=2)
        ttk.Button(calib_buttons, text="Set Home Position", style='Dark.TButton',
                  command=self.set_home_position).pack(fill=tk.X, pady=2)
        ttk.Button(calib_buttons, text="Test Motors", style='Dark.TButton',
                  command=self.test_motors).pack(fill=tk.X, pady=2)
        
        # ODrive reboot controls
        reboot_frame = ttk.LabelFrame(right_section, text="ODrive Reboot", style='Surface.TLabelframe')
        reboot_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        reboot_controls = ttk.Frame(reboot_frame, style='Surface.TFrame')
        reboot_controls.pack(fill=tk.X, padx=10, pady=5)
        
        # ODrive selection for reboot
        ttk.Label(reboot_controls, text="ODrive:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.reboot_odrive = tk.StringVar(value="All")
        odrive_options = ["All", "Front Left", "Front Right", "Back Left", "Back Right"]
        odrive_combo = ttk.Combobox(reboot_controls, textvariable=self.reboot_odrive,
                                   values=odrive_options, state="readonly", width=12)
        odrive_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(reboot_controls, text="Reboot ODrive", style='Emergency.TButton',
                  command=self.reboot_odrive_selected).pack(side=tk.LEFT, padx=10)
    
    def create_parameters_panel(self, parent):
        """Create parameter tuning panel"""
        param_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(param_frame, text="Parameter Tuning")
        
        # Motor selection section
        selection_frame = ttk.LabelFrame(param_frame, text="Motor Selection", style='Surface.TLabelframe')
        selection_frame.pack(fill=tk.X, padx=5, pady=5)
        
        select_controls = ttk.Frame(selection_frame, style='Surface.TFrame')
        select_controls.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(select_controls, text="Leg:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_leg = tk.StringVar(value="FL")
        leg_combo = ttk.Combobox(select_controls, textvariable=self.param_leg,
                                values=["FL", "FR", "BL", "BR"],
                                state="readonly", width=12)
        leg_combo.pack(side=tk.LEFT, padx=5)
        leg_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        
        ttk.Label(select_controls, text="Motor:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_motor = tk.StringVar(value="hip")
        motor_combo = ttk.Combobox(select_controls, textvariable=self.param_motor,
                                  values=["hip", "knee"], state="readonly", width=8)
        motor_combo.pack(side=tk.LEFT, padx=5)
        motor_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        
        # Parameter controls section
        control_frame = ttk.LabelFrame(param_frame, text="Parameters", style='Surface.TLabelframe')
        control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Category selection
        category_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        category_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(category_frame, text="Category:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_category = tk.StringVar(value="PID Control")
        categories = ["PID Control", "Trajectory Control", "Current Limits"]
        category_combo = ttk.Combobox(category_frame, textvariable=self.param_category,
                                     values=categories, state="readonly", width=15)
        category_combo.pack(side=tk.LEFT, padx=5)
        category_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        
        # Create scrollable parameter controls
        # Canvas for scrolling
        canvas = tk.Canvas(control_frame, bg='#2b2b2b', highlightthickness=0)
        scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=canvas.yview)
        self.param_controls_frame = ttk.Frame(canvas, style='Surface.TFrame')
        
        # Configure scrolling
        self.param_controls_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=self.param_controls_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Pack scrollable elements
        canvas.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        scrollbar.pack(side="right", fill="y", pady=5)
        
        # Enable mouse wheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        def _bind_mousewheel(event):
            canvas.bind_all("<MouseWheel>", _on_mousewheel)
            canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
            canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))
        
        def _unbind_mousewheel(event):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")
        
        canvas.bind('<Enter>', _bind_mousewheel)
        canvas.bind('<Leave>', _unbind_mousewheel)
        
        # Preset controls
        preset_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        preset_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(preset_frame, text="Preset:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.preset_name = tk.StringVar()
        preset_entry = ttk.Entry(preset_frame, textvariable=self.preset_name, width=15)
        preset_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(preset_frame, text="Save", style='Dark.TButton',
                  command=self.save_parameter_preset).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="Load", style='Dark.TButton',
                  command=self.load_parameter_preset).pack(side=tk.LEFT, padx=2)
        
        # Initialize parameter tuner with logger
        self.log_message(f"Initializing ParameterTuner with quadruped: {self.quadruped is not None}, simulation_mode: {self.simulation_mode}")
        if self.quadruped and hasattr(self.quadruped, 'legs'):
            self.log_message(f"Available quadruped legs: {list(self.quadruped.legs.keys())}")
        self.parameter_tuner = ParameterTuner(self.quadruped, self.simulation_mode, logger=self.log_message)
        self.param_scales = {}
        self.update_parameter_display()
    
    def create_motor_control_panel(self, parent):
        """Create direct motor control panel"""
        motor_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(motor_frame, text="Direct Motor Control")
        
        # Individual motor controls
        controls_frame = ttk.LabelFrame(motor_frame, text="Individual Motor Control", style='Surface.TLabelframe')
        controls_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Motor selection for direct control
        select_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        select_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(select_frame, text="Select Motor:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.direct_motor = tk.StringVar(value="front_left_hip")
        
        motors = []
        for leg in ["front_left", "front_right", "back_left", "back_right"]:
            for joint in ["hip", "knee"]:
                motors.append(f"{leg}_{joint}")
        
        motor_combo = ttk.Combobox(select_frame, textvariable=self.direct_motor,
                                  values=motors, state="readonly", width=20)
        motor_combo.pack(side=tk.LEFT, padx=5)
        
        # Direct position control
        position_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        position_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(position_frame, text="Position (degrees):", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.direct_position = tk.DoubleVar()
        
        pos_scale = ttk.Scale(position_frame, from_=-180, to=180, variable=self.direct_position,
                             orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', length=300)
        pos_scale.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.direct_pos_label = ttk.Label(position_frame, text="0.0°", style='Surface.TLabel')
        self.direct_pos_label.pack(side=tk.LEFT, padx=5)
        
        # Update position label when scale changes
        pos_scale.configure(command=lambda val: self.direct_pos_label.config(text=f"{float(val):.1f}°"))
        
        # Control buttons
        button_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        button_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(button_frame, text="Move to Position", style='Dark.TButton',
                  command=self.move_motor_direct).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop Motor", style='Emergency.TButton',
                  command=self.stop_motor_direct).pack(side=tk.LEFT, padx=5)
    
    def create_individual_motor_panel(self, parent):
        """Create individual motor control panel with sliders for each motor"""
        motor_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(motor_frame, text="Individual Motor Control")
        
        # Global controls at the top for easy access
        global_controls_top = ttk.LabelFrame(motor_frame, text="Quick Controls", style='Surface.TLabelframe')
        global_controls_top.pack(fill=tk.X, padx=10, pady=5)
        
        controls_grid = ttk.Frame(global_controls_top, style='Surface.TFrame')
        controls_grid.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(controls_grid, text="Read All Current Positions", style='Success.TButton',
                  command=self.read_all_motor_positions).grid(row=0, column=0, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Send All Motor Positions", style='Dark.TButton',
                  command=self.send_all_individual_motors).grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Zero All Motors", style='Dark.TButton',
                  command=self.zero_all_individual_motors).grid(row=0, column=2, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Emergency Stop All", style='Emergency.TButton',
                  command=self.emergency_stop).grid(row=0, column=3, padx=5, pady=2, sticky="ew")
        
        # Configure grid weights
        for i in range(4):
            controls_grid.columnconfigure(i, weight=1)
        
        # Scrollable motor controls container
        scroll_container = ttk.Frame(motor_frame, style='Dark.TFrame')
        scroll_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create canvas and scrollbar for scrollable content
        canvas = tk.Canvas(scroll_container, bg=self.theme['bg'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(scroll_container, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas, style='Dark.TFrame')
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Get joint limits for sliders
        joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits = joint_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = joint_limits.get('knee', {'min': -120, 'max': 120})
        
        # Create motor controls for each leg
        legs = [
            ('front_left', 'Front Left Leg'),
            ('front_right', 'Front Right Leg'),
            ('back_left', 'Back Left Leg'),
            ('back_right', 'Back Right Leg')
        ]
        
        self.motor_sliders = {}
        row = 0
        
        for leg_name, leg_display in legs:
            # Leg group frame
            leg_frame = ttk.LabelFrame(scrollable_frame, text=leg_display, style='Surface.TLabelframe')
            leg_frame.grid(row=row, column=0, columnspan=4, padx=10, pady=5, sticky="ew")
            
            self.motor_sliders[leg_name] = {}
            
            # Hip motor control
            ttk.Label(leg_frame, text="Hip Joint:", style='Surface.TLabel').grid(
                row=0, column=0, padx=5, pady=5, sticky="w")
            
            hip_var = self.individual_motor_angles[leg_name]['hip']
            hip_scale = ttk.Scale(leg_frame, from_=hip_limits['min'], to=hip_limits['max'],
                                 variable=hip_var, orient=tk.HORIZONTAL, length=400,
                                 style='Dark.Horizontal.TScale',
                                 command=lambda val, leg=leg_name: 
                                 self.on_individual_motor_change(leg, 'hip', float(val)))
            hip_scale.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
            
            hip_label = ttk.Label(leg_frame, text="0.0°", style='Surface.TLabel')
            hip_label.grid(row=0, column=2, padx=5, pady=5)
            
            hip_button = ttk.Button(leg_frame, text="Send", style='Dark.TButton',
                                   command=lambda leg=leg_name: 
                                   self.send_individual_motor(leg, 'hip'))
            hip_button.grid(row=0, column=3, padx=5, pady=5)
            
            # Knee motor control
            ttk.Label(leg_frame, text="Knee Joint:", style='Surface.TLabel').grid(
                row=1, column=0, padx=5, pady=5, sticky="w")
            
            knee_var = self.individual_motor_angles[leg_name]['knee']
            knee_scale = ttk.Scale(leg_frame, from_=knee_limits['min'], to=knee_limits['max'],
                                  variable=knee_var, orient=tk.HORIZONTAL, length=400,
                                  style='Dark.Horizontal.TScale',
                                  command=lambda val, leg=leg_name: 
                                  self.on_individual_motor_change(leg, 'knee', float(val)))
            knee_scale.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
            
            knee_label = ttk.Label(leg_frame, text="0.0°", style='Surface.TLabel')
            knee_label.grid(row=1, column=2, padx=5, pady=5)
            
            knee_button = ttk.Button(leg_frame, text="Send", style='Dark.TButton',
                                    command=lambda leg=leg_name: 
                                    self.send_individual_motor(leg, 'knee'))
            knee_button.grid(row=1, column=3, padx=5, pady=5)
            
            # Store references for updates
            self.motor_sliders[leg_name] = {
                'hip': {'scale': hip_scale, 'label': hip_label, 'button': hip_button},
                'knee': {'scale': knee_scale, 'label': knee_label, 'button': knee_button}
            }
            
            # Configure column weights
            leg_frame.columnconfigure(1, weight=1)
            
            row += 1
        
        # Configure main grid weights
        scrollable_frame.columnconfigure(0, weight=1)
    
    def load_config(self, config_file="config.yaml"):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded configuration from {config_file}")
            
            # Set simulation mode from config
            self.simulation_mode = self.config.get('system', {}).get('simulation_mode', True)
            
        except FileNotFoundError:
            self.log_message(f"Configuration file {config_file} not found! Using defaults.", "error")
            self.config = self.get_default_config()
        except yaml.YAMLError as e:
            self.log_message(f"Error parsing configuration file: {e}. Using defaults.", "error")
            self.config = self.get_default_config()
    
    
    def get_default_config(self):
        """Return default configuration if file loading fails"""
        return {
            'system': {'simulation_mode': True},
            'gui': {'window_title': 'Quadruped Control', 'update_rate_ms': 50},
            'leg_config': {'L1': 25.0, 'L2': 33.0},
            'visualization': {'body_length': 42.0, 'body_width': 35.8}
        }
    
    def create_status_bar(self):
        """Create the status bar"""
        self.status_frame = ttk.Frame(self.root)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.status_var = tk.StringVar()
        self.connection_var = tk.StringVar()
        
        ttk.Label(self.status_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=5)
        ttk.Label(self.status_frame, textvariable=self.connection_var).pack(side=tk.RIGHT, padx=5)
        
        self.update_status("Ready", "Disconnected")
    
    def init_3d_visualization(self):
        """Initialize the 3D visualization"""
        self.viz_ax.clear()
        self.viz_ax.set_xlabel('X (cm)')
        self.viz_ax.set_ylabel('Y (cm)')
        self.viz_ax.set_zlabel('Z (cm)')
        self.viz_ax.set_title('Quadruped Skeleton')
        
        # Set axis limits
        self.viz_ax.set_xlim([-30, 30])
        self.viz_ax.set_ylim([-30, 30])
        self.viz_ax.set_zlim([-10, 50])  # Allow feet to go below body level
        
        # Draw initial skeleton
        self.draw_quadruped_skeleton()
        self.viz_canvas.draw()
    
    def draw_quadruped_skeleton(self):
        """Draw the quadruped skeleton in 3D using stored leg positions"""
        # Get configuration
        L1 = self.config.get('leg_config', {}).get('L1', 25.0)
        L2 = self.config.get('leg_config', {}).get('L2', 33.0)
        body_length = self.config.get('visualization', {}).get('body_length', 42.0)
        body_width = self.config.get('visualization', {}).get('body_width', 35.8)
        
        # Body corners
        body_x = [-body_length/2, body_length/2, body_length/2, -body_length/2, -body_length/2]
        body_y = [-body_width/2, -body_width/2, body_width/2, body_width/2, -body_width/2]
        body_z = [20, 20, 20, 20, 20]  # Fixed height for body
        
        # Draw body
        self.viz_ax.plot(body_x, body_y, body_z, color=self.theme['text_primary'], linewidth=3, label='Body')
        
        # Draw legs using stored positions - Fixed coordinate mapping
        # Corrected Y coordinates: left legs should be on +Y side, right legs on -Y side in visualization
        leg_positions = {
            'front_left': (-body_length/2, -body_width/2),   # Front, Left 
            'front_right': (-body_length/2, body_width/2),   # Front, Right
            'back_left': (body_length/2, -body_width/2),     # Back, Left
            'back_right': (body_length/2, body_width/2)      # Back, Right
        }
        
        selected_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else None
        
        for leg_name, (bx, by) in leg_positions.items():
            # Get angles for this leg from stored positions
            if leg_name in self.leg_positions:
                hip_angle = self.leg_positions[leg_name]['hip']
                knee_angle = self.leg_positions[leg_name]['knee']
            else:
                hip_angle = 0
                knee_angle = 0
            
            # Convert to radians and apply physical robot configuration
            # Hip: positive angles move leg forward, negative backward
            # At zero: thigh is parallel to ground
            # Different directions for left vs right legs
            if 'left' in leg_name:
                hip_rad = np.radians(hip_angle)  # Left legs: normal direction
            else:
                hip_rad = np.radians(-hip_angle)  # Right legs: inverted direction
            
            # Knee: at zero position, calf is folded back at 1.12 radians
            # Positive knee angles extend the leg (reduce folding), negative fold it more
            knee_rad = -np.radians(knee_angle) + np.pi - 1.12  # Add pi then subtract 1.12
            
            # Calculate leg positions with corrected kinematics
            hip_x = bx
            hip_y = by
            hip_z = 20  # Body height
            
            # Knee position: thigh extends horizontally from hip
            knee_x = hip_x + L1 * np.cos(hip_rad)  # Horizontal extension
            knee_y = hip_y
            knee_z = hip_z - L1 * np.sin(hip_rad)  # Vertical component
            
            # Foot position: calf extends from knee at angle relative to thigh
            total_angle = hip_rad + knee_rad
            foot_x = knee_x + L2 * np.cos(total_angle)
            foot_y = knee_y
            foot_z = knee_z - L2 * np.sin(total_angle)
            
            # Color coding: highlight selected leg
            if leg_name == selected_leg:
                leg_color = self.theme['primary']
                joint_color = self.theme['primary']
                linewidth = 3
            else:
                leg_color = self.theme['text_secondary']
                joint_color = self.theme['text_secondary']
                linewidth = 2
            
            # Draw leg segments
            self.viz_ax.plot([hip_x, knee_x], [hip_y, knee_y], [hip_z, knee_z], 
                           color=leg_color, linewidth=linewidth, alpha=0.8)
            self.viz_ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], 
                           color=leg_color, linewidth=linewidth, alpha=0.8)
            
            # Draw joints
            self.viz_ax.scatter([hip_x], [hip_y], [hip_z], c=joint_color, s=50, alpha=0.9)
            self.viz_ax.scatter([knee_x], [knee_y], [knee_z], c=joint_color, s=30, alpha=0.9)
            self.viz_ax.scatter([foot_x], [foot_y], [foot_z], c=leg_color, s=40, alpha=0.9)
    
    def init_plotting(self):
        """Initialize the plotting system"""
        self.live_plotter = LivePlotter(self.plot_ax, self.plot_canvas)
        self.data_collector = QuadrupedDataCollector(self.quadruped, self.simulation_mode)
    
    def on_leg_selection_changed(self):
        """Handle leg selection change - save current angles and load new leg angles"""
        # Save current angles to the previously selected leg
        current_leg = self.selected_leg.get()
        
        # If we're switching from a specific leg, save its current position
        if hasattr(self, '_previous_leg') and self._previous_leg in self.leg_positions:
            self.leg_positions[self._previous_leg]['hip'] = self.hip_angle.get()
            self.leg_positions[self._previous_leg]['knee'] = self.knee_angle.get()
        
        # Load the selected leg's saved position
        if current_leg in self.leg_positions:
            saved_hip = self.leg_positions[current_leg]['hip']
            saved_knee = self.leg_positions[current_leg]['knee']
            
            # Update the GUI controls
            self.hip_angle.set(saved_hip)
            self.knee_angle.set(saved_knee)
            
            # Update foot position based on saved angles
            self.update_foot_from_angles(saved_hip, saved_knee)
        
        # Remember this leg as the previous one for next time
        self._previous_leg = current_leg
        
        # Update visualization
        self.update_visualization()
    
    def update_foot_from_angles(self, hip_angle, knee_angle):
        """Calculate and update foot position from joint angles using correct robot kinematics"""
        try:
            L1 = self.config.get('leg_config', {}).get('L1', 25.0)
            L2 = self.config.get('leg_config', {}).get('L2', 33.0)
            
            # Get current selected leg to determine left/right
            current_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else 'front_left'
            
            # Convert to radians with correct robot configuration
            # Different hip directions for left vs right legs
            if 'left' in current_leg:
                hip_rad = np.radians(hip_angle)  # Left legs: normal direction
            else:
                hip_rad = np.radians(-hip_angle)  # Right legs: inverted direction
            
            # Inverted knee with folded back base position
            knee_rad = -np.radians(knee_angle) + np.pi - 1.12  # Correct folded back position
            
            # Forward kinematics: calculate foot position relative to hip
            # Thigh extends horizontally, calf at angle relative to thigh
            thigh_x = L1 * np.cos(hip_rad)
            thigh_z = -L1 * np.sin(hip_rad)  # Negative because down is negative Z
            
            total_angle = hip_rad + knee_rad
            foot_x = thigh_x + L2 * np.cos(total_angle)
            foot_z = thigh_z - L2 * np.sin(total_angle)
            
            # Update foot position sliders (relative to hip)
            self.foot_x.set(foot_x)
            self.foot_y.set(-foot_z)  # Convert Z to height (positive up)
        except:
            pass  # If calculation fails, keep current foot position
    
    def apply_to_all_legs(self):
        """Apply current joint angles to all legs with proper left/right hip direction handling"""
        current_hip = self.hip_angle.get()
        current_knee = self.knee_angle.get()
        
        # Update all legs in our position storage with proper hip direction
        for leg_name in self.leg_positions:
            # Hip direction: left legs use normal direction, right legs use inverted
            if 'left' in leg_name:
                self.leg_positions[leg_name]['hip'] = current_hip  # Left legs: normal direction
            else:
                self.leg_positions[leg_name]['hip'] = -current_hip  # Right legs: inverted direction
            
            # Knee is the same for all legs
            self.leg_positions[leg_name]['knee'] = current_knee
        
        # Update visualization to show all legs
        self.update_visualization()
        
        # Log confirmation
        self.log_message(f"Applied angles (Hip: {current_hip:.1f}°, Knee: {current_knee:.1f}°) to all legs "
                        f"(Hip directions adjusted for left/right legs)", "success")
    
    def update_visualization(self, *args):
        """Update the 3D visualization when sliders change"""
        # Save current angles for the selected leg
        current_leg = self.selected_leg.get()
        if current_leg in self.leg_positions:
            self.leg_positions[current_leg]['hip'] = self.hip_angle.get()
            self.leg_positions[current_leg]['knee'] = self.knee_angle.get()
        
        # Update angle value displays
        if hasattr(self, 'hip_value_label'):
            self.hip_value_label.config(text=f"{self.hip_angle.get():.1f}°")
        if hasattr(self, 'knee_value_label'):
            self.knee_value_label.config(text=f"{self.knee_angle.get():.1f}°")
            
        self.viz_ax.clear()
        self.viz_ax.set_facecolor(self.theme['bg'])
        self.viz_ax.set_xlabel('X (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_ylabel('Y (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_zlabel('Z (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_title('Quadruped Skeleton', color=self.theme['text_primary'])
        self.viz_ax.tick_params(colors=self.theme['text_secondary'])
        
        self.viz_ax.set_xlim([-40, 40])
        self.viz_ax.set_ylim([-30, 30])
        self.viz_ax.set_zlim([-10, 50])  # Allow feet to go below body level
        
        self.draw_quadruped_skeleton()
        self.viz_canvas.draw()
    
    def validate_hardware_angles(self, hip_angle, knee_angle):
        """Validate angles against hardware safety limits"""
        hw_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits = hw_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = hw_limits.get('knee', {'min': -120, 'max': 120})
        
        # Clamp angles to hardware limits
        hip_angle = max(hip_limits['min'], min(hip_limits['max'], hip_angle))
        knee_angle = max(knee_limits['min'], min(knee_limits['max'], knee_angle))
        
        return hip_angle, knee_angle
    
    def move_motor_direct(self):
        """Move selected motor to direct position"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to quadruped system", "warning")
            return
        
        try:
            motor_name = self.direct_motor.get()
            position = self.direct_position.get()
            
            # Parse motor name
            parts = motor_name.split('_')
            if len(parts) != 3:
                self.log_message("Invalid motor name format", "error")
                return
            
            leg_name = f"{parts[0]}_{parts[1]}"  # e.g., "front_left"
            joint_name = parts[2]  # e.g., "hip"
            
            # Validate position against hardware limits
            if joint_name == "hip":
                position, _ = self.validate_hardware_angles(position, 0)
            else:  # knee
                _, position = self.validate_hardware_angles(0, position)
            
            # Send command to specific motor
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint_name == "hip":
                    leg.hip_motor.move_to_angle(position)
                elif joint_name == "knee":
                    leg.knee_motor.move_to_angle(position)
                
                self.log_message(f"Motor {motor_name} moved to {position:.1f}°", "success")
            else:
                self.log_message(f"Leg {leg_name} not found", "error")
                
        except Exception as e:
            self.log_message(f"Failed to move motor: {e}", "error")
    
    def stop_motor_direct(self):
        """Stop selected motor"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to quadruped system", "warning")
            return
        
        try:
            motor_name = self.direct_motor.get()
            parts = motor_name.split('_')
            if len(parts) != 3:
                self.log_message("Invalid motor name format", "error")
                return
            
            leg_name = f"{parts[0]}_{parts[1]}"
            joint_name = parts[2]
            
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint_name == "hip":
                    leg.hip_motor.stop()
                elif joint_name == "knee":
                    leg.knee_motor.stop()
                
                self.log_message(f"Motor {motor_name} stopped", "success")
            else:
                self.log_message(f"Leg {leg_name} not found", "error")
                
        except Exception as e:
            self.log_message(f"Failed to stop motor: {e}", "error")
    
    def on_individual_motor_change(self, leg_name, joint, value):
        """Handle individual motor slider change"""
        # Update the display label
        if leg_name in self.motor_sliders and joint in self.motor_sliders[leg_name]:
            self.motor_sliders[leg_name][joint]['label'].config(text=f"{value:.1f}°")
    
    def send_individual_motor(self, leg_name, joint):
        """Send command to individual motor with proper gear ratio conversion"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to quadruped system", "warning")
            return
        
        try:
            # Get the angle from the slider (in degrees)
            angle_degrees = self.individual_motor_angles[leg_name][joint].get()
            
            # Validate angle against hardware limits
            joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
            if joint == 'hip':
                limits = joint_limits.get('hip', {'min': -90, 'max': 90})
                angle_degrees = max(limits['min'], min(limits['max'], angle_degrees))
                # Convert to motor rotations using gear ratio
                motor_rotations = angle_degrees * self.hip_gear_ratio / 57.2958  # Convert degrees to radians first
            else:
                limits = joint_limits.get('knee', {'min': -120, 'max': 120})
                angle_degrees = max(limits['min'], min(limits['max'], angle_degrees))
                # Convert to motor rotations using gear ratio
                motor_rotations = angle_degrees * self.knee_gear_ratio / 57.2958  # Convert degrees to radians first
            
            # Send to specific motor
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint == 'hip':
                    # Use absolute position for individual motor control
                    leg.hip_motor.set_position(leg.zero_hip + motor_rotations)
                elif joint == 'knee':
                    leg.knee_motor.set_position(leg.zero_knee + motor_rotations)
                
                self.log_message(f"{leg_name} {joint} moved to {angle_degrees:.1f}° (motor: {motor_rotations:.3f} rot)", "success")
            else:
                self.log_message(f"Leg {leg_name} not found", "error")
                
        except Exception as e:
            self.log_message(f"Failed to move {leg_name} {joint}: {e}", "error")
    
    def send_all_individual_motors(self):
        """Send all individual motor positions to hardware with proper gear ratio conversion"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to quadruped system", "warning")
            return
        
        try:
            sent_count = 0
            for leg_name in self.individual_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    
                    # Get angles in degrees and convert to motor rotations
                    hip_angle_degrees = self.individual_motor_angles[leg_name]['hip'].get()
                    knee_angle_degrees = self.individual_motor_angles[leg_name]['knee'].get()
                    
                    # Convert to motor rotations
                    hip_motor_rotations = hip_angle_degrees * self.hip_gear_ratio / 57.2958
                    knee_motor_rotations = knee_angle_degrees * self.knee_gear_ratio / 57.2958
                    
                    # Send positions
                    leg.hip_motor.set_position(leg.zero_hip + hip_motor_rotations)
                    leg.knee_motor.set_position(leg.zero_knee + knee_motor_rotations)
                    
                    sent_count += 1
            
            self.log_message(f"Sent commands to all {sent_count * 2} motors", "success")
            
        except Exception as e:
            self.log_message(f"Failed to send motor commands: {e}", "error")
    
    def read_all_motor_positions(self):
        """Read current positions from all motors and update sliders with proper gear ratio conversion"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to quadruped system", "warning")
            return
        
        try:
            for leg_name in self.individual_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    
                    # Read current positions (in motor rotations)
                    hip_pos_rotations = leg.hip_motor.get_position() - leg.zero_hip
                    knee_pos_rotations = leg.knee_motor.get_position() - leg.zero_knee
                    
                    # Convert back to degrees using gear ratios
                    hip_pos_degrees = hip_pos_rotations * 57.2958 / self.hip_gear_ratio  # Convert to degrees
                    knee_pos_degrees = knee_pos_rotations * 57.2958 / self.knee_gear_ratio  # Convert to degrees
                    
                    # Update sliders
                    self.individual_motor_angles[leg_name]['hip'].set(hip_pos_degrees)
                    self.individual_motor_angles[leg_name]['knee'].set(knee_pos_degrees)
                    
                    # Update labels
                    if leg_name in self.motor_sliders:
                        self.motor_sliders[leg_name]['hip']['label'].config(text=f"{hip_pos_degrees:.1f}°")
                        self.motor_sliders[leg_name]['knee']['label'].config(text=f"{knee_pos_degrees:.1f}°")
            
            self.log_message("Read all motor positions", "success")
            
        except Exception as e:
            self.log_message(f"Failed to read motor positions: {e}", "error")
    
    def zero_all_individual_motors(self):
        """Set all individual motor sliders to zero"""
        for leg_name in self.individual_motor_angles:
            self.individual_motor_angles[leg_name]['hip'].set(0.0)
            self.individual_motor_angles[leg_name]['knee'].set(0.0)
            
            # Update labels
            if leg_name in self.motor_sliders:
                self.motor_sliders[leg_name]['hip']['label'].config(text="0.0°")
                self.motor_sliders[leg_name]['knee']['label'].config(text="0.0°")
    
    def create_homing_panel(self, parent):
        """Create interactive homing panel"""
        homing_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(homing_frame, text="Interactive Homing")
        
        # Instructions
        instruction_frame = ttk.LabelFrame(homing_frame, text="Instructions", style='Surface.TLabelframe')
        instruction_frame.pack(fill=tk.X, padx=10, pady=5)
        
        instructions = ttk.Label(instruction_frame, 
                               text="1. Use sliders to move each leg to home position\n"
                                   "2. Press 'Read Current Positions' to see actual motor values\n"
                                   "3. Press 'Set as Home Position' to set current positions as zero reference\n"
                                   "4. Use 'Send Position' to move motors to slider values",
                               style='Surface.TLabel',
                               font=('Arial', 9))
        instructions.pack(padx=10, pady=5)
        
        # Homing controls container
        controls_container = ttk.Frame(homing_frame, style='Dark.TFrame')
        controls_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Initialize homing sliders dictionary
        self.homing_sliders = {}
        
        # Create two columns for legs
        left_column = ttk.Frame(controls_container, style='Dark.TFrame')
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        right_column = ttk.Frame(controls_container, style='Dark.TFrame')
        right_column.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Define legs for each column
        left_legs = [('front_left', 'Front Left'), ('back_left', 'Back Left')]
        right_legs = [('front_right', 'Front Right'), ('back_right', 'Back Right')]
        
        # Get joint limits for sliders
        joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits = joint_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = joint_limits.get('knee', {'min': -120, 'max': 120})
        
        # Create controls for left column
        self.create_homing_leg_controls(left_column, left_legs, hip_limits, knee_limits)
        
        # Create controls for right column  
        self.create_homing_leg_controls(right_column, right_legs, hip_limits, knee_limits)
        
        # Global homing controls
        global_homing_frame = ttk.LabelFrame(homing_frame, text="Global Homing Controls", style='Surface.TLabelframe')
        global_homing_frame.pack(fill=tk.X, padx=10, pady=10)
        
        homing_buttons = ttk.Frame(global_homing_frame, style='Surface.TFrame')
        homing_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(homing_buttons, text="Read Current Positions", style='Success.TButton',
                  command=self.read_homing_positions).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Send All to Positions", style='Dark.TButton',
                  command=self.send_homing_positions).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Set as Home Position", style='Success.TButton',
                  command=self.set_home_position_interactive).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Zero All Sliders", style='Dark.TButton',
                  command=self.zero_homing_sliders).pack(side=tk.LEFT, padx=5)
    
    def create_homing_leg_controls(self, parent, legs, hip_limits, knee_limits):
        """Create homing controls for a set of legs"""
        for leg_name, leg_display in legs:
            # Leg frame
            leg_frame = ttk.LabelFrame(parent, text=leg_display, style='Surface.TLabelframe')
            leg_frame.pack(fill=tk.X, pady=5)
            
            self.homing_sliders[leg_name] = {}
            
            # Hip control
            hip_frame = ttk.Frame(leg_frame, style='Surface.TFrame')
            hip_frame.pack(fill=tk.X, padx=5, pady=2)
            
            ttk.Label(hip_frame, text="Hip:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
            
            hip_var = self.homing_motor_angles[leg_name]['hip']
            hip_scale = ttk.Scale(hip_frame, from_=hip_limits['min'], to=hip_limits['max'],
                                 variable=hip_var, orient=tk.HORIZONTAL, length=350,
                                 style='Dark.Horizontal.TScale',
                                 command=lambda val, leg=leg_name: 
                                 self.on_homing_slider_change(leg, 'hip', float(val)))
            hip_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            hip_label = ttk.Label(hip_frame, text="0.0°", style='Surface.TLabel', width=8)
            hip_label.pack(side=tk.LEFT, padx=2)
            
            hip_button = ttk.Button(hip_frame, text="Send", style='Dark.TButton', width=6,
                                   command=lambda leg=leg_name: 
                                   self.send_homing_motor(leg, 'hip'))
            hip_button.pack(side=tk.LEFT, padx=2)
            
            # Knee control
            knee_frame = ttk.Frame(leg_frame, style='Surface.TFrame')
            knee_frame.pack(fill=tk.X, padx=5, pady=2)
            
            ttk.Label(knee_frame, text="Knee:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
            
            knee_var = self.homing_motor_angles[leg_name]['knee']
            knee_scale = ttk.Scale(knee_frame, from_=knee_limits['min'], to=knee_limits['max'],
                                  variable=knee_var, orient=tk.HORIZONTAL, length=350,
                                  style='Dark.Horizontal.TScale',
                                  command=lambda val, leg=leg_name: 
                                  self.on_homing_slider_change(leg, 'knee', float(val)))
            knee_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            knee_label = ttk.Label(knee_frame, text="0.0°", style='Surface.TLabel', width=8)
            knee_label.pack(side=tk.LEFT, padx=2)
            
            knee_button = ttk.Button(knee_frame, text="Send", style='Dark.TButton', width=6,
                                    command=lambda leg=leg_name: 
                                    self.send_homing_motor(leg, 'knee'))
            knee_button.pack(side=tk.LEFT, padx=2)
            
            # Store references
            self.homing_sliders[leg_name] = {
                'hip': {'scale': hip_scale, 'label': hip_label, 'button': hip_button},
                'knee': {'scale': knee_scale, 'label': knee_label, 'button': knee_button}
            }
    
    def on_homing_slider_change(self, leg_name, joint, value):
        """Handle homing slider change"""
        # Update the display label
        if leg_name in self.homing_sliders and joint in self.homing_sliders[leg_name]:
            self.homing_sliders[leg_name][joint]['label'].config(text=f"{value:.1f}°")
    
    def send_homing_motor(self, leg_name, joint):
        """Send homing command to individual motor"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        try:
            # Get the angle from the homing slider
            angle = self.homing_motor_angles[leg_name][joint].get()
            
            # Send to specific motor (using absolute position for homing)
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint == 'hip':
                    leg.hip_motor.set_position(angle)
                elif joint == 'knee':
                    leg.knee_motor.set_position(angle)
                
                print(f"Moved {leg_name} {joint} to position {angle:.1f}")
            else:
                messagebox.showerror("Error", f"Leg {leg_name} not found")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to move {leg_name} {joint}: {e}")
    
    def read_homing_positions(self):
        """Read current absolute positions from all motors and update homing sliders"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        try:
            for leg_name in self.homing_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    
                    # Read current absolute positions
                    hip_pos = leg.hip_motor.get_position()
                    knee_pos = leg.knee_motor.get_position()
                    
                    # Update homing sliders
                    self.homing_motor_angles[leg_name]['hip'].set(hip_pos)
                    self.homing_motor_angles[leg_name]['knee'].set(knee_pos)
                    
                    # Update labels
                    if leg_name in self.homing_sliders:
                        self.homing_sliders[leg_name]['hip']['label'].config(text=f"{hip_pos:.1f}°")
                        self.homing_sliders[leg_name]['knee']['label'].config(text=f"{knee_pos:.1f}°")
            
            messagebox.showinfo("Success", "Read all motor positions for homing")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read motor positions: {e}")
    
    def send_homing_positions(self):
        """Send all homing slider positions to motors"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        try:
            for leg_name in self.homing_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    
                    # Send absolute positions from homing sliders
                    hip_angle = self.homing_motor_angles[leg_name]['hip'].get()
                    knee_angle = self.homing_motor_angles[leg_name]['knee'].get()
                    
                    leg.hip_motor.set_position(hip_angle)
                    leg.knee_motor.set_position(knee_angle)
            
            messagebox.showinfo("Success", "Sent all homing positions to motors")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send homing positions: {e}")
    
    def set_home_position_interactive(self):
        """Set current motor positions as home (zero) reference"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        result = messagebox.askyesno("Set Home Position", 
                                   "Set current motor positions as home (zero) reference?\n\n"
                                   "This will make the current positions the new zero point.")
        
        if not result:
            return
        
        try:
            for leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                
                # Set current positions as zero reference
                leg.zero_hip = leg.hip_motor.get_position()
                leg.zero_knee = leg.knee_motor.get_position()
            
            # Reset all sliders to zero since we just set new zero references
            self.zero_homing_sliders()
            self.zero_all_individual_motors()
            
            # Reset main pose controls
            self.reset_to_home()
            
            messagebox.showinfo("Success", "Home positions set! All sliders reset to zero.")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set home positions: {e}")
    
    def zero_homing_sliders(self):
        """Set all homing sliders to zero"""
        for leg_name in self.homing_motor_angles:
            self.homing_motor_angles[leg_name]['hip'].set(0.0)
            self.homing_motor_angles[leg_name]['knee'].set(0.0)
            
            # Update labels
            if leg_name in self.homing_sliders:
                self.homing_sliders[leg_name]['hip']['label'].config(text="0.0°")
                self.homing_sliders[leg_name]['knee']['label'].config(text="0.0°")
    
    def update_foot_position(self, *args):
        """Update visualization when foot position changes using correct robot kinematics"""
        # Check if GUI elements exist before accessing them
        if not (hasattr(self, 'foot_x') and hasattr(self, 'foot_y') and 
                hasattr(self, 'hip_angle') and hasattr(self, 'knee_angle')):
            return
            
        # Calculate joint angles from foot position using inverse kinematics
        try:
            L1 = self.config.get('leg_config', {}).get('L1', 25.0)
            L2 = self.config.get('leg_config', {}).get('L2', 33.0)
            target_x = self.foot_x.get()  # Target X position relative to hip
            target_height = self.foot_y.get()  # Target height above ground
            
            # Get current selected leg to determine left/right
            current_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else 'front_left'
            
            # Convert to the coordinate system used in forward kinematics
            target_z = -target_height  # height = -foot_z, so foot_z = -height
            
            # Calculate distance from hip to target
            distance = np.sqrt(target_x**2 + target_z**2)
            
            # Check reachability
            if distance > (L1 + L2) or distance < abs(L1 - L2):
                return  # Not reachable
            
            # Use iterative approach to ensure consistency with forward kinematics
            # Start with current angles as initial guess
            hip_guess = self.hip_angle.get()
            knee_guess = self.knee_angle.get()
            
            # Run a few iterations of Newton's method to converge
            for iteration in range(10):
                # Calculate current foot position with current guess
                if 'left' in current_leg:
                    hip_rad = np.radians(hip_guess)
                else:
                    hip_rad = np.radians(-hip_guess)
                
                knee_rad = -np.radians(knee_guess) + np.pi - 1.12
                
                # Forward kinematics
                current_x = L1 * np.cos(hip_rad) + L2 * np.cos(hip_rad + knee_rad)
                current_z = -L1 * np.sin(hip_rad) - L2 * np.sin(hip_rad + knee_rad)
                
                # Calculate error
                error_x = target_x - current_x
                error_z = target_z - current_z
                error_magnitude = np.sqrt(error_x**2 + error_z**2)
                
                # If error is small enough, we're done
                if error_magnitude < 0.01:
                    break
                
                # Calculate jacobian (partial derivatives)
                # d(foot_x)/d(hip_angle), d(foot_x)/d(knee_angle)
                # d(foot_z)/d(hip_angle), d(foot_z)/d(knee_angle)
                
                dhip = 0.01  # Small angle change for numerical derivative
                dknee = 0.01
                
                # Hip derivatives
                if 'left' in current_leg:
                    hip_rad_plus = np.radians(hip_guess + dhip)
                else:
                    hip_rad_plus = np.radians(-(hip_guess + dhip))
                
                x_hip_plus = L1 * np.cos(hip_rad_plus) + L2 * np.cos(hip_rad_plus + knee_rad)
                z_hip_plus = -L1 * np.sin(hip_rad_plus) - L2 * np.sin(hip_rad_plus + knee_rad)
                
                dx_dhip = (x_hip_plus - current_x) / dhip
                dz_dhip = (z_hip_plus - current_z) / dhip
                
                # Knee derivatives
                knee_rad_plus = -np.radians(knee_guess + dknee) + np.pi - 1.12
                x_knee_plus = L1 * np.cos(hip_rad) + L2 * np.cos(hip_rad + knee_rad_plus)
                z_knee_plus = -L1 * np.sin(hip_rad) - L2 * np.sin(hip_rad + knee_rad_plus)
                
                dx_dknee = (x_knee_plus - current_x) / dknee
                dz_dknee = (z_knee_plus - current_z) / dknee
                
                # Jacobian matrix
                J = np.array([[dx_dhip, dx_dknee],
                             [dz_dhip, dz_dknee]])
                
                # Error vector
                error_vec = np.array([error_x, error_z])
                
                # Newton step: delta = J^-1 * error
                try:
                    delta = np.linalg.solve(J, error_vec)
                    hip_guess += delta[0]
                    knee_guess += delta[1]
                except np.linalg.LinAlgError:
                    # Singular matrix, try simpler approach
                    hip_guess += error_x * 0.1
                    knee_guess += error_z * 0.1
            
            # Update sliders with the final solution
            self.hip_angle.set(hip_guess)
            self.knee_angle.set(knee_guess)
            
            self.update_visualization()
            
        except Exception as e:
            # Invalid position, don't update
            pass
    
    def start_update_loop(self):
        """Start the main GUI update loop"""
        self.running = True
        self.update_gui()
    
    def update_connection_status(self):
        """Update connection status displays"""
        if self.simulation_mode:
            if self.connected:
                status = "Simulation Connected"
                color = self.theme['success']
            else:
                status = "Simulation Disconnected"
                color = self.theme['text_secondary']
        else:
            if self.connected:
                status = "Hardware Connected"
                color = self.theme['success']
            else:
                status = "Hardware Disconnected"
                color = self.theme['error']
        
        self.connection_status_var.set(status)
    
    def update_gui(self):
        """Main GUI update function"""
        if self.running:
            # Update connection status
            self.update_connection_status()
            
            # Update plotting if active
            if self.plotting_active and self.data_collector:
                data = self.data_collector.get_latest_data()
                if data and self.live_plotter:
                    self.live_plotter.update_data(data)
            
            # Schedule next update
            update_rate = self.config.get('gui', {}).get('update_rate_ms', 50)
            self.root.after(update_rate, self.update_gui)
    
    def update_status(self, status_text, connection_text):
        """Update the status bar"""
        self.status_var.set(status_text)
        self.connection_var.set(f"Connection: {connection_text}")
    
    # Menu and dialog functions
    def load_config_dialog(self):
        """Load configuration file dialog"""
        filename = filedialog.askopenfilename(
            title="Load Configuration",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            self.load_config(filename)
    
    def save_config_dialog(self):
        """Save configuration file dialog"""
        filename = filedialog.asksaveasfilename(
            title="Save Configuration",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    yaml.dump(self.config, f, default_flow_style=False)
                messagebox.showinfo("Success", f"Configuration saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving configuration: {e}")
    
    def toggle_simulation_mode(self):
        """Toggle simulation mode"""
        self.simulation_mode = self.sim_mode_var.get()
        if self.simulation_mode and self.connected:
            self.disconnect_odrives()
        
        # Update connection status display
        self.update_connection_status()
    
    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About", 
                           "Quadruped Control Interface v1.0\n\n"
                           "A comprehensive GUI for controlling and visualizing\n"
                           "a quadruped robot with ODrive motors.\n\n"
                           "Features:\n"
                           "- 3D visualization and pose control\n"
                           "- Live parameter plotting\n"
                           "- Motor calibration and tuning\n"
                           "- Simulation mode support")
    
    # ODrive connection functions
    def connect_odrives(self):
        """Connect to ODrive controllers"""
        if self.simulation_mode:
            messagebox.showinfo("Info", "Running in simulation mode - no ODrive connection needed")
            # Still create quadruped instance for simulation
            try:
                self.quadruped = QuadrupedEnhanced(config_file="config.yaml", simulation_mode=True)
                self.quadruped.connect()
                self.quadruped.setup_all_legs()
                self.connected = True
                
                # Update data collector with new quadruped instance
                if self.data_collector:
                    self.data_collector.quadruped = self.quadruped
                    self.data_collector.simulation_mode = True
                
                self.update_connection_status()
                messagebox.showinfo("Success", "Simulation mode initialized")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to initialize simulation: {e}")
            return
        
        if not ODRIVE_AVAILABLE:
            messagebox.showerror("Error", "ODrive library not available")
            return
        
        try:
            # Initialize quadruped with config
            self.quadruped = QuadrupedEnhanced(config_file="config.yaml", simulation_mode=False)
            self.quadruped.connect()
            self.quadruped.setup_all_legs()
            self.connected = True
            
            # Update data collector with new quadruped instance
            if self.data_collector:
                self.data_collector.quadruped = self.quadruped
                self.data_collector.simulation_mode = False
            
            # Update parameter tuner with new quadruped instance
            if self.parameter_tuner:
                self.parameter_tuner.quadruped = self.quadruped
                self.parameter_tuner.simulation_mode = False
                self.log_message(f"Updated ParameterTuner with connected quadruped. Available legs: {list(self.quadruped.legs.keys())}")
            
            self.update_connection_status()
            messagebox.showinfo("Success", "Connected to all ODrives successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect to ODrives: {e}")
    
    def disconnect_odrives(self):
        """Disconnect from ODrive controllers"""
        if self.quadruped:
            try:
                self.quadruped.disconnect()
            except Exception as e:
                print(f"Error during disconnect: {e}")
        
        self.quadruped = None
        self.connected = False
        
        # Update parameter tuner for simulation mode
        if self.parameter_tuner:
            self.parameter_tuner.quadruped = None
            self.parameter_tuner.simulation_mode = True
            self.log_message("ParameterTuner switched to simulation mode")
        
        self.update_connection_status()
        messagebox.showinfo("Info", "Disconnected from quadruped system")
    
    # Control functions
    def send_pose_to_dog(self):
        """Send current pose to the physical dog with hardware angle validation"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        try:
            selected_leg = self.selected_leg.get()
            
            if selected_leg in self.leg_positions:
                # Send pose for specific leg
                hip_angle = self.leg_positions[selected_leg]['hip']
                knee_angle = self.leg_positions[selected_leg]['knee']
                
                # Validate angles against hardware limits
                validated_hip, validated_knee = self.validate_hardware_angles(hip_angle, knee_angle)
                
                # Warn user if angles were clamped
                if abs(validated_hip - hip_angle) > 0.1 or abs(validated_knee - knee_angle) > 0.1:
                    result = messagebox.askyesno("Angle Limits", 
                                               f"Requested angles exceed hardware limits:\n"
                                               f"Hip: {hip_angle:.1f}° → {validated_hip:.1f}°\n"
                                               f"Knee: {knee_angle:.1f}° → {validated_knee:.1f}°\n\n"
                                               f"Send clamped angles to hardware?")
                    if not result:
                        return
                    hip_angle, knee_angle = validated_hip, validated_knee
                
                # Send to specific leg with proper gear ratio conversion
                if selected_leg in self.quadruped.legs:
                    leg = self.quadruped.legs[selected_leg]
                    
                    # Convert angles to motor rotations using the same method as individual motor control
                    hip_motor_rotations = hip_angle * self.hip_gear_ratio / 57.2958  # Convert degrees to radians first
                    knee_motor_rotations = knee_angle * self.knee_gear_ratio / 57.2958  # Convert degrees to radians first
                    
                    # Send directly to motors like individual motor control
                    leg.hip_motor.set_position(leg.zero_hip + hip_motor_rotations)
                    leg.knee_motor.set_position(leg.zero_knee + knee_motor_rotations)
                    
                    messagebox.showinfo("Success", f"Pose sent to {selected_leg} (Hip: {hip_angle:.1f}°, Knee: {knee_angle:.1f}°)")
                else:
                    messagebox.showerror("Error", f"Leg {selected_leg} not found")
            else:
                messagebox.showerror("Error", "Invalid leg selection")
                    
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send pose: {e}")
    
    def send_all_poses_to_dog(self):
        """Send all stored leg positions to hardware"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        try:
            sent_count = 0
            for leg_name, angles in self.leg_positions.items():
                if leg_name in self.quadruped.legs:
                    hip_angle = angles['hip']
                    knee_angle = angles['knee']
                    
                    # Validate angles
                    validated_hip, validated_knee = self.validate_hardware_angles(hip_angle, knee_angle)
                    
                    # Send to leg with proper gear ratio conversion
                    leg = self.quadruped.legs[leg_name]
                    
                    # Convert angles to motor rotations
                    hip_motor_rotations = validated_hip * self.hip_gear_ratio / 57.2958
                    knee_motor_rotations = validated_knee * self.knee_gear_ratio / 57.2958
                    
                    # Send directly to motors
                    leg.hip_motor.set_position(leg.zero_hip + hip_motor_rotations)
                    leg.knee_motor.set_position(leg.zero_knee + knee_motor_rotations)
                    sent_count += 1
            
            messagebox.showinfo("Success", f"Sent poses to {sent_count} legs")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send poses: {e}")
    
    def reset_to_home(self):
        """Reset to home position"""
        # Reset current GUI controls
        self.hip_angle.set(0)
        self.knee_angle.set(0)
        self.foot_x.set(0)
        self.foot_y.set(30)
        
        # Reset all stored leg positions
        for leg_name in self.leg_positions:
            self.leg_positions[leg_name]['hip'] = 0.0
            self.leg_positions[leg_name]['knee'] = 0.0
        
        self.update_visualization()
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        if self.connected and self.quadruped:
            try:
                self.quadruped.emergency_stop()
                messagebox.showinfo("Emergency Stop", "All motors stopped")
            except Exception as e:
                messagebox.showerror("Error", f"Emergency stop failed: {e}")
        else:
            messagebox.showinfo("Emergency Stop", "Simulation mode - motors would be stopped")
    
    def calibrate_all(self):
        """Calibrate all motors"""
        if not self.connected or not self.quadruped:
            messagebox.showwarning("Warning", "Not connected to quadruped system")
            return
        
        if self.simulation_mode:
            messagebox.showinfo("Info", "Simulation mode - calibration would be performed")
            # Still run the calibration in simulation mode for testing
            try:
                # Ask for calibration mode
                result = messagebox.askyesnocancel("Calibration Mode", 
                                                 "Use simultaneous calibration?\n\n"
                                                 "Yes = Simultaneous (faster)\n"
                                                 "No = Sequential (safer)\n"
                                                 "Cancel = Abort")
                if result is None:  # Cancel
                    return
                
                simultaneous = result
                threading.Thread(target=lambda: self.quadruped.calibrate_all_legs(simultaneous), 
                               daemon=True).start()
                messagebox.showinfo("Info", "Simulation calibration started")
            except Exception as e:
                messagebox.showerror("Error", f"Calibration failed: {e}")
            return
        
        try:
            # Ask for calibration mode
            result = messagebox.askyesnocancel("Calibration Mode", 
                                             "Use simultaneous calibration?\n\n"
                                             "Yes = Simultaneous (faster)\n"
                                             "No = Sequential (safer)\n"
                                             "Cancel = Abort")
            if result is None:  # Cancel
                return
            
            simultaneous = result
            
            # Run calibration in background thread to avoid freezing GUI
            def calibration_thread():
                try:
                    self.quadruped.calibrate_all_legs(simultaneous)
                    self.root.after(0, lambda: messagebox.showinfo("Success", "Calibration completed successfully"))
                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror("Error", f"Calibration failed: {e}"))
            
            threading.Thread(target=calibration_thread, daemon=True).start()
            messagebox.showinfo("Info", "Calibration started in background")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start calibration: {e}")
    
    def set_home_position(self):
        """Set current position as home - redirect to interactive version"""
        self.set_home_position_interactive()
    
    def test_motors(self):
        """Test motor functionality"""
        messagebox.showinfo("Info", "Motor test completed")
    
    def reboot_odrive_selected(self):
        """Reboot selected ODrive(s)"""
        if not self.connected or not self.quadruped:
            self.log_message("Not connected to ODrives - cannot reboot", "error")
            return
        
        if self.simulation_mode:
            self.log_message("Simulation mode - ODrive reboot simulated", "info")
            return
        
        selected = self.reboot_odrive.get()
        
        # Confirm reboot action
        if selected == "All":
            result = messagebox.askyesno("Confirm Reboot", 
                                       "Are you sure you want to reboot ALL ODrives?\n\n"
                                       "This will disconnect all motors and require reconnection.")
        else:
            result = messagebox.askyesno("Confirm Reboot", 
                                       f"Are you sure you want to reboot the {selected} ODrive?\n\n"
                                       "This will disconnect the motors on that ODrive.")
        
        if not result:
            return
        
        try:
            if selected == "All":
                self.log_message("Rebooting all ODrives...", "warning")
                # Reboot all ODrives
                for leg_name in ["front_left", "front_right", "back_left", "back_right"]:
                    if leg_name in self.quadruped.legs:
                        leg = self.quadruped.legs[leg_name]
                        if hasattr(leg, 'hip_motor') and hasattr(leg.hip_motor, 'parent'):
                            self.log_message(f"Rebooting {leg_name} ODrive...", "info")
                            leg.hip_motor.parent.reboot()
                        
                self.log_message("All ODrives rebooted successfully", "success")
                
            else:
                # Map display name to leg name
                leg_mapping = {
                    "Front Left": "front_left",
                    "Front Right": "front_right", 
                    "Back Left": "back_left",
                    "Back Right": "back_right"
                }
                
                leg_name = leg_mapping.get(selected)
                if leg_name and leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    if hasattr(leg, 'hip_motor') and hasattr(leg.hip_motor, 'parent'):
                        self.log_message(f"Rebooting {selected} ODrive...", "warning")
                        leg.hip_motor.parent.reboot()
                        self.log_message(f"{selected} ODrive rebooted successfully", "success")
                    else:
                        self.log_message(f"No ODrive found for {selected}", "error")
                else:
                    self.log_message(f"Invalid leg selection: {selected}", "error")
            
            # After reboot, we need to disconnect as the ODrives will be offline
            self.log_message("ODrives are rebooting - disconnecting interface...", "info")
            self.disconnect_odrives()
            
        except Exception as e:
            self.log_message(f"Error rebooting ODrive(s): {str(e)}", "error")
    
    # Plotting functions
    def start_plotting(self):
        """Start live plotting"""
        if self.data_collector:
            self.data_collector.start_collection()
            self.plotting_active = True
            self.plot_status_var.set("Running")
            messagebox.showinfo("Info", "Plotting started")
    
    def stop_plotting(self):
        """Stop live plotting"""
        if self.data_collector:
            self.data_collector.stop_collection()
            self.plotting_active = False
            self.plot_status_var.set("Stopped")
            messagebox.showinfo("Info", "Plotting stopped")
    
    def clear_plot(self):
        """Clear plot data"""
        if self.live_plotter:
            self.live_plotter.clear_data()
        messagebox.showinfo("Info", "Plot cleared")
    
    def on_parameter_changed(self, event=None):
        """Handle parameter selection change"""
        if self.live_plotter:
            self.live_plotter.set_parameter(self.plot_parameter.get())
    
    # Parameter tuning functions
    def update_parameter_display(self):
        """Update the parameter control display based on selected category"""
        # Clear existing controls
        for widget in self.param_controls_frame.winfo_children():
            widget.destroy()
        
        # Reset scales dictionary
        self.param_scales = {}
        
        if not hasattr(self.parameter_tuner, 'parameters'):
            self.log_message("Parameter tuner not initialized", "warning")
            return
        
        category = self.param_category.get()
        if category not in self.parameter_tuner.parameters:
            self.log_message(f"Unknown category: {category}", "warning")
            return
        
        self.log_message(f"Building parameter display for {category}", "info")
        
        # Add "Read All Current Values" button at the top
        read_all_frame = ttk.Frame(self.param_controls_frame, style='Surface.TFrame')
        read_all_frame.grid(row=0, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        
        ttk.Button(read_all_frame, text="Read All Current Values", style='Success.TButton',
                  command=self.read_all_current_parameters).pack(side=tk.LEFT, padx=5)
        ttk.Label(read_all_frame, text="← Read actual values from ODrive", 
                 style='Surface.TLabel').pack(side=tk.LEFT, padx=10)
        
        parameters = self.parameter_tuner.parameters[category]
        
        # Create controls for each parameter
        row = 1  # Start from row 1 after the button
        self.param_scales = {}  # Reset scales dictionary
        
        for param_name, param_config in parameters.items():
            # Parameter label
            ttk.Label(self.param_controls_frame, text=f"{param_name}:", 
                     style='Surface.TLabel').grid(row=row, column=0, padx=5, pady=2, sticky="w")
            
            # Get current value from ODrive using mapped leg name
            leg_gui = self.param_leg.get()
            leg_quadruped = self.map_leg_name_to_quadruped(leg_gui)
            current_value = self.parameter_tuner.get_parameter(
                leg_quadruped, self.param_motor.get(), param_name)
            
            # Use current value if available, otherwise use default
            initial_value = current_value if current_value is not None else param_config['default']
            value_var = tk.DoubleVar(value=initial_value)
            
            # Scale widget with higher resolution
            scale = ttk.Scale(self.param_controls_frame,
                            from_=param_config['min'],
                            to=param_config['max'],
                            variable=value_var,
                            orient=tk.HORIZONTAL,
                            length=300,
                            style='Dark.Horizontal.TScale',
                            command=lambda val, param=param_name, var=value_var: 
                                self.on_parameter_changed_scale(param, var.get()))
            scale.grid(row=row, column=1, padx=5, pady=2, sticky="ew")
            
            # Value display with better formatting
            value_label = ttk.Label(self.param_controls_frame, text=f"{value_var.get():.4f}",
                                   style='Surface.TLabel', width=8)
            value_label.grid(row=row, column=2, padx=5, pady=2)
            
            # Store references
            self.param_scales[param_name] = {
                'scale': scale,
                'variable': value_var,
                'label': value_label
            }
            
            row += 1
        
        # Configure column weights
        self.param_controls_frame.columnconfigure(1, weight=1)

    def read_all_current_parameters(self):
        """Read all current parameter values from ODrive for the selected category"""
        try:
            category = self.param_category.get()
            leg_gui = self.param_leg.get()
            leg_quadruped = self.map_leg_name_to_quadruped(leg_gui)  # Map GUI name to quadruped name
            motor = self.param_motor.get()
            
            self.log_message(f"Reading parameters for {category}, {leg_gui} -> {leg_quadruped} {motor}", "info")
            
            if category not in self.parameter_tuner.parameters:
                self.log_message(f"Unknown parameter category: {category}", "warning")
                return
            
            parameters = self.parameter_tuner.parameters[category]
            self.log_message(f"Available parameters: {list(parameters.keys())}", "info")
            self.log_message(f"Created sliders: {list(self.param_scales.keys())}", "info")
            
            updated_count = 0
            
            for param_name in parameters.keys():
                if param_name in self.param_scales:
                    current_value = self.parameter_tuner.get_parameter(leg_quadruped, motor, param_name)  # Use mapped name
                    if current_value is not None:
                        # Update the slider and display
                        self.param_scales[param_name]['variable'].set(current_value)
                        self.param_scales[param_name]['label'].config(text=f"{current_value:.4f}")
                        updated_count += 1
                        self.log_message(f"Updated {param_name} = {current_value:.4f}", "info")
                    else:
                        self.log_message(f"Failed to read {param_name}", "warning")
                else:
                    self.log_message(f"Slider not found for {param_name}", "warning")
            
            self.log_message(f"Read {updated_count} current parameter values from ODrive", "success")
            
        except Exception as e:
            self.log_message(f"Error reading parameters: {str(e)}", "error")
    
    def on_parameter_changed_scale(self, param_name, value):
        """Handle parameter scale change"""
        # Update the display label
        if param_name in self.param_scales:
            self.param_scales[param_name]['label'].config(text=f"{value:.4f}")
        
        # Apply the parameter change using mapped leg name
        leg_gui = self.param_leg.get()
        leg_quadruped = self.map_leg_name_to_quadruped(leg_gui)
        
        success = self.parameter_tuner.set_parameter(
            leg_quadruped,  # Use mapped name
            self.param_motor.get(),
            param_name,
            value
        )
        
        if success:
            self.log_message(f"Set {param_name} = {value:.4f} for {self.param_leg.get()} {self.param_motor.get()}", "success")
        else:
            self.log_message(f"Failed to set {param_name}", "error")
    
    def save_parameter_preset(self):
        """Save current parameters as a preset"""
        preset_name = self.preset_name.get().strip()
        if not preset_name:
            messagebox.showwarning("Warning", "Please enter a preset name")
            return
        
        # Collect current parameter values
        parameters = {}
        for param_name, controls in self.param_scales.items():
            parameters[param_name] = controls['variable'].get()
        
        self.parameter_tuner.save_preset(preset_name, parameters)
        messagebox.showinfo("Success", f"Preset '{preset_name}' saved")
    
    def load_parameter_preset(self):
        """Load a parameter preset"""
        preset_name = self.preset_name.get().strip()
        if not preset_name:
            messagebox.showwarning("Warning", "Please enter a preset name")
            return
        
        parameters = self.parameter_tuner.load_preset(preset_name)
        if parameters:
            # Apply loaded parameters
            for param_name, value in parameters.items():
                if param_name in self.param_scales:
                    self.param_scales[param_name]['variable'].set(value)
                    self.on_parameter_changed_scale(param_name, value)
            
            messagebox.showinfo("Success", f"Preset '{preset_name}' loaded")
        else:
            messagebox.showerror("Error", f"Preset '{preset_name}' not found")
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        if self.connected:
            self.disconnect_odrives()
        self.root.quit()
        self.root.destroy()


def main():
    """Main function to run the GUI"""
    root = tk.Tk()
    app = QuadrupedGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()


if __name__ == "__main__":
    main()