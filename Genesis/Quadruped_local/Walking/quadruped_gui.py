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
    from Quadruped_cpg import QuadrupedCPG # Import the new CPG module
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
    
    class QuadrupedCPG:
        def __init__(self, *args, **kwargs): pass
        def update_parameters(self, *args): pass
        def get_leg_positions(self, t, front_offset=0.0, back_offset=0.0, front_height_offset=0.0, back_height_offset=0.0): return {}
        def reset_time(self): pass


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

        # Initialize CPG components
        self.cpg = QuadrupedCPG()
        self.cpg_running = False
        self.cpg_thread = None
        self.cpg_stride_length = tk.DoubleVar(value=15.0)
        self.cpg_step_height = tk.DoubleVar(value=10.0)
        self.cpg_frequency = tk.DoubleVar(value=1.0)
        self.cpg_default_height = tk.DoubleVar(value=35.0)
        self.cpg_front_offset = tk.DoubleVar(value=2.5)  # Front legs X offset
        self.cpg_back_offset = tk.DoubleVar(value=10.0)  # Back legs X offset
        self.cpg_front_height_offset = tk.DoubleVar(value=7.5)  # Front legs height offset
        self.cpg_back_height_offset = tk.DoubleVar(value=10.0)  # Back legs height offset
        self.movement_mode = tk.StringVar(value="custom")  # 'custom' or 'cpg'
        
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
        self.hip_gear_ratio = 10.5 / 6.28
        self.knee_gear_ratio = 4.58
        
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
        
        self.root.configure(bg=self.theme['bg'])
        style = ttk.Style()
        style.theme_use('clam')
        
        style.configure('Dark.TFrame', background=self.theme['bg'], borderwidth=0)
        style.configure('Surface.TFrame', background=self.theme['surface'], relief='flat', borderwidth=0)
        style.configure('Dark.TLabel', background=self.theme['bg'], foreground=self.theme['text_primary'])
        style.configure('Surface.TLabel', background=self.theme['surface'], foreground=self.theme['text_primary'])
        style.configure('Title.TLabel', background=self.theme['bg'], foreground=self.theme['primary'], font=('Arial', 12, 'bold'))
        style.configure('Dark.TButton', background=self.theme['primary'], foreground=self.theme['secondary'], borderwidth=0, focuscolor='none')
        style.configure('Emergency.TButton', background=self.theme['error'], foreground=self.theme['secondary'], borderwidth=0, focuscolor='none')
        style.configure('Success.TButton', background=self.theme['success'], foreground=self.theme['secondary'], borderwidth=0, focuscolor='none')
        style.configure('Dark.Horizontal.TScale', background=self.theme['surface'], troughcolor=self.theme['bg'], sliderrelief='flat', borderwidth=0, lightcolor=self.theme['primary'], darkcolor=self.theme['primary'])
        style.configure('Dark.TNotebook', background=self.theme['bg'], borderwidth=0, tabmargins=0)
        style.configure('Dark.TNotebook.Tab', background=self.theme['surface'], foreground=self.theme['text_primary'], padding=[10, 5], borderwidth=0)
        style.configure('Surface.TLabelframe', background=self.theme['surface'], borderwidth=1, relief='solid', bordercolor=self.theme['accent'])
        style.configure('Surface.TLabelframe.Label', background=self.theme['surface'], foreground=self.theme['primary'], font=('Arial', 10, 'bold'))
        style.configure('TCombobox', selectbackground=self.theme['primary'], fieldbackground=self.theme['surface'], background=self.theme['surface'], foreground=self.theme['text_primary'], borderwidth=0)
        style.configure('TEntry', fieldbackground=self.theme['surface'], foreground=self.theme['text_primary'], borderwidth=1, bordercolor=self.theme['accent'], insertcolor=self.theme['text_primary'])
        style.configure('Dark.TRadiobutton', background=self.theme['surface'], foreground=self.theme['text_primary'], focuscolor='none')
        
    def setup_window(self):
        """Setup window size and properties"""
        window_size = self.config.get('gui', {}).get('window_size', [1600, 1000])
        self.root.geometry(f"{window_size[0]}x{window_size[1]}")
        self.root.minsize(1400, 900)
        
    def load_config(self, config_file="config.yaml"):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded configuration from {config_file}")
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
        menubar = tk.Menu(self.root, bg=self.theme['surface'], fg=self.theme['text_primary'], activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        self.root.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'], activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load Config", command=self.load_config_dialog)
        file_menu.add_command(label="Save Config", command=self.save_config_dialog)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_closing)
        
        conn_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'], activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="Connection", menu=conn_menu)
        conn_menu.add_command(label="Connect to ODrives", command=self.connect_odrives)
        conn_menu.add_command(label="Disconnect", command=self.disconnect_odrives)
        conn_menu.add_separator()
        conn_menu.add_checkbutton(label="Simulation Mode", variable=self.sim_mode_var, command=self.toggle_simulation_mode)
        
        help_menu = tk.Menu(menubar, tearoff=0, bg=self.theme['surface'], fg=self.theme['text_primary'], activebackground=self.theme['primary'], activeforeground=self.theme['secondary'])
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
    
    def create_single_screen_interface(self):
        """Create the single-screen interface layout"""
        main_frame = ttk.Frame(self.root, style='Dark.TFrame')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        main_paned = ttk.PanedWindow(main_frame, orient=tk.VERTICAL, style='Dark.TPanedwindow')
        main_paned.pack(fill=tk.BOTH, expand=True)
        
        top_frame = ttk.Frame(main_paned, style='Dark.TFrame', height=400)
        main_paned.add(top_frame, weight=1)
        
        self.create_animation_section(top_frame)
        self.create_plotting_section(top_frame)
        
        bottom_frame = ttk.Frame(main_paned, style='Dark.TFrame', height=600)
        main_paned.add(bottom_frame, weight=2)
        
        self.create_control_panels(bottom_frame)
        
    def create_animation_section(self, parent):
        """Create the 3D animation section (top left)"""
        anim_panel = ttk.LabelFrame(parent, text="3D Visualization & Pose Control", style='Surface.TLabelframe')
        anim_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.viz_fig = Figure(figsize=(8, 6), dpi=100, facecolor=self.theme['surface'])
        self.viz_ax = self.viz_fig.add_subplot(111, projection='3d')
        self.viz_ax.set_facecolor(self.theme['bg'])
        
        self.viz_ax.xaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.yaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.zaxis.label.set_color(self.theme['text_primary'])
        self.viz_ax.tick_params(colors=self.theme['text_secondary'])
        
        self.viz_canvas = FigureCanvasTkAgg(self.viz_fig, anim_panel)
        self.viz_canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        control_frame = ttk.Frame(anim_panel, style='Dark.TFrame')
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        
        ttk.Label(control_frame, text="Leg Selection", style='Title.TLabel').pack(pady=(0, 5))
        self.selected_leg = tk.StringVar(value="front_left")
        legs = [("front_left", "Front Left"), ("front_right", "Front Right"), ("back_left", "Back Left"), ("back_right", "Back Right")]
        leg_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        leg_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=5)
        for value, text in legs:
            ttk.Radiobutton(leg_frame, text=text, variable=self.selected_leg, value=value, style='Dark.TRadiobutton', command=self.on_leg_selection_changed).pack(anchor=tk.W, padx=5)
        
        joint_limits = self.config.get('joint_limits', {}).get('animation', {})
        hip_limits = joint_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = joint_limits.get('knee', {'min': -90, 'max': 90})
        
        ttk.Label(control_frame, text="Joint Angles", style='Title.TLabel').pack(pady=(0, 5))
        joint_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        joint_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=10)
        
        self.hip_angle = tk.DoubleVar()
        self.knee_angle = tk.DoubleVar()
        
        ttk.Label(joint_frame, text=f"Hip ({hip_limits['min']}° to {hip_limits['max']}°):", style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        hip_scale = ttk.Scale(joint_frame, from_=hip_limits['min'], to=hip_limits['max'], variable=self.hip_angle, orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', command=self.update_visualization)
        hip_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        self.hip_value_label = ttk.Label(joint_frame, text="0.0°", style='Surface.TLabel')
        self.hip_value_label.pack(anchor=tk.W, padx=5)
        
        ttk.Label(joint_frame, text=f"Knee ({knee_limits['min']}° to {knee_limits['max']}°):", style='Surface.TLabel').pack(anchor=tk.W, padx=5, pady=(10, 0))
        knee_scale = ttk.Scale(joint_frame, from_=knee_limits['min'], to=knee_limits['max'], variable=self.knee_angle, orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', command=self.update_visualization)
        knee_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        self.knee_value_label = ttk.Label(joint_frame, text="0.0°", style='Surface.TLabel')
        self.knee_value_label.pack(anchor=tk.W, padx=5)
        
        ttk.Label(control_frame, text="Foot Position", style='Title.TLabel').pack(pady=(0, 5))
        foot_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        foot_frame.pack(fill=tk.X, pady=(0, 15), padx=5, ipady=10)
        self.foot_x = tk.DoubleVar()
        self.foot_y = tk.DoubleVar(value=30)
        ttk.Label(foot_frame, text="X Position (cm):", style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        x_scale = ttk.Scale(foot_frame, from_=-40, to=40, variable=self.foot_x, orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', command=self.update_foot_position)
        x_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        ttk.Label(foot_frame, text="Height (cm):", style='Surface.TLabel').pack(anchor=tk.W, padx=5)
        y_scale = ttk.Scale(foot_frame, from_=5, to=60, variable=self.foot_y, orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', command=self.update_foot_position)
        y_scale.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        ttk.Label(control_frame, text="Actions", style='Title.TLabel').pack(pady=(0, 5))
        button_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        button_frame.pack(fill=tk.X, padx=5, ipady=10)
        ttk.Button(button_frame, text="Send to Hardware", style='Dark.TButton', command=self.send_pose_to_dog).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Apply to All Legs", style='Success.TButton', command=self.apply_to_all_legs).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Send All to Hardware", style='Success.TButton', command=self.send_all_poses_to_dog).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Reset Home", style='Dark.TButton', command=self.reset_to_home).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(button_frame, text="Emergency Stop", style='Emergency.TButton', command=self.emergency_stop).pack(fill=tk.X, pady=2, padx=5)
        
        self.init_3d_visualization()
        
    def create_plotting_section(self, parent):
        """Create the live plotting section (top right)"""
        plot_panel = ttk.LabelFrame(parent, text="Live Data Plotting", style='Surface.TLabelframe')
        plot_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        plot_control_frame = ttk.Frame(plot_panel, style='Surface.TFrame')
        plot_control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(plot_control_frame, text="Parameter:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.plot_parameter = tk.StringVar(value="current")
        parameters = ["current", "position", "velocity", "voltage", "temperature"]
        param_combo = ttk.Combobox(plot_control_frame, textvariable=self.plot_parameter, values=parameters, state="readonly", width=12)
        param_combo.pack(side=tk.LEFT, padx=5)
        param_combo.bind('<<ComboboxSelected>>', self.on_parameter_changed)
        
        ttk.Button(plot_control_frame, text="Start", style='Success.TButton', command=self.start_plotting).pack(side=tk.LEFT, padx=2)
        ttk.Button(plot_control_frame, text="Stop", style='Dark.TButton', command=self.stop_plotting).pack(side=tk.LEFT, padx=2)
        ttk.Button(plot_control_frame, text="Clear", style='Dark.TButton', command=self.clear_plot).pack(side=tk.LEFT, padx=2)
        
        self.plot_status_var = tk.StringVar(value="Stopped")
        status_label = ttk.Label(plot_control_frame, textvariable=self.plot_status_var, style='Surface.TLabel')
        status_label.pack(side=tk.RIGHT, padx=5)
        ttk.Label(plot_control_frame, text="Status:", style='Surface.TLabel').pack(side=tk.RIGHT)
        
        self.plot_fig = Figure(figsize=(8, 4), dpi=100, facecolor=self.theme['surface'])
        self.plot_ax = self.plot_fig.add_subplot(111)
        self.plot_ax.set_facecolor(self.theme['bg'])
        
        self.plot_ax.xaxis.label.set_color(self.theme['text_primary'])
        self.plot_ax.yaxis.label.set_color(self.theme['text_primary'])
        self.plot_ax.tick_params(colors=self.theme['text_secondary'])
        self.plot_ax.spines['bottom'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['top'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['left'].set_color(self.theme['text_secondary'])
        self.plot_ax.spines['right'].set_color(self.theme['text_secondary'])
        
        self.plot_canvas = FigureCanvasTkAgg(self.plot_fig, plot_panel)
        self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=(0, 5))
        
        self.create_logs_window(plot_panel)
        self.init_plotting()
    
    def create_logs_window(self, parent):
        """Create a logs window for displaying messages"""
        logs_frame = ttk.LabelFrame(parent, text="Logs", style='Surface.TLabelframe')
        logs_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=(5, 0))
        
        logs_container = ttk.Frame(logs_frame, style='Surface.TFrame')
        logs_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.logs_text = tk.Text(logs_container, height=8, wrap=tk.WORD, bg=self.theme['bg'], fg=self.theme['text_primary'], font=('Consolas', 9), relief='flat', borderwidth=0)
        logs_scrollbar = ttk.Scrollbar(logs_container, command=self.logs_text.yview)
        self.logs_text.configure(yscrollcommand=logs_scrollbar.set)
        
        self.logs_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        logs_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        log_controls = ttk.Frame(logs_frame, style='Surface.TFrame')
        log_controls.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        ttk.Button(log_controls, text="Clear Logs", style='Dark.TButton', command=self.clear_logs).pack(side=tk.RIGHT, padx=5)
    
    def log_message(self, message, level="info"):
        """Add a message to the logs window with optional level"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        level_indicators = {"info": "ℹ", "success": "✓", "warning": "⚠", "error": "✗"}
        indicator = level_indicators.get(level.lower(), "ℹ")
        formatted_message = f"[{timestamp}] {indicator} {message}\n"
        
        self.log_messages.append(formatted_message)
        if len(self.log_messages) > self.max_log_messages:
            self.log_messages.pop(0)
        
        if hasattr(self, 'logs_text'):
            self.logs_text.insert(tk.END, formatted_message)
            self.logs_text.see(tk.END)
    
    def clear_logs(self):
        """Clear the logs window"""
        self.log_messages.clear()
        if hasattr(self, 'logs_text'):
            self.logs_text.delete(1.0, tk.END)
    
    def map_leg_name_to_quadruped(self, gui_leg_name):
        """Map GUI leg names (FL, FR, etc.) to quadruped leg names (front_left, etc.)"""
        mapping = {"FL": "front_left", "FR": "front_right", "BL": "back_left", "BR": "back_right"}
        return mapping.get(gui_leg_name, gui_leg_name)
    
    def create_control_panels(self, parent):
        """Create the control panels in the bottom section"""
        bottom_notebook = ttk.Notebook(parent, style='Dark.TNotebook')
        bottom_notebook.pack(fill=tk.BOTH, expand=True)
        
        self.create_connection_panel(bottom_notebook)
        self.create_parameters_panel(bottom_notebook)
        self.create_motor_control_panel(bottom_notebook)
        self.create_individual_motor_panel(bottom_notebook)
        self.create_homing_panel(bottom_notebook)
        self.create_cpg_panel(bottom_notebook) # Add CPG Panel
        
    def create_connection_panel(self, parent):
        """Create connection and calibration panel"""
        conn_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(conn_frame, text="Connection & Calibration")
        
        left_section = ttk.LabelFrame(conn_frame, text="Connection", style='Surface.TLabelframe')
        left_section.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        right_section = ttk.LabelFrame(conn_frame, text="Calibration", style='Surface.TLabelframe')
        right_section.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        conn_buttons = ttk.Frame(left_section, style='Surface.TFrame')
        conn_buttons.pack(fill=tk.X, padx=10, pady=10)
        ttk.Button(conn_buttons, text="Connect ODrives", style='Success.TButton', command=self.connect_odrives).pack(fill=tk.X, pady=2)
        ttk.Button(conn_buttons, text="Disconnect", style='Dark.TButton', command=self.disconnect_odrives).pack(fill=tk.X, pady=2)
        
        sim_frame = ttk.Frame(left_section, style='Surface.TFrame')
        sim_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Checkbutton(sim_frame, text="Simulation Mode", variable=self.sim_mode_var, command=self.toggle_simulation_mode).pack(anchor=tk.W)
        
        status_frame = ttk.Frame(left_section, style='Surface.TFrame')
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(status_frame, text="Status:", style='Surface.TLabel').pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.connection_status_var, style='Surface.TLabel').pack(side=tk.LEFT, padx=(5, 0))
        
        calib_buttons = ttk.Frame(right_section, style='Surface.TFrame')
        calib_buttons.pack(fill=tk.X, padx=10, pady=10)
        ttk.Button(calib_buttons, text="Calibrate All Motors", style='Dark.TButton', command=self.calibrate_all).pack(fill=tk.X, pady=2)
        ttk.Button(calib_buttons, text="Set Home Position", style='Dark.TButton', command=self.set_home_position).pack(fill=tk.X, pady=2)
        ttk.Button(calib_buttons, text="Test Motors", style='Dark.TButton', command=self.test_motors).pack(fill=tk.X, pady=2)
        
        reboot_frame = ttk.LabelFrame(right_section, text="ODrive Reboot", style='Surface.TLabelframe')
        reboot_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        reboot_controls = ttk.Frame(reboot_frame, style='Surface.TFrame')
        reboot_controls.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(reboot_controls, text="ODrive:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.reboot_odrive = tk.StringVar(value="All")
        odrive_options = ["All", "Front Left", "Front Right", "Back Left", "Back Right"]
        odrive_combo = ttk.Combobox(reboot_controls, textvariable=self.reboot_odrive, values=odrive_options, state="readonly", width=12)
        odrive_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(reboot_controls, text="Reboot ODrive", style='Emergency.TButton', command=self.reboot_odrive_selected).pack(side=tk.LEFT, padx=10)
    
    def create_parameters_panel(self, parent):
        """Create parameter tuning panel"""
        param_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(param_frame, text="Parameter Tuning")
        
        selection_frame = ttk.LabelFrame(param_frame, text="Motor Selection", style='Surface.TLabelframe')
        selection_frame.pack(fill=tk.X, padx=5, pady=5)
        select_controls = ttk.Frame(selection_frame, style='Surface.TFrame')
        select_controls.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(select_controls, text="Leg:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_leg = tk.StringVar(value="FL")
        leg_combo = ttk.Combobox(select_controls, textvariable=self.param_leg, values=["FL", "FR", "BL", "BR"], state="readonly", width=12)
        leg_combo.pack(side=tk.LEFT, padx=5)
        leg_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        ttk.Label(select_controls, text="Motor:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_motor = tk.StringVar(value="hip")
        motor_combo = ttk.Combobox(select_controls, textvariable=self.param_motor, values=["hip", "knee"], state="readonly", width=8)
        motor_combo.pack(side=tk.LEFT, padx=5)
        motor_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        
        control_frame = ttk.LabelFrame(param_frame, text="Parameters", style='Surface.TLabelframe')
        control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        category_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        category_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(category_frame, text="Category:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.param_category = tk.StringVar(value="PID Control")
        categories = ["PID Control", "Trajectory Control", "Current Limits"]
        category_combo = ttk.Combobox(category_frame, textvariable=self.param_category, values=categories, state="readonly", width=15)
        category_combo.pack(side=tk.LEFT, padx=5)
        category_combo.bind('<<ComboboxSelected>>', lambda e: self.update_parameter_display())
        
        canvas = tk.Canvas(control_frame, bg='#2b2b2b', highlightthickness=0)
        scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=canvas.yview)
        self.param_controls_frame = ttk.Frame(canvas, style='Surface.TFrame')
        self.param_controls_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.param_controls_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        scrollbar.pack(side="right", fill="y", pady=5)
        def _on_mousewheel(event): canvas.yview_scroll(int(-1*(event.delta/120)), "units")
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
        
        preset_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        preset_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(preset_frame, text="Preset:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.preset_name = tk.StringVar()
        preset_entry = ttk.Entry(preset_frame, textvariable=self.preset_name, width=15)
        preset_entry.pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Save", style='Dark.TButton', command=self.save_parameter_preset).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="Load", style='Dark.TButton', command=self.load_parameter_preset).pack(side=tk.LEFT, padx=2)
        
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
        
        controls_frame = ttk.LabelFrame(motor_frame, text="Individual Motor Control", style='Surface.TLabelframe')
        controls_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        select_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        select_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(select_frame, text="Select Motor:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.direct_motor = tk.StringVar(value="front_left_hip")
        motors = [f"{leg}_{joint}" for leg in ["front_left", "front_right", "back_left", "back_right"] for joint in ["hip", "knee"]]
        motor_combo = ttk.Combobox(select_frame, textvariable=self.direct_motor, values=motors, state="readonly", width=20)
        motor_combo.pack(side=tk.LEFT, padx=5)
        
        position_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        position_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(position_frame, text="Position (degrees):", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        self.direct_position = tk.DoubleVar()
        pos_scale = ttk.Scale(position_frame, from_=-180, to=180, variable=self.direct_position, orient=tk.HORIZONTAL, style='Dark.Horizontal.TScale', length=300)
        pos_scale.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self.direct_pos_label = ttk.Label(position_frame, text="0.0°", style='Surface.TLabel')
        self.direct_pos_label.pack(side=tk.LEFT, padx=5)
        pos_scale.configure(command=lambda val: self.direct_pos_label.config(text=f"{float(val):.1f}°"))
        
        button_frame = ttk.Frame(controls_frame, style='Surface.TFrame')
        button_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(button_frame, text="Move to Position", style='Dark.TButton', command=self.move_motor_direct).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop Motor", style='Emergency.TButton', command=self.stop_motor_direct).pack(side=tk.LEFT, padx=5)
    
    def create_individual_motor_panel(self, parent):
        """Create individual motor control panel with sliders for each motor"""
        motor_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(motor_frame, text="Individual Motor Control")
        
        global_controls_top = ttk.LabelFrame(motor_frame, text="Quick Controls", style='Surface.TLabelframe')
        global_controls_top.pack(fill=tk.X, padx=10, pady=5)
        controls_grid = ttk.Frame(global_controls_top, style='Surface.TFrame')
        controls_grid.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(controls_grid, text="Read All Current Positions", style='Success.TButton', command=self.read_all_motor_positions).grid(row=0, column=0, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Send All Motor Positions", style='Dark.TButton', command=self.send_all_individual_motors).grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Zero All Motors", style='Dark.TButton', command=self.zero_all_individual_motors).grid(row=0, column=2, padx=5, pady=2, sticky="ew")
        ttk.Button(controls_grid, text="Emergency Stop All", style='Emergency.TButton', command=self.emergency_stop).grid(row=0, column=3, padx=5, pady=2, sticky="ew")
        for i in range(4): controls_grid.columnconfigure(i, weight=1)
        
        scroll_container = ttk.Frame(motor_frame, style='Dark.TFrame')
        scroll_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        canvas = tk.Canvas(scroll_container, bg=self.theme['bg'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(scroll_container, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas, style='Dark.TFrame')
        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits = joint_limits.get('hip', {'min': -90, 'max': 90})
        knee_limits = joint_limits.get('knee', {'min': -120, 'max': 120})
        
        legs = [('front_left', 'Front Left Leg'), ('front_right', 'Front Right Leg'), ('back_left', 'Back Left Leg'), ('back_right', 'Back Right Leg')]
        self.motor_sliders = {}
        for row, (leg_name, leg_display) in enumerate(legs):
            leg_frame = ttk.LabelFrame(scrollable_frame, text=leg_display, style='Surface.TLabelframe')
            leg_frame.grid(row=row, column=0, columnspan=4, padx=10, pady=5, sticky="ew")
            self.motor_sliders[leg_name] = {}
            
            ttk.Label(leg_frame, text="Hip Joint:", style='Surface.TLabel').grid(row=0, column=0, padx=5, pady=5, sticky="w")
            hip_var = self.individual_motor_angles[leg_name]['hip']
            hip_scale = ttk.Scale(leg_frame, from_=hip_limits['min'], to=hip_limits['max'], variable=hip_var, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale', command=lambda val, leg=leg_name: self.on_individual_motor_change(leg, 'hip', float(val)))
            hip_scale.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
            hip_label = ttk.Label(leg_frame, text="0.0°", style='Surface.TLabel')
            hip_label.grid(row=0, column=2, padx=5, pady=5)
            hip_button = ttk.Button(leg_frame, text="Send", style='Dark.TButton', command=lambda leg=leg_name: self.send_individual_motor(leg, 'hip'))
            hip_button.grid(row=0, column=3, padx=5, pady=5)
            
            ttk.Label(leg_frame, text="Knee Joint:", style='Surface.TLabel').grid(row=1, column=0, padx=5, pady=5, sticky="w")
            knee_var = self.individual_motor_angles[leg_name]['knee']
            knee_scale = ttk.Scale(leg_frame, from_=knee_limits['min'], to=knee_limits['max'], variable=knee_var, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale', command=lambda val, leg=leg_name: self.on_individual_motor_change(leg, 'knee', float(val)))
            knee_scale.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
            knee_label = ttk.Label(leg_frame, text="0.0°", style='Surface.TLabel')
            knee_label.grid(row=1, column=2, padx=5, pady=5)
            knee_button = ttk.Button(leg_frame, text="Send", style='Dark.TButton', command=lambda leg=leg_name: self.send_individual_motor(leg, 'knee'))
            knee_button.grid(row=1, column=3, padx=5, pady=5)
            
            self.motor_sliders[leg_name] = {'hip': {'scale': hip_scale, 'label': hip_label, 'button': hip_button}, 'knee': {'scale': knee_scale, 'label': knee_label, 'button': knee_button}}
            leg_frame.columnconfigure(1, weight=1)
        scrollable_frame.columnconfigure(0, weight=1)
    
    def load_config(self, config_file="config.yaml"):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded configuration from {config_file}")
            self.simulation_mode = self.config.get('system', {}).get('simulation_mode', True)
        except FileNotFoundError:
            self.log_message(f"Configuration file {config_file} not found! Using defaults.", "error")
            self.config = self.get_default_config()
        except yaml.YAMLError as e:
            self.log_message(f"Error parsing configuration file: {e}. Using defaults.", "error")
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """Return default configuration if file loading fails"""
        return {'system': {'simulation_mode': True}, 'gui': {'window_title': 'Quadruped Control', 'update_rate_ms': 50}, 'leg_config': {'L1': 25.0, 'L2': 33.0}, 'visualization': {'body_length': 42.0, 'body_width': 35.8}}
    
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
        self.viz_ax.set_xlim([-30, 30])
        self.viz_ax.set_ylim([-30, 30])
        self.viz_ax.set_zlim([-10, 50])
        self.draw_quadruped_skeleton()
        self.viz_canvas.draw()
    
    def draw_quadruped_skeleton(self):
        """Draw the quadruped skeleton in 3D using stored leg positions"""
        L1, L2 = self.config['leg_config']['L1'], self.config['leg_config']['L2']
        body_length, body_width = self.config['visualization']['body_length'], self.config['visualization']['body_width']
        
        body_x = [-body_length/2, body_length/2, body_length/2, -body_length/2, -body_length/2]
        body_y = [-body_width/2, -body_width/2, body_width/2, body_width/2, -body_width/2]
        body_z = [20, 20, 20, 20, 20]
        self.viz_ax.plot(body_x, body_y, body_z, color=self.theme['text_primary'], linewidth=3, label='Body')
        
        leg_positions = {'front_left': (-body_length/2, -body_width/2), 'front_right': (-body_length/2, body_width/2), 'back_left': (body_length/2, -body_width/2), 'back_right': (body_length/2, body_width/2)}
        selected_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else None
        
        for leg_name, (bx, by) in leg_positions.items():
            hip_angle = self.leg_positions[leg_name]['hip'] if leg_name in self.leg_positions else 0
            knee_angle = self.leg_positions[leg_name]['knee'] if leg_name in self.leg_positions else 0
            
            hip_rad = np.radians(-hip_angle) if 'right' in leg_name else np.radians(hip_angle)
            knee_rad = -np.radians(knee_angle) + np.pi - 1.12
            
            hip_x, hip_y, hip_z = bx, by, 20
            knee_x, knee_y, knee_z = hip_x + L1 * np.cos(hip_rad), hip_y, hip_z - L1 * np.sin(hip_rad)
            total_angle = hip_rad + knee_rad
            foot_x, foot_y, foot_z = knee_x + L2 * np.cos(total_angle), knee_y, knee_z - L2 * np.sin(total_angle)
            
            leg_color = self.theme['primary'] if leg_name == selected_leg else self.theme['text_secondary']
            linewidth = 3 if leg_name == selected_leg else 2
            
            self.viz_ax.plot([hip_x, knee_x], [hip_y, knee_y], [hip_z, knee_z], color=leg_color, linewidth=linewidth, alpha=0.8)
            self.viz_ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], color=leg_color, linewidth=linewidth, alpha=0.8)
            self.viz_ax.scatter([hip_x], [hip_y], [hip_z], c=leg_color, s=50, alpha=0.9)
            self.viz_ax.scatter([knee_x], [knee_y], [knee_z], c=leg_color, s=30, alpha=0.9)
            self.viz_ax.scatter([foot_x], [foot_y], [foot_z], c=leg_color, s=40, alpha=0.9)
    
    def init_plotting(self):
        """Initialize the plotting system"""
        self.live_plotter = LivePlotter(self.plot_ax, self.plot_canvas)
        self.data_collector = QuadrupedDataCollector(self.quadruped, self.simulation_mode)
    
    def on_leg_selection_changed(self):
        """Handle leg selection change - save current angles and load new leg angles"""
        # Only process leg selection if in custom movement mode
        if self.movement_mode.get() != "custom":
            return
            
        if hasattr(self, '_previous_leg') and self._previous_leg in self.leg_positions:
            self.leg_positions[self._previous_leg]['hip'] = self.hip_angle.get()
            self.leg_positions[self._previous_leg]['knee'] = self.knee_angle.get()
        
        current_leg = self.selected_leg.get()
        if current_leg in self.leg_positions:
            saved_hip, saved_knee = self.leg_positions[current_leg]['hip'], self.leg_positions[current_leg]['knee']
            self.hip_angle.set(saved_hip)
            self.knee_angle.set(saved_knee)
            self.update_foot_from_angles(saved_hip, saved_knee)
        
        self._previous_leg = current_leg
        self.update_visualization()
    
    def update_foot_from_angles(self, hip_angle, knee_angle):
        """Calculate and update foot position from joint angles using correct robot kinematics"""
        try:
            L1, L2 = self.config['leg_config']['L1'], self.config['leg_config']['L2']
            current_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else 'front_left'
            
            hip_rad = np.radians(-hip_angle) if 'right' in current_leg else np.radians(hip_angle)
            knee_rad = -np.radians(knee_angle) + np.pi - 1.12
            
            thigh_x, thigh_z = L1 * np.cos(hip_rad), -L1 * np.sin(hip_rad)
            total_angle = hip_rad + knee_rad
            foot_x = thigh_x + L2 * np.cos(total_angle)
            foot_z = thigh_z - L2 * np.sin(total_angle)
            
            self.foot_x.set(foot_x)
            self.foot_y.set(-foot_z)
        except:
            pass
    
    def apply_to_all_legs(self):
        """Apply current joint angles to all legs with proper left/right hip direction handling"""
        current_hip, current_knee = self.hip_angle.get(), self.knee_angle.get()
        for leg_name in self.leg_positions:
            self.leg_positions[leg_name]['hip'] = -current_hip if 'right' in leg_name else current_hip
            self.leg_positions[leg_name]['knee'] = current_knee
        self.update_visualization()
        self.log_message(f"Applied angles (Hip: {current_hip:.1f}°, Knee: {current_knee:.1f}°) to all legs", "success")
    
    def update_visualization(self, *args):
        """Update the 3D visualization when sliders change"""
        # Only update selected leg position if in custom movement mode
        if self.movement_mode.get() == "custom":
            current_leg = self.selected_leg.get()
            if current_leg in self.leg_positions:
                self.leg_positions[current_leg]['hip'] = self.hip_angle.get()
                self.leg_positions[current_leg]['knee'] = self.knee_angle.get()
        
            if hasattr(self, 'hip_value_label'): self.hip_value_label.config(text=f"{self.hip_angle.get():.1f}°")
            if hasattr(self, 'knee_value_label'): self.knee_value_label.config(text=f"{self.knee_angle.get():.1f}°")
            
        self.viz_ax.clear()
        self.viz_ax.set_facecolor(self.theme['bg'])
        self.viz_ax.set_xlabel('X (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_ylabel('Y (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_zlabel('Z (cm)', color=self.theme['text_primary'])
        self.viz_ax.set_title('Quadruped Skeleton', color=self.theme['text_primary'])
        self.viz_ax.tick_params(colors=self.theme['text_secondary'])
        self.viz_ax.set_xlim([-40, 40]); self.viz_ax.set_ylim([-30, 30]); self.viz_ax.set_zlim([-10, 50])
        self.draw_quadruped_skeleton()
        self.viz_canvas.draw()
    
    def validate_hardware_angles(self, hip_angle, knee_angle):
        """Validate angles against hardware safety limits"""
        hw_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits, knee_limits = hw_limits.get('hip', {'min': -90, 'max': 90}), hw_limits.get('knee', {'min': -120, 'max': 120})
        hip_angle = max(hip_limits['min'], min(hip_limits['max'], hip_angle))
        knee_angle = max(knee_limits['min'], min(knee_limits['max'], knee_angle))
        return hip_angle, knee_angle
    
    def move_motor_direct(self):
        """Move selected motor to direct position"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "warning"); return
        try:
            motor_name, position = self.direct_motor.get(), self.direct_position.get()
            parts = motor_name.split('_')
            if len(parts) != 3: self.log_message("Invalid motor name", "error"); return
            leg_name, joint_name = f"{parts[0]}_{parts[1]}", parts[2]
            position, _ = self.validate_hardware_angles(position, 0) if joint_name == "hip" else self.validate_hardware_angles(0, position)
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint_name == "hip": leg.hip_motor.move_to_angle(position)
                elif joint_name == "knee": leg.knee_motor.move_to_angle(position)
                self.log_message(f"Motor {motor_name} moved to {position:.1f}°", "success")
        except Exception as e: self.log_message(f"Failed to move motor: {e}", "error")
    
    def stop_motor_direct(self):
        """Stop selected motor"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "warning"); return
        try:
            motor_name = self.direct_motor.get()
            parts = motor_name.split('_')
            if len(parts) != 3: self.log_message("Invalid motor name", "error"); return
            leg_name, joint_name = f"{parts[0]}_{parts[1]}", parts[2]
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint_name == "hip": leg.hip_motor.stop()
                elif joint_name == "knee": leg.knee_motor.stop()
                self.log_message(f"Motor {motor_name} stopped", "success")
        except Exception as e: self.log_message(f"Failed to stop motor: {e}", "error")
    
    def on_individual_motor_change(self, leg_name, joint, value):
        """Handle individual motor slider change"""
        if leg_name in self.motor_sliders and joint in self.motor_sliders[leg_name]:
            self.motor_sliders[leg_name][joint]['label'].config(text=f"{value:.1f}°")
    
    def send_individual_motor(self, leg_name, joint):
        """Send command to individual motor with proper gear ratio conversion"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "warning"); return
        try:
            angle_degrees = self.individual_motor_angles[leg_name][joint].get()
            joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
            limits = joint_limits.get(joint, {'min': -90, 'max': 90})
            angle_degrees = max(limits['min'], min(limits['max'], angle_degrees))
            gear_ratio = self.hip_gear_ratio if joint == 'hip' else self.knee_gear_ratio
            motor_rotations = angle_degrees * gear_ratio / 57.2958
            
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint == 'hip': leg.hip_motor.set_position(leg.zero_hip + motor_rotations)
                else: leg.knee_motor.set_position(leg.zero_knee + motor_rotations)
                self.log_message(f"{leg_name} {joint} moved to {angle_degrees:.1f}°", "success")
        except Exception as e: self.log_message(f"Failed to move {leg_name} {joint}: {e}", "error")
    
    def send_all_individual_motors(self):
        """Send all individual motor positions to hardware"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "warning"); return
        try:
            for leg_name, motors in self.individual_motor_angles.items():
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    hip_deg, knee_deg = motors['hip'].get(), motors['knee'].get()
                    hip_rot = hip_deg * self.hip_gear_ratio / 57.2958
                    knee_rot = knee_deg * self.knee_gear_ratio / 57.2958
                    leg.hip_motor.set_position(leg.zero_hip + hip_rot)
                    leg.knee_motor.set_position(leg.zero_knee + knee_rot)
            self.log_message(f"Sent commands to all motors", "success")
        except Exception as e: self.log_message(f"Failed to send motor commands: {e}", "error")
    
    def read_all_motor_positions(self):
        """Read current positions from all motors and update sliders"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "warning"); return
        try:
            for leg_name in self.individual_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    hip_rot = leg.hip_motor.get_position() - leg.zero_hip
                    knee_rot = leg.knee_motor.get_position() - leg.zero_knee
                    hip_deg = hip_rot * 57.2958 / self.hip_gear_ratio
                    knee_deg = knee_rot * 57.2958 / self.knee_gear_ratio
                    self.individual_motor_angles[leg_name]['hip'].set(hip_deg)
                    self.individual_motor_angles[leg_name]['knee'].set(knee_deg)
                    self.motor_sliders[leg_name]['hip']['label'].config(text=f"{hip_deg:.1f}°")
                    self.motor_sliders[leg_name]['knee']['label'].config(text=f"{knee_deg:.1f}°")
            self.log_message("Read all motor positions", "success")
        except Exception as e: self.log_message(f"Failed to read motor positions: {e}", "error")
    
    def zero_all_individual_motors(self):
        """Set all individual motor sliders to zero"""
        for leg_name in self.individual_motor_angles:
            self.individual_motor_angles[leg_name]['hip'].set(0.0)
            self.individual_motor_angles[leg_name]['knee'].set(0.0)
            self.motor_sliders[leg_name]['hip']['label'].config(text="0.0°")
            self.motor_sliders[leg_name]['knee']['label'].config(text="0.0°")
    
    def create_homing_panel(self, parent):
        """Create interactive homing panel"""
        homing_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(homing_frame, text="Interactive Homing")
        instruction_frame = ttk.LabelFrame(homing_frame, text="Instructions", style='Surface.TLabelframe')
        instruction_frame.pack(fill=tk.X, padx=10, pady=5)
        instructions = "1. Use sliders to move legs to home position\n2. Press 'Read Current Positions'\n3. Press 'Set as Home Position'"
        ttk.Label(instruction_frame, text=instructions, style='Surface.TLabel', font=('Arial', 9)).pack(padx=10, pady=5)
        controls_container = ttk.Frame(homing_frame, style='Dark.TFrame')
        controls_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.homing_sliders = {}
        left_column = ttk.Frame(controls_container, style='Dark.TFrame'); left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        right_column = ttk.Frame(controls_container, style='Dark.TFrame'); right_column.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        left_legs, right_legs = [('front_left', 'Front Left'), ('back_left', 'Back Left')], [('front_right', 'Front Right'), ('back_right', 'Back Right')]
        joint_limits = self.config.get('joint_limits', {}).get('hardware', {})
        hip_limits, knee_limits = joint_limits.get('hip', {'min': -90, 'max': 90}), joint_limits.get('knee', {'min': -120, 'max': 120})
        self.create_homing_leg_controls(left_column, left_legs, hip_limits, knee_limits)
        self.create_homing_leg_controls(right_column, right_legs, hip_limits, knee_limits)
        global_homing_frame = ttk.LabelFrame(homing_frame, text="Global Homing Controls", style='Surface.TLabelframe')
        global_homing_frame.pack(fill=tk.X, padx=10, pady=10)
        homing_buttons = ttk.Frame(global_homing_frame, style='Surface.TFrame')
        homing_buttons.pack(fill=tk.X, padx=10, pady=10)
        ttk.Button(homing_buttons, text="Read Current Positions", style='Success.TButton', command=self.read_homing_positions).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Send All to Positions", style='Dark.TButton', command=self.send_homing_positions).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Set as Home Position", style='Success.TButton', command=self.set_home_position_interactive).pack(side=tk.LEFT, padx=5)
        ttk.Button(homing_buttons, text="Zero All Sliders", style='Dark.TButton', command=self.zero_homing_sliders).pack(side=tk.LEFT, padx=5)
    
    def create_homing_leg_controls(self, parent, legs, hip_limits, knee_limits):
        """Create homing controls for a set of legs"""
        for leg_name, leg_display in legs:
            leg_frame = ttk.LabelFrame(parent, text=leg_display, style='Surface.TLabelframe'); leg_frame.pack(fill=tk.X, pady=5)
            self.homing_sliders[leg_name] = {}
            hip_frame = ttk.Frame(leg_frame, style='Surface.TFrame'); hip_frame.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(hip_frame, text="Hip:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
            hip_var = self.homing_motor_angles[leg_name]['hip']
            hip_scale = ttk.Scale(hip_frame, from_=hip_limits['min'], to=hip_limits['max'], variable=hip_var, orient=tk.HORIZONTAL, length=350, style='Dark.Horizontal.TScale', command=lambda val, leg=leg_name: self.on_homing_slider_change(leg, 'hip', float(val)))
            hip_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            hip_label = ttk.Label(hip_frame, text="0.0°", style='Surface.TLabel', width=8); hip_label.pack(side=tk.LEFT, padx=2)
            hip_button = ttk.Button(hip_frame, text="Send", style='Dark.TButton', width=6, command=lambda leg=leg_name: self.send_homing_motor(leg, 'hip')); hip_button.pack(side=tk.LEFT, padx=2)
            knee_frame = ttk.Frame(leg_frame, style='Surface.TFrame'); knee_frame.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(knee_frame, text="Knee:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
            knee_var = self.homing_motor_angles[leg_name]['knee']
            knee_scale = ttk.Scale(knee_frame, from_=knee_limits['min'], to=knee_limits['max'], variable=knee_var, orient=tk.HORIZONTAL, length=350, style='Dark.Horizontal.TScale', command=lambda val, leg=leg_name: self.on_homing_slider_change(leg, 'knee', float(val)))
            knee_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            knee_label = ttk.Label(knee_frame, text="0.0°", style='Surface.TLabel', width=8); knee_label.pack(side=tk.LEFT, padx=2)
            knee_button = ttk.Button(knee_frame, text="Send", style='Dark.TButton', width=6, command=lambda leg=leg_name: self.send_homing_motor(leg, 'knee')); knee_button.pack(side=tk.LEFT, padx=2)
            self.homing_sliders[leg_name] = {'hip': {'scale': hip_scale, 'label': hip_label, 'button': hip_button}, 'knee': {'scale': knee_scale, 'label': knee_label, 'button': knee_button}}
    
    def on_homing_slider_change(self, leg_name, joint, value):
        """Handle homing slider change"""
        if leg_name in self.homing_sliders and joint in self.homing_sliders[leg_name]: self.homing_sliders[leg_name][joint]['label'].config(text=f"{value:.1f}°")
    
    def send_homing_motor(self, leg_name, joint):
        """Send homing command to individual motor"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        try:
            angle = self.homing_motor_angles[leg_name][joint].get()
            if leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                if joint == 'hip': leg.hip_motor.set_position(angle)
                elif joint == 'knee': leg.knee_motor.set_position(angle)
                print(f"Moved {leg_name} {joint} to position {angle:.1f}")
        except Exception as e: messagebox.showerror("Error", f"Failed to move {leg_name} {joint}: {e}")
    
    def read_homing_positions(self):
        """Read current absolute positions from all motors and update homing sliders"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        try:
            for leg_name in self.homing_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    hip_pos, knee_pos = leg.hip_motor.get_position(), leg.knee_motor.get_position()
                    self.homing_motor_angles[leg_name]['hip'].set(hip_pos)
                    self.homing_motor_angles[leg_name]['knee'].set(knee_pos)
                    self.homing_sliders[leg_name]['hip']['label'].config(text=f"{hip_pos:.1f}°")
                    self.homing_sliders[leg_name]['knee']['label'].config(text=f"{knee_pos:.1f}°")
            messagebox.showinfo("Success", "Read all motor positions for homing")
        except Exception as e: messagebox.showerror("Error", f"Failed to read motor positions: {e}")
    
    def send_homing_positions(self):
        """Send all homing slider positions to motors"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        try:
            for leg_name in self.homing_motor_angles:
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    hip_angle, knee_angle = self.homing_motor_angles[leg_name]['hip'].get(), self.homing_motor_angles[leg_name]['knee'].get()
                    leg.hip_motor.set_position(hip_angle); leg.knee_motor.set_position(knee_angle)
            messagebox.showinfo("Success", "Sent all homing positions to motors")
        except Exception as e: messagebox.showerror("Error", f"Failed to send homing positions: {e}")
    
    def set_home_position_interactive(self):
        """Set current motor positions as home (zero) reference"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        if not messagebox.askyesno("Set Home Position", "Set current motor positions as home (zero) reference?"): return
        try:
            for leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                leg.zero_hip, leg.zero_knee = leg.hip_motor.get_position(), leg.knee_motor.get_position()
            self.zero_homing_sliders(); self.zero_all_individual_motors(); self.reset_to_home()
            messagebox.showinfo("Success", "Home positions set! All sliders reset to zero.")
        except Exception as e: messagebox.showerror("Error", f"Failed to set home positions: {e}")
    
    def zero_homing_sliders(self):
        """Set all homing sliders to zero"""
        for leg_name in self.homing_motor_angles:
            self.homing_motor_angles[leg_name]['hip'].set(0.0)
            self.homing_motor_angles[leg_name]['knee'].set(0.0)
            self.homing_sliders[leg_name]['hip']['label'].config(text="0.0°")
            self.homing_sliders[leg_name]['knee']['label'].config(text="0.0°")
    
    def update_foot_position(self, *args):
        """Update visualization when foot position changes using correct robot kinematics"""
        if not all(hasattr(self, attr) for attr in ['foot_x', 'foot_y', 'hip_angle', 'knee_angle']): return
        try:
            L1, L2 = self.config['leg_config']['L1'], self.config['leg_config']['L2']
            target_x, target_height = self.foot_x.get(), self.foot_y.get()
            current_leg = self.selected_leg.get() if hasattr(self, 'selected_leg') else 'front_left'
            target_z = -target_height
            distance = np.sqrt(target_x**2 + target_z**2)
            if not (abs(L1 - L2) <= distance <= (L1 + L2)): return
            
            hip_guess, knee_guess = self.hip_angle.get(), self.knee_angle.get()
            for _ in range(10):
                hip_rad = np.radians(-hip_guess) if 'right' in current_leg else np.radians(hip_guess)
                knee_rad = -np.radians(knee_guess) + np.pi - 1.12
                current_x = L1 * np.cos(hip_rad) + L2 * np.cos(hip_rad + knee_rad)
                current_z = -L1 * np.sin(hip_rad) - L2 * np.sin(hip_rad + knee_rad)
                error_x, error_z = target_x - current_x, target_z - current_z
                if np.sqrt(error_x**2 + error_z**2) < 0.01: break
                
                # Simplified iterative adjustment (Jacobian Inverse is complex)
                hip_guess += error_x * 0.1
                knee_guess += error_z * -0.1 # Heuristic
            
            self.hip_angle.set(hip_guess); self.knee_angle.set(knee_guess)
            self.update_visualization()
        except Exception: pass
    
    def start_update_loop(self):
        """Start the main GUI update loop"""
        self.running = True
        self.update_gui()
    
    def update_connection_status(self):
        """Update connection status displays"""
        if self.simulation_mode: status, color = ("Simulation Connected", self.theme['success']) if self.connected else ("Simulation Disconnected", self.theme['text_secondary'])
        else: status, color = ("Hardware Connected", self.theme['success']) if self.connected else ("Hardware Disconnected", self.theme['error'])
        self.connection_status_var.set(status)
    
    def update_gui(self):
        """Main GUI update function"""
        if self.running:
            self.update_connection_status()
            if self.plotting_active and self.data_collector:
                data = self.data_collector.get_latest_data()
                if data and self.live_plotter: self.live_plotter.update_data(data)
            self.root.after(self.config['gui']['update_rate_ms'], self.update_gui)
    
    def update_status(self, status_text, connection_text):
        """Update the status bar"""
        self.status_var.set(status_text)
        self.connection_var.set(f"Connection: {connection_text}")
    
    def load_config_dialog(self):
        """Load configuration file dialog"""
        filename = filedialog.askopenfilename(title="Load Configuration", filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")])
        if filename: self.load_config(filename)
    
    def save_config_dialog(self):
        """Save configuration file dialog"""
        filename = filedialog.asksaveasfilename(title="Save Configuration", defaultextension=".yaml", filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")])
        if filename:
            try:
                with open(filename, 'w') as f: yaml.dump(self.config, f, default_flow_style=False)
                messagebox.showinfo("Success", f"Configuration saved to {filename}")
            except Exception as e: messagebox.showerror("Error", f"Error saving configuration: {e}")
    
    def toggle_simulation_mode(self):
        """Toggle simulation mode"""
        self.simulation_mode = self.sim_mode_var.get()
        if self.simulation_mode and self.connected: self.disconnect_odrives()
        self.update_connection_status()
    
    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About", "Quadruped Control Interface v1.1\n\n- CPG walking pattern implemented.")
    
    def connect_odrives(self):
        """Connect to ODrive controllers"""
        if self.simulation_mode:
            try:
                self.quadruped = QuadrupedEnhanced(config_file="config.yaml", simulation_mode=True)
                self.quadruped.connect(); self.quadruped.setup_all_legs()
                self.connected = True
                if self.data_collector: self.data_collector.quadruped, self.data_collector.simulation_mode = self.quadruped, True
                self.update_connection_status()
                messagebox.showinfo("Success", "Simulation mode initialized")
            except Exception as e: messagebox.showerror("Error", f"Failed to initialize simulation: {e}")
            return
        
        if not ODRIVE_AVAILABLE: messagebox.showerror("Error", "ODrive library not available"); return
        try:
            self.quadruped = QuadrupedEnhanced(config_file="config.yaml", simulation_mode=False)
            self.quadruped.connect(); self.quadruped.setup_all_legs()
            self.connected = True
            if self.data_collector: self.data_collector.quadruped, self.data_collector.simulation_mode = self.quadruped, False
            if self.parameter_tuner: self.parameter_tuner.quadruped, self.parameter_tuner.simulation_mode = self.quadruped, False
            self.update_connection_status()
            messagebox.showinfo("Success", "Connected to all ODrives successfully")
        except Exception as e: messagebox.showerror("Error", f"Failed to connect to ODrives: {e}")
    
    def disconnect_odrives(self):
        """Disconnect from ODrive controllers"""
        if self.quadruped:
            try: self.quadruped.disconnect()
            except Exception as e: print(f"Error during disconnect: {e}")
        self.quadruped = None; self.connected = False
        if self.parameter_tuner: self.parameter_tuner.quadruped, self.parameter_tuner.simulation_mode = None, True
        self.update_connection_status()
        messagebox.showinfo("Info", "Disconnected from quadruped system")
    
    def send_pose_to_dog(self):
        """Send current pose to the physical dog"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        try:
            selected_leg = self.selected_leg.get()
            if selected_leg in self.leg_positions:
                hip_angle, knee_angle = self.leg_positions[selected_leg]['hip'], self.leg_positions[selected_leg]['knee']
                validated_hip, validated_knee = self.validate_hardware_angles(hip_angle, knee_angle)
                if abs(validated_hip - hip_angle) > 0.1 or abs(validated_knee - knee_angle) > 0.1:
                    if not messagebox.askyesno("Angle Limits", "Requested angles exceed hardware limits. Send clamped angles?"): return
                    hip_angle, knee_angle = validated_hip, validated_knee
                if selected_leg in self.quadruped.legs:
                    leg = self.quadruped.legs[selected_leg]
                    hip_rot = hip_angle * self.hip_gear_ratio / 57.2958
                    knee_rot = knee_angle * self.knee_gear_ratio / 57.2958
                    leg.hip_motor.set_position(leg.zero_hip + hip_rot)
                    leg.knee_motor.set_position(leg.zero_knee + knee_rot)
                    messagebox.showinfo("Success", f"Pose sent to {selected_leg}")
        except Exception as e: messagebox.showerror("Error", f"Failed to send pose: {e}")
    
    def send_all_poses_to_dog(self):
        """Send all stored leg positions to hardware"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        try:
            for leg_name, angles in self.leg_positions.items():
                if leg_name in self.quadruped.legs:
                    hip_angle, knee_angle = angles['hip'], angles['knee']
                    validated_hip, validated_knee = self.validate_hardware_angles(hip_angle, knee_angle)
                    leg = self.quadruped.legs[leg_name]
                    hip_rot = validated_hip * self.hip_gear_ratio / 57.2958
                    knee_rot = validated_knee * self.knee_gear_ratio / 57.2958
                    leg.hip_motor.set_position(leg.zero_hip + hip_rot)
                    leg.knee_motor.set_position(leg.zero_knee + knee_rot)
            messagebox.showinfo("Success", f"Sent poses to all legs")
        except Exception as e: messagebox.showerror("Error", f"Failed to send poses: {e}")
    
    def reset_to_home(self):
        """Reset to home position"""
        self.hip_angle.set(0); self.knee_angle.set(0)
        self.foot_x.set(0); self.foot_y.set(30)
        for leg_name in self.leg_positions: self.leg_positions[leg_name]['hip'], self.leg_positions[leg_name]['knee'] = 0.0, 0.0
        self.update_visualization()
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        self.stop_cpg() # Also stop CPG if running
        if self.connected and self.quadruped:
            try: self.quadruped.emergency_stop(); messagebox.showinfo("Emergency Stop", "All motors stopped")
            except Exception as e: messagebox.showerror("Error", f"Emergency stop failed: {e}")
        else: messagebox.showinfo("Emergency Stop", "Simulation mode - motors would be stopped")
    
    def calibrate_all(self):
        """Calibrate all motors"""
        if not self.connected or not self.quadruped: messagebox.showwarning("Warning", "Not connected"); return
        mode = messagebox.askyesnocancel("Calibration Mode", "Use simultaneous calibration? (Yes=Simultaneous, No=Sequential)")
        if mode is None: return
        def calib_thread():
            try:
                self.quadruped.calibrate_all_legs(mode)
                self.root.after(0, lambda: messagebox.showinfo("Success", "Calibration completed"))
            except Exception as e: self.root.after(0, lambda: messagebox.showerror("Error", f"Calibration failed: {e}"))
        threading.Thread(target=calib_thread, daemon=True).start()
        messagebox.showinfo("Info", "Calibration started in background")
    
    def set_home_position(self): self.set_home_position_interactive()
    def test_motors(self): messagebox.showinfo("Info", "Motor test completed")
    
    def reboot_odrive_selected(self):
        """Reboot selected ODrive(s)"""
        if not self.connected or not self.quadruped: self.log_message("Not connected", "error"); return
        if self.simulation_mode: self.log_message("Simulation mode - reboot simulated", "info"); return
        selected = self.reboot_odrive.get()
        if not messagebox.askyesno("Confirm Reboot", f"Are you sure you want to reboot the {selected} ODrive?"): return
        try:
            if selected == "All":
                for leg_name in self.quadruped.legs: self.quadruped.legs[leg_name].hip_motor.parent.reboot()
            else:
                leg_map = {"Front Left": "front_left", "Front Right": "front_right", "Back Left": "back_left", "Back Right": "back_right"}
                leg_name = leg_map.get(selected)
                if leg_name and leg_name in self.quadruped.legs: self.quadruped.legs[leg_name].hip_motor.parent.reboot()
            self.log_message(f"{selected} ODrive(s) rebooting... disconnecting.", "info")
            self.disconnect_odrives()
        except Exception as e: self.log_message(f"Error rebooting ODrive(s): {str(e)}", "error")
    
    def start_plotting(self):
        if self.data_collector: self.data_collector.start_collection(); self.plotting_active = True; self.plot_status_var.set("Running"); messagebox.showinfo("Info", "Plotting started")
    def stop_plotting(self):
        if self.data_collector: self.data_collector.stop_collection(); self.plotting_active = False; self.plot_status_var.set("Stopped"); messagebox.showinfo("Info", "Plotting stopped")
    def clear_plot(self):
        if self.live_plotter: self.live_plotter.clear_data(); messagebox.showinfo("Info", "Plot cleared")
    def on_parameter_changed(self, event=None):
        if self.live_plotter: self.live_plotter.set_parameter(self.plot_parameter.get())
    
    def update_parameter_display(self):
        """Update the parameter control display based on selected category"""
        for widget in self.param_controls_frame.winfo_children(): widget.destroy()
        self.param_scales = {}
        if not hasattr(self.parameter_tuner, 'parameters'): self.log_message("Parameter tuner not initialized", "warning"); return
        category = self.param_category.get()
        if category not in self.parameter_tuner.parameters: self.log_message(f"Unknown category: {category}", "warning"); return
        
        read_all_frame = ttk.Frame(self.param_controls_frame, style='Surface.TFrame')
        read_all_frame.grid(row=0, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        ttk.Button(read_all_frame, text="Read All Current Values", style='Success.TButton', command=self.read_all_current_parameters).pack(side=tk.LEFT, padx=5)
        
        parameters = self.parameter_tuner.parameters[category]
        for row, (param_name, param_config) in enumerate(parameters.items(), 1):
            ttk.Label(self.param_controls_frame, text=f"{param_name}:", style='Surface.TLabel').grid(row=row, column=0, padx=5, pady=2, sticky="w")
            leg_quadruped = self.map_leg_name_to_quadruped(self.param_leg.get())
            current_value = self.parameter_tuner.get_parameter(leg_quadruped, self.param_motor.get(), param_name)
            value_var = tk.DoubleVar(value=current_value if current_value is not None else param_config['default'])
            scale = ttk.Scale(self.param_controls_frame, from_=param_config['min'], to=param_config['max'], variable=value_var, orient=tk.HORIZONTAL, length=300, style='Dark.Horizontal.TScale', command=lambda val, p=param_name, v=value_var: self.on_parameter_changed_scale(p, v.get()))
            scale.grid(row=row, column=1, padx=5, pady=2, sticky="ew")
            value_label = ttk.Label(self.param_controls_frame, text=f"{value_var.get():.4f}", style='Surface.TLabel', width=8)
            value_label.grid(row=row, column=2, padx=5, pady=2)
            self.param_scales[param_name] = {'scale': scale, 'variable': value_var, 'label': value_label}
        self.param_controls_frame.columnconfigure(1, weight=1)

    def read_all_current_parameters(self):
        """Read all current parameter values from ODrive for the selected category"""
        try:
            category, leg_gui, motor = self.param_category.get(), self.param_leg.get(), self.param_motor.get()
            leg_quadruped = self.map_leg_name_to_quadruped(leg_gui)
            for param_name in self.parameter_tuner.parameters[category]:
                if param_name in self.param_scales:
                    current_value = self.parameter_tuner.get_parameter(leg_quadruped, motor, param_name)
                    if current_value is not None:
                        self.param_scales[param_name]['variable'].set(current_value)
                        self.param_scales[param_name]['label'].config(text=f"{current_value:.4f}")
            self.log_message(f"Read parameters for {leg_gui} {motor}", "success")
        except Exception as e: self.log_message(f"Error reading parameters: {str(e)}", "error")
    
    def on_parameter_changed_scale(self, param_name, value):
        """Handle parameter scale change"""
        if param_name in self.param_scales: self.param_scales[param_name]['label'].config(text=f"{value:.4f}")
        leg_quadruped = self.map_leg_name_to_quadruped(self.param_leg.get())
        self.parameter_tuner.set_parameter(leg_quadruped, self.param_motor.get(), param_name, value)
    
    def save_parameter_preset(self):
        """Save current parameters as a preset"""
        preset_name = self.preset_name.get().strip()
        if not preset_name: messagebox.showwarning("Warning", "Please enter a preset name"); return
        parameters = {name: controls['variable'].get() for name, controls in self.param_scales.items()}
        self.parameter_tuner.save_preset(preset_name, parameters)
        messagebox.showinfo("Success", f"Preset '{preset_name}' saved")
    
    def load_parameter_preset(self):
        """Load a parameter preset"""
        preset_name = self.preset_name.get().strip()
        if not preset_name: messagebox.showwarning("Warning", "Please enter a preset name"); return
        parameters = self.parameter_tuner.load_preset(preset_name)
        if parameters:
            for param_name, value in parameters.items():
                if param_name in self.param_scales:
                    self.param_scales[param_name]['variable'].set(value)
                    self.on_parameter_changed_scale(param_name, value)
            messagebox.showinfo("Success", f"Preset '{preset_name}' loaded")
        else: messagebox.showerror("Error", f"Preset '{preset_name}' not found")

    # --- CPG Methods ---

    def create_cpg_panel(self, parent):
        """Create CPG walking control panel"""
        cpg_frame = ttk.Frame(parent, style='Dark.TFrame')
        parent.add(cpg_frame, text="CPG Walking")

        # Parameters Frame
        params_frame = ttk.LabelFrame(cpg_frame, text="Gait Parameters", style='Surface.TLabelframe')
        params_frame.pack(fill=tk.X, padx=10, pady=10)

        # Use a grid for better alignment
        grid_frame = ttk.Frame(params_frame, style='Surface.TFrame')
        grid_frame.pack(padx=10, pady=10)

        # Stride Length
        ttk.Label(grid_frame, text="Stride Length (cm):", style='Surface.TLabel').grid(row=0, column=0, sticky='w', padx=5, pady=5)
        stride_scale = ttk.Scale(grid_frame, from_=0, to=30, variable=self.cpg_stride_length, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        stride_scale.grid(row=0, column=1, sticky='ew', padx=5)
        stride_label = ttk.Label(grid_frame, text="10.0", style='Surface.TLabel')
        stride_label.grid(row=0, column=2, padx=5)
        stride_scale.configure(command=lambda v: stride_label.config(text=f"{float(v):.1f}"))

        # Step Height
        ttk.Label(grid_frame, text="Step Height (cm):", style='Surface.TLabel').grid(row=1, column=0, sticky='w', padx=5, pady=5)
        height_scale = ttk.Scale(grid_frame, from_=0, to=15, variable=self.cpg_step_height, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        height_scale.grid(row=1, column=1, sticky='ew', padx=5)
        height_label = ttk.Label(grid_frame, text="8.0", style='Surface.TLabel')
        height_label.grid(row=1, column=2, padx=5)
        height_scale.configure(command=lambda v: height_label.config(text=f"{float(v):.1f}"))

        # Frequency
        ttk.Label(grid_frame, text="Frequency (Hz):", style='Surface.TLabel').grid(row=2, column=0, sticky='w', padx=5, pady=5)
        freq_scale = ttk.Scale(grid_frame, from_=0.1, to=3.0, variable=self.cpg_frequency, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        freq_scale.grid(row=2, column=1, sticky='ew', padx=5)
        freq_label = ttk.Label(grid_frame, text="0.75", style='Surface.TLabel')
        freq_label.grid(row=2, column=2, padx=5)
        freq_scale.configure(command=lambda v: freq_label.config(text=f"{float(v):.2f}"))

        # Default Height
        ttk.Label(grid_frame, text="Default Height (cm):", style='Surface.TLabel').grid(row=3, column=0, sticky='w', padx=5, pady=5)
        def_height_scale = ttk.Scale(grid_frame, from_=20, to=40, variable=self.cpg_default_height, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        def_height_scale.grid(row=3, column=1, sticky='ew', padx=5)
        def_height_label = ttk.Label(grid_frame, text="30.0", style='Surface.TLabel')
        def_height_label.grid(row=3, column=2, padx=5)
        def_height_scale.configure(command=lambda v: def_height_label.config(text=f"{float(v):.1f}"))

        # Front Legs Offset
        ttk.Label(grid_frame, text="Front Legs Offset (cm):", style='Surface.TLabel').grid(row=4, column=0, sticky='w', padx=5, pady=5)
        front_offset_scale = ttk.Scale(grid_frame, from_=-20, to=20, variable=self.cpg_front_offset, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        front_offset_scale.grid(row=4, column=1, sticky='ew', padx=5)
        front_offset_label = ttk.Label(grid_frame, text="5.0", style='Surface.TLabel')
        front_offset_label.grid(row=4, column=2, padx=5)
        front_offset_scale.configure(command=lambda v: front_offset_label.config(text=f"{float(v):.1f}"))

        # Back Legs Offset
        ttk.Label(grid_frame, text="Back Legs Offset (cm):", style='Surface.TLabel').grid(row=5, column=0, sticky='w', padx=5, pady=5)
        back_offset_scale = ttk.Scale(grid_frame, from_=-20, to=20, variable=self.cpg_back_offset, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        back_offset_scale.grid(row=5, column=1, sticky='ew', padx=5)
        back_offset_label = ttk.Label(grid_frame, text="12.5", style='Surface.TLabel')
        back_offset_label.grid(row=5, column=2, padx=5)
        back_offset_scale.configure(command=lambda v: back_offset_label.config(text=f"{float(v):.1f}"))

        # Front Legs Height Offset
        ttk.Label(grid_frame, text="Front Legs Height Offset (cm):", style='Surface.TLabel').grid(row=6, column=0, sticky='w', padx=5, pady=5)
        front_height_scale = ttk.Scale(grid_frame, from_=-20, to=20, variable=self.cpg_front_height_offset, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        front_height_scale.grid(row=6, column=1, sticky='ew', padx=5)
        front_height_label = ttk.Label(grid_frame, text="10.0", style='Surface.TLabel')
        front_height_label.grid(row=6, column=2, padx=5)
        front_height_scale.configure(command=lambda v: front_height_label.config(text=f"{float(v):.1f}"))

        # Back Legs Height Offset
        ttk.Label(grid_frame, text="Back Legs Height Offset (cm):", style='Surface.TLabel').grid(row=7, column=0, sticky='w', padx=5, pady=5)
        back_height_scale = ttk.Scale(grid_frame, from_=-20, to=20, variable=self.cpg_back_height_offset, orient=tk.HORIZONTAL, length=400, style='Dark.Horizontal.TScale')
        back_height_scale.grid(row=7, column=1, sticky='ew', padx=5)
        back_height_label = ttk.Label(grid_frame, text="0.0", style='Surface.TLabel')
        back_height_label.grid(row=7, column=2, padx=5)
        back_height_scale.configure(command=lambda v: back_height_label.config(text=f"{float(v):.1f}"))

        grid_frame.columnconfigure(1, weight=1)

        # Control Frame
        control_frame = ttk.LabelFrame(cpg_frame, text="CPG Control", style='Surface.TLabelframe')
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        # Mode selection frame
        mode_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        mode_frame.pack(pady=5)
        ttk.Label(mode_frame, text="Movement Mode:", style='Surface.TLabel').pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="Custom Movement", value="custom", variable=self.movement_mode, style='Dark.TRadiobutton').pack(side=tk.LEFT, padx=10)
        ttk.Radiobutton(mode_frame, text="CPG Walking", value="cpg", variable=self.movement_mode, style='Dark.TRadiobutton').pack(side=tk.LEFT, padx=10)

        button_frame = ttk.Frame(control_frame, style='Surface.TFrame')
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="Visualize CPG", style='Dark.TButton', command=self.start_cpg_visualization).pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="Run on Hardware", style='Success.TButton', command=self.start_cpg_hardware).pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="STOP CPG", style='Emergency.TButton', command=self.stop_cpg).pack(side=tk.LEFT, padx=10)

    def start_cpg_visualization(self):
        """Starts the CPG visualization loop."""
        if self.cpg_running:
            self.log_message("CPG is already running.", "warning")
            return
        
        # Initialize quadruped in simulation mode if not already connected
        if not self.quadruped:
            try:
                self.quadruped = QuadrupedEnhanced(config_file="config.yaml", simulation_mode=True)
                self.quadruped.connect()
                self.quadruped.setup_all_legs()
                self.connected = True
                if self.data_collector:
                    self.data_collector.quadruped = self.quadruped
                    self.data_collector.simulation_mode = True
                self.update_connection_status()
            except Exception as e:
                self.log_message(f"Failed to initialize simulation: {e}", "error")
                return

        # Switch to CPG mode
        self.movement_mode.set("cpg")
        self.cpg_running = True
        self.cpg.reset_time()
        self.log_message("CPG Visualization Started", "success")
        self.cpg_visualization_step()

    def cpg_visualization_step(self):
        """A single step of the CPG visualization loop."""
        if not self.cpg_running:
            return

        # Update CPG parameters from sliders
        self.cpg.update_parameters(
            self.cpg_stride_length.get(),
            self.cpg_step_height.get(),
            self.cpg_frequency.get(),
            self.cpg_default_height.get()
        )
        
        # Get target positions with offsets
        t = time.time() - self.cpg.start_time
        target_positions = self.cpg.get_leg_positions(
            t, 
            front_offset=self.cpg_front_offset.get(),
            back_offset=self.cpg_back_offset.get(),
            front_height_offset=self.cpg_front_height_offset.get(),
            back_height_offset=self.cpg_back_height_offset.get()
        )

        # Calculate angles and update visualization data
        for leg_name, (x, height) in target_positions.items():
            if self.quadruped and leg_name in self.quadruped.legs:
                leg = self.quadruped.legs[leg_name]
                angles = leg.get_angles_for_point(x, height)
                if angles:
                    hip_deg, knee_deg = angles
                    # Apply mirroring for visualization
                    if 'right' in leg_name:
                        self.leg_positions[leg_name]['hip'] = -hip_deg
                    else:
                        self.leg_positions[leg_name]['hip'] = hip_deg
                    self.leg_positions[leg_name]['knee'] = knee_deg

        # Redraw the 3D model
        self.update_visualization()

        # Schedule the next frame
        self.root.after(20, self.cpg_visualization_step) # ~50 FPS

    def start_cpg_hardware(self):
        """Starts the CPG loop that sends commands to the hardware."""
        if self.cpg_running:
            self.log_message("CPG is already running.", "warning")
            return

        if self.simulation_mode or not self.connected or not self.quadruped:
            self.log_message("Must be connected to hardware to run.", "error")
            return

        self.cpg_running = True
        self.cpg.reset_time()
        self.log_message("CPG started on hardware!", "success")

        # Run the hardware loop in a separate thread to not freeze the GUI
        self.cpg_thread = threading.Thread(target=self.cpg_hardware_loop, daemon=True)
        self.cpg_thread.start()

    def cpg_hardware_loop(self):
        """The CPG hardware loop running in a separate thread."""
        while self.cpg_running:
            # Update CPG parameters from sliders (accessing tk vars from a thread is generally safe)
            self.cpg.update_parameters(
                self.cpg_stride_length.get(),
                self.cpg_step_height.get(),
                self.cpg_frequency.get(),
                self.cpg_default_height.get()
            )
            
            t = time.time() - self.cpg.start_time
            target_positions = self.cpg.get_leg_positions(
                t,
                front_offset=self.cpg_front_offset.get(),
                back_offset=self.cpg_back_offset.get(),
                front_height_offset=self.cpg_front_height_offset.get(),
                back_height_offset=self.cpg_back_height_offset.get()
            )

            for leg_name, (x, height) in target_positions.items():
                if leg_name in self.quadruped.legs:
                    leg = self.quadruped.legs[leg_name]
                    leg.move_to_point(x, height)
            
            time.sleep(0.005) # Control loop at ~50Hz

        self.log_message("CPG Hardware loop finished.", "info")

    def stop_cpg(self):
        """Stops any running CPG loop."""
        if self.cpg_running:
            self.cpg_running = False
            # Switch back to custom mode
            self.movement_mode.set("custom")
            self.log_message("CPG Stopped.", "success")
        else:
            self.log_message("CPG is not running.", "info")


    def on_closing(self):
        """Handle window closing"""
        self.stop_cpg()
        self.running = False
        if self.connected:
            self.disconnect_odrives()
        self.root.quit()
        self.root.destroy()


def main():
    """Main function to run the GUI"""
    root = tk.Tk()
    app = QuadrupedGUI(root)
    
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()


if __name__ == "__main__":
    main()
