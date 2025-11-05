"""
Quick test to verify gravity is working
"""
import genesis as gs
import math

# Initialize Genesis
gs.init()

# Create scene
scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.02, substeps=2),
    viewer_options=gs.options.ViewerOptions(
        max_FPS=60,
        camera_pos=(2.0, -2.0, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
    ),
    show_viewer=True,
)

# Add plane
plane = scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

# Add Q1 robot at HIGH position to see if it falls
robot = scene.add_entity(
    gs.morphs.URDF(
        file="urdf/q1/kutta.urdf",
        pos=(0, 0, 2.0),  # Spawn 2 meters high!
        quat=(math.cos(math.pi/4), -math.sin(math.pi/4), 0, 0),
    ),
)

# Build scene
scene.build()

print("Robot spawned at height 2.0m")
print("If gravity is working, robot should fall down")
print("Press Ctrl+C to exit")

# Run simulation and check position
for i in range(500):
    scene.step()
    
    if i % 50 == 0:
        pos = robot.get_pos()
        # Handle both batched and non-batched position tensors
        height = pos[2] if pos.dim() == 1 else pos[0, 2]
        print(f"Step {i:3d}: Robot height = {height:.3f}m")
