#!/bin/bash
# Quick test script for the exploration environment

echo "========================================"
echo "CEERI Drone Exploration - Quick Test"
echo "========================================"
echo ""

# Check if we're in the right directory
if [ ! -f "exploration_env.py" ]; then
    echo "Error: Please run this script from the CEERI directory"
    echo "cd /home/ritwik/NUS/Genesis/examples/CEERI"
    exit 1
fi

# Check dependencies
echo "Checking dependencies..."
python3 -c "import genesis; import torch; import pynput" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Warning: Some dependencies missing. Install with:"
    echo "  pip install genesis-world torch pynput"
    echo ""
fi

echo "Starting teleoperation test..."
echo ""
echo "Controls:"
echo "  Arrow keys: Move"
echo "  A/D: Rotate"
echo "  Space: Stop"
echo "  R: Reset"
echo "  ESC: Quit"
echo ""
echo "Press Enter to continue..."
read

python3 exploration_teleop.py

echo ""
echo "Test complete!"
