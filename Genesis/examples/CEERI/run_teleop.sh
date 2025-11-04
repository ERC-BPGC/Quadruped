#!/bin/bash
# Simple script to run the teleop with the correct environment

source ~/anaconda3/etc/profile.d/conda.sh
conda activate genesis
python exploration_teleop.py --cpu
