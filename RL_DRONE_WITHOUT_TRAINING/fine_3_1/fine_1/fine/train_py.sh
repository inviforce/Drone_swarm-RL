#!/bin/bash
# train_py.sh
# This script launches your training (train.py) directly without tmux

VENV_ACTIVATE="/home/c3ilab/Desktop/rl_mini_pro/final_rl_minipro/rl_atom_bomb/bin/activate"
TRAIN_SCRIPT="/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/rl/train.py"

# Activate the virtual environment
source "$VENV_ACTIVATE"

# Run the training script
python3 "$TRAIN_SCRIPT"
