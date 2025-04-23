#!/bin/bash

FLY_LAUNCH_SCRIPT="/home/c3ilab/Desktop/fine_3_1/fine_3_1/fine_1/fine/fly_drone.sh"

# --- 1. Send Ctrl+C (SIGINT) to all tmux panes in all sessions ---
tmux list-sessions -F '#S' | while read session; do
  # Loop over all panes in each session
  tmux list-panes -t "$session" -a -F '#{session_name}:#{window_index}.#{pane_index}' | while read pane; do
    echo "Sending Ctrl+C to $pane"
    tmux send-keys -t "$pane" C-c
  done
done

# Give processes a moment to react to SIGINT
sleep 3

# --- 2. Forcefully kill Gazebo, PX4, etc. if they are still around ---
echo "Checking and force-killing gzserver, gzclient, px4..."
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 px4 2>/dev/null || true

# --- 3. Kill the tmux server ---
echo "Killing tmux server..."
tmux kill-server

# Wait for processes to fully reset
sleep 10

# --- 4. Restart your script ---
echo "Restarting fly_drone with script: $FLY_LAUNCH_SCRIPT"
bash "$FLY_LAUNCH_SCRIPT"

sleep 45
