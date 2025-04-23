
#!/bin/bash

# Loop over all sessions
tmux list-sessions -F '#S' | while read session; do
  # Loop over all panes in each session
  tmux list-panes -t "$session" -a -F '#{session_name}:#{window_index}.#{pane_index}' | while read pane; do
    echo "Sending Ctrl+C to $pane"
    tmux send-keys -t "$pane" C-c
  done
done

# Give processes a moment to react to SIGINT
sleep 3

# Kill the tmux server after Ctrl+C is sent to all panes
echo "Killing tmux server..."
tmux kill-server