#!/bin/bash

SESSION="fly_drone"

VENV_ACTIVATE="/home/parth/rl_atom_bomb/bin/activate"  # Adjust path if needed

# Kill existing session if it exists.
tmux kill-session -t "$SESSION" 2>/dev/null

# Start a new detached session.
tmux new-session -d -s "$SESSION"

# 1. PX4: Rename the initial window and run PX4 SITL with environment variables.
tmux rename-window -t "$SESSION":0 'PX4'
tmux send-keys -t "$SESSION":0 "source ${VENV_ACTIVATE} && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make -C /home/parth/PX4-Autopilot px4_sitl gz_x500_mono_cam" C-m

# 2. MAVROS: Create a new window and launch MAVROS after a delay.
tmux new-window -t "$SESSION" -n 'MAVROS'
tmux send-keys -t "$SESSION":1 "source ${VENV_ACTIVATE} && sleep 10 && ros2 launch mavros px4.launch fcu_url:=udp://:14540@" C-m

# 3. Camera Bridge: Create a new window and run the camera bridge command.
tmux new-window -t "$SESSION" -n 'CameraBridge'
tmux send-keys -t "$SESSION":2 "source ${VENV_ACTIVATE} && sleep 15 && ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo --ros-args -r /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image:=/camera/left/image -r /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info:=/camera/left/camera_info -r /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image:=/camera/right/image -r /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info:=/camera/right/camera_info -r /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/image:=/camera/down/image -r /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/camera_info:=/camera/down/camera_info" C-m

# 4. Send velocity: Create a new window to run send_velocity.py.
tmux new-window -t "$SESSION" -n 'Velocity'
tmux send-keys -t "$SESSION":3 "source ${VENV_ACTIVATE} && sleep 16 && python3 /home/parth/Desktop/fine_1/fine/send_velocity.py" C-m

# 5. Set OFFBOARD mode: Create a new window and call the service after delay.
tmux new-window -t "$SESSION" -n 'OFFBOARD_Mode'
tmux send-keys -t "$SESSION":4 "source ${VENV_ACTIVATE} && sleep 18 && ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\"" C-m

# 6. Arm Drone: Create a new window to arm the drone.
tmux new-window -t "$SESSION" -n 'Arming'
tmux send-keys -t "$SESSION":5 "source ${VENV_ACTIVATE} && sleep 20 && ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\"" C-m

# 7. Run combined_left_yolo.py: Create a new window.
tmux new-window -t "$SESSION" -n 'YOLO'
tmux send-keys -t "$SESSION":6 "source ${VENV_ACTIVATE} && sleep 24 && python3 /home/parth/Desktop/fine_1/fine/combined_left_yolo.py" C-m

# 8. Run collision detection: Create a new window.
tmux new-window -t "$SESSION" -n 'Collision'
tmux send-keys -t "$SESSION":7 "source ${VENV_ACTIVATE} && sleep 35 && python3 /home/parth/Desktop/fine_1/fine/collision.py" C-m

# Attach to the tmux session.
tmux attach-session -t "$SESSION"
