#!/bin/bash

SESSION="px4_sim"
VENV_ACTIVATE="/home/c3ilab/Desktop/final_rl_minipro/rl_atom_bomb"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Start a new session
tmux new-session -d -s $SESSION

# 1. PX4
tmux rename-window -t $SESSION 'PX4'
tmux send-keys -t $SESSION "$VENV_ACTIVATE && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make -C /home/c3ilab/PX4-Autopilot px4_sitl gz_x500_mono_cam" C-m
# Sleep to ensure PX4 is ready before launching MAVROS
tmux send-keys -t $SESSION "sleep 30" C-m

# 2. MAVROS
tmux new-window -t $SESSION -n 'MAVROS'
tmux send-keys -t $SESSION:1 "sleep 15 && $VENV_ACTIVATE && ros2 launch mavros px4.launch fcu_url:=udp://:14540@" C-m
# Sleep to ensure MAVROS is ready before camera bridge
tmux send-keys -t $SESSION:1 "sleep 30" C-m

# 3. Camera Bridge
tmux new-window -t $SESSION -n 'CameraBridge'
tmux send-keys -t $SESSION:2 "sleep 25 && $VENV_ACTIVATE && ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo --ros-args -r /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image:=/camera/left/image -r /world/default/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info:=/camera/left/camera_info -r /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image:=/camera/right/image -r /world/default/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info:=/camera/right/camera_info -r /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/image:=/camera/down/image -r /world/default/model/x500_mono_cam_0/model/down_cam/link/camera_link/sensor/imager/camera_info:=/camera/down/camera_info" C-m
# Sleep to ensure Camera Bridge is ready before sending velocity
tmux send-keys -t $SESSION:2 "sleep 20" C-m

# 4. Send velocity
tmux new-window -t $SESSION -n 'Velocity'
tmux send-keys -t $SESSION:3 "sleep 35 && $VENV_ACTIVATE && python3 /home/c3ilab/Desktop/fine/send_velocity.py" C-m
# Sleep to ensure velocity is sent before changing mode
tmux send-keys -t $SESSION:3 "sleep 20" C-m

# 5. Set OFFBOARD mode
tmux new-window -t $SESSION -n 'OFFBOARD_Mode'
tmux send-keys -t $SESSION:4 "sleep 40 && $VENV_ACTIVATE && ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: \"OFFBOARD\"}'" C-m
# Sleep to ensure OFFBOARD mode is set before arming
tmux send-keys -t $SESSION:4 "sleep 10" C-m

# 6. Arm Drone
tmux new-window -t $SESSION -n 'Arming'
tmux send-keys -t $SESSION:5 "sleep 45 && $VENV_ACTIVATE && ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'" C-m
# Sleep to ensure the drone is armed before running YOLO
tmux send-keys -t $SESSION:5 "sleep 10" C-m

# 7. Run combined_left_yolo.py
tmux new-window -t $SESSION -n 'YOLO'
tmux send-keys -t $SESSION:6 "sleep 20 && $VENV_ACTIVATE && python3 /home/c3ilab/Desktop/fine/combined_left_yolo.py" C-m
# Sleep to ensure YOLO is running before collision detection
tmux send-keys -t $SESSION:6 "sleep 30" C-m

# 8. Run collision detection
tmux new-window -t $SESSION -n 'Collision'
tmux send-keys -t $SESSION:7 "sleep 20 && $VENV_ACTIVATE && python3 /home/c3ilab/Desktop/fine/collision.py" C-m
# Sleep to ensure collision detection is running before training
tmux send-keys -t $SESSION:7 "sleep 20" C-m

# 9. Run train.py
# tmux new-window -t $SESSION -n 'Training'
# tmux send-keys -t $SESSION:8 "sleep 20 && $VENV_ACTIVATE && python3 /home/c3ilab/Desktop/fine/rl/train.py" C-m

# Attach to the session
tmux attach-session -t $SESSION
