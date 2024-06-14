#####
# Launch Radar Node
# Open terminal

ssh ubuntu@192.168.2.222

# the password is turtlebot4

ll /dev/serial/by-id
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

# Wait 10 seconds

cd ~/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch
ros2 launch 1843_Standard.py


# Launch Broadcaster Node
# Open terminal

ssh ubuntu@192.168.2.222

# the password is turtlebot4

cd ~
cd broadcaster_ws/
. install/setup.bash
ros2 launch tf_broadcaster tf_broadcaster.launch.py


# Launch MapGen Node
# Open terminal

ssh ubuntu@192.168.2.222

# the password is turtlebot4
cd ~/map_gen_ws/
ros2 launch octomap_server2 octomap_server_launch.py


# Visualizer
ros2 launch turtlebot4_viz view_model.launch.py

#
