# Camera-Drones-2021

## Setup Project

1.) Create catkin workspace
```
mkdir -p ~/camera_drones_ws/src   
cd ~/camera_drones_ws/
catkin build
```

2.) Clone repository into "src" folder
```
cd ~/camera_drones_ws/src
git clone git@github.com:jonasniedermair/Camera-Drones-2021.git
```

3.) Install required dependencies
```
sudo apt install ros-melodic-octomap-ros
```
4.) Build packages
```
cd ~/camera_drones_ws
catkin build
```

## Run 1. Assignment
To run the 1. Assignment execute these commands in seperate terminal windows (Dont forget to source your workspace "source devel/setup.bash").

Run astra_camera:
```
roslaunch astra_camera astra.launch load_driver:=false publish_tf:=true
```

Run octomap_server:
```
roslaunch octomap_server octomap_mapping.launch
```

Start RViz and load configuration file (File -> Open Config -> Select rviz_config.rviz)
```
rviz
```

Play rosbag (Can be downloaded here: https://files.icg.tugraz.at/f/fb3643f14540402895d5/)
```
rosbag play [rosbag]
```

After executing these commands, you should see the creation of the octomap in RViz.

You can save the created octomap as binary, by executing the command:
```
rosrun octomap_server octomap_saver -f octomap.bt
```

## Run 2. Assignment
To run the 2. Assignment execute these commands in seperate terminal windows:

Run octomap_mapping:
```
roslaunch dla2_path_planner octomap_mapping_a2.launch 
```

Run path planner:
```
roslaunch dla2_path_planner path_planning.launch
```

Set up the starting point
```
rostopic pub current_position geometry_msgs/Point -- 0.0 0.0 3.0
```

Set up the end point
```
rostopic pub goal_position geometry_msgs/Point -- 2.0 -27.0 15.0
```

NOTE: Run the commands above in sequence!

## Run 3. Assignment

Run octomap_mapping:
```
roslaunch dla2_path_planner octomap_mapping_a2.launch 
```

Run path planner:
```
roslaunch dla2_path_planner path_planning.launch
```

Run trajectory_visualization of the raw path:
```
roslaunch dla2_path_planner trajectory_visualization.launch 
```

Run trajectory_visualization of the simplified path:
```
roslaunch dla2_path_planner trajectory_visualization.launch node_name:="trajectory_visualization" input_topic:="planned_trajectory" output_topic:="trajectory_markers" marker_color:="W"

```

Set up the starting point
```
rostopic pub current_position geometry_msgs/Point -- 0.0 0.0 3.0
```

Set up the end point
```
rostopic pub goal_position geometry_msgs/Point -- 2.0 -27.0 15.0
```

NOTE: Run the commands above in sequence!



