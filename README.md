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


