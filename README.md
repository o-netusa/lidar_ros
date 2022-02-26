# O-Net LiDAR ROS Driver

## Build within Docker container
* Start Docker container: `/opt/share/ros/start_ros_docker.sh $(pwd)`
* Inside the Docker container, cd into the repo directory and execute
```
source /opt/ros/melodic/setup.bash
catkin_make
```

## Build without Docker container
* cd into the repo directory and execute
```
source /opt/ros/melodic/setup.bash
catkin_make
```

## Launch the ROS node
* After catkin_make, set environment using `source devel/setup.bash` and execute `roslaunch lidar_ros_driver run.launch`
* If `save_bag` set to `True`, the saved bag file can be found in `devel/lib/lidar_ros_driver/test.bag`

## Create install package
* Execute `catkin_make install` and package the `install` directory
