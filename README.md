# O-Net LiDAR ROS Driver

## Build steps
* Start Docker container: `/opt/share/ros/start_ros_docker.sh $(pwd)`
* Inside the Docker container, cd into the repo directory and execute `catkin_make`

## Create install package
* `catkin_make install`
* `target -zcf onet_lidar_ros_dirver.tar.gz install`

## Launch the ROS node
* Extract the package and cd into the directory
* `roslaunch launch/run.launch`
