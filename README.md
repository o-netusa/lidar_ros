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

## PointCloud2 to PCL PointCloud Conversion

Convert `sensor_msgs::PointCloud2` objects coming from the velodyne_pointcloud driver to `pcl::PointCloud<onet::lidar_ros::PointXYZIRGBT>` objects.

The `sensor_msgs::PointCloud2` ROS message is constructed by using the `onet::lidar_ros::PointXYZIRGBT` container.
Both define the following channels:

* x - The x coord in Cartesian coordinates
* y - The y coord in Cartesian coordinates
* z - The z coord in Cartesian coordinates
* intensity - The measured intensity at the point
* r - RGB information at the point (calculated from intensity)
* g - RGB information at the point (calculated from intensity)
* b - RGB information at the point (calculated from intensity)
* utc_time - The time stamp (UTC) of the measured point
* ms_time - The time stamp (millisecond) of the measured point

To convert a `sensor_msgs::PointCloud2` published by the velodyne driver to PCL you could use the following code:

```
#include <pcl_conversions/pcl_conversions.h>
#include <lidar_ros_driver/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud){

  pcl::PCLPointCloud2 pcl_pointcloud2;
  pcl_conversions::toPCL(*cloud, pcl_pointcloud2);
  pcl::PointCloud<onet::lidar_ros::PointXYZIRGBT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<onet::lidar_ros::PointXYZIRGBT>);
  pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

  // use pcl_cloud_ptr
}
```

## How to use time stamp of each point
* utc_time - The time stamp (UTC) of the measured point
* ms_time - The time stamp (millisecond) of the measured point

```

#include <boost/date_time/gregorian/gregorian.hpp>
#include <ctime>

boost::gregorian::date now_date = boost::gregorian::day_clock::universal_day();
        struct tm now_tm = boost::gregorian::to_tm(now_date);
        now_tm.tm_hour = (point.utc >> 12) + 8;
        now_tm.tm_min = (point.utc & 0xFC0) >> 6;
        now_tm.tm_sec = point.utc & 0x3F;
        double now_sec = (mktime(&now_tm) * 1000000 + (point.time_stamp >> 10) * 1000 +
                        (point.time_stamp & 0x3FF)) /
                        1000000.0;
```