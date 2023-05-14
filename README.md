# Cepton2 ROS

## 0.0 Usage
After catkin_make and source, use the following launch file to start the driver:
```sh
roslaunch cepton2_ros driver.launch
```

## 1.0 - Overview

This ROS package provides support for Cepton LiDAR with SDK Version >= 2.0.

The ROS package for Cepton LiDAR with SDK Version <2.0 is located at:
https://github.com/ceptontech/cepton_sdk_redist/tree/master/ros

## 2.0 - Installation

- Install ROS and create catkin workspace

- Change to the catkin_ws directory.

- Clone/copy the package into catkin_ws/src/.

  - Path to this README.md file would be catkin_ws/src/cepton2_ros/README.md

- Run catkin_make.
```sh
catkin_make
```

- Source the catkin setup script.
```sh
source ./devel/setup.bash
```

## 3.0 - Getting Started


### 3.1 - Launching the driver


To use the driver standalone, you must first launch the manager nodelet
```sh
roslaunch cepton2_ros manager.launch
```

Then you can launch the Publisher nodelet with the default yaml config file path (default is default_params.yaml)
```sh
roslaunch cepton2_ros publisher.launch
```

This loads the parameters from cepton2_ros/config/default_params.yaml into the rosparam server and starts the publisher nodelet to begin publishing point data and sensor information from a live sensor.

You may also define your own yaml config file and pass in as an argument:
```sh
roslaunch cepton2_ros publisher.launch config_path:=<path_to_file>
```

The YAML config is described in Section 4.0.

### 3.2 - Visualizing the point cloud

The driver publishes sensor_msgs/PointCloud2.h messages which can be visualized in RViz. Currently the frame_id is set to "cepton2" and not configurable, but this may be changed upon request.

Once the publisher is launched, you can open up RViz, add a new PointCloud2 display and configure the topic to match the publisher data. You will need to also configure the "Fixed Frame" under "Global Options" to be "cepton2".

To see what topics are being published use:
```sh
rostopic list
```

NOTE: the channel name of the PointCloud2 display should be changed from "intensity" to "reflectivity" in order to visualize the reflectivity with each point.

## 4.0 - Configuration Parameter Arguments

- capture_path: path to the pcap capture file, if no path given the nodelet will begin networking thread for live sensors

- capture_loop: true - replay will loop (default), false - replay will run once and stop

- points_topic_prefix: configurable prefix for the topic publishing points (default: cepton2/points)

- info_topic_prefix: configurable prefix for the topic publishing sensor information (default: cepton2/sensor_information)

## 5.0 - Points Topic
The nodelet will publish sensor_msgs/PointCloud2.h messages each containing an array of points with the following structure:

```sh
struct CustomCeptonPoint {
    float x
    float y,
    float z,
    float reflectivity,
    uint8_t relative_timestamp,
    uint8_t flags,
    uint8_t channel_id,
    uint8_t valid
}
```
- x, y, z - cartestian coordinates of the point in meters.

- reflectivity - reflectivity measurement from 0-255%

- relative_timestamp - time in ns from the previous laser firing

- flags - bitwise flags, see include/cepton2_sdk.h for more information

- channel_id - the laser from which the point was detected

- valid - true if the point is valid, false if not

The header timestamp of each message sent with this topic will contain the start_timestamp of the frame. To calculate the precise timestamp of each point you would need to use this value.

The prefix of the point topic is configurable through the yaml file. For each sensor connected there will be a separate topic with a different seriai number suffix. The naming convention is as follows:

```sh
<topic_prefix>_<serial_num>
```

Example: If topic prefix is cepton2/points and two sensors (10123 & 10126) are connected, then the nodelet will publish two topics:
```sh
cepton2/points_10123
cepton2/points_10126
```

## 6.0 - Sensor Information Topic
The nodelet publishes a sensor information message that is universal for all sensors connected. This topic name can be configurable via the publisher configuration parameters. The sensor information message has the following information:

```sh
SensorInformation.msg

Header header

uint64 handle
uint64 serial_number
string model_name
uint16 model
uint16 model_reserved

string firmware_version
            
int64 power_up_timestamp
int64 time_sync_offset
int64 time_sync_drift

uint8 return_count
uint8 segment_count

bool is_pps_connected
bool is_nmea_connected
bool is_ptp_connected

uint8[] data # `cepton_sdk::SensorInformation` bytes
```

## 7.0 - Known Issues
- Publishing sensor transformations is not supported at this time.
- RViz launch file not working with config.
