# Livox ROS Driver 2 (ROS2 Humble Fork)

This is a forked version of `livox_ros_driver2`, optimized specifically for **ROS2 Humble on Ubuntu 22.04**. The original driver was intended to support multiple ROS versions, but this fork removes all legacy code for ROS1 and older ROS2 distributions to provide a cleaner, more streamlined experience for Humble users.

All ROS1-related files, conditional compilation blocks, and the `build.sh` script have been removed. This package is now a standard ROS2 package and should be built using `colcon`.

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS Requirements

  * Ubuntu 22.04 for ROS2 Humble

### 1.2 Install ROS2 Humble

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

A Desktop-Full installation is recommended.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
# Example workspace structure
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/ggory15/livox_ros_driver2
```

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

Navigate to your workspace root and build the package using `colcon`:
```shell
cd ~/livox_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 2.4 Run Livox ROS Driver 2:

Source your workspace's setup file and use `ros2 launch`:
```shell
source install/setup.bash
ros2 launch livox_ros_driver2 [launch file]
```

in which,  

* **[launch file]** : is the ROS2 launch file you want to use; the `launch` folder contains several launch samples for your reference.

An rviz launch example for HAP LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch file and internal parameter configuration

### 3.1 Launch file configuration

Launch files are located in the `livox_ros_driver2/launch` directory. Different launch files are provided for different scenarios:

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| rviz_HAP_launch.py   | Connect to HAP LiDAR, publish PointCloud2 data, and autoload rviz. |
| msg_HAP_launch.py     | Connect to HAP LiDAR and publish Livox custom message data. |
| rviz_MID360_launch.py | Connect to MID360 LiDAR, publish PointCloud2 data, and autoload rviz. |
| msg_MID360_launch.py  | Connect to MID360 LiDAR and publish Livox custom message data. |
| rviz_mixed.py    | Connect to both HAP and MID360 LiDARs, publish PointCloud2 data, and autoload rviz. |

### 3.2 Main internal parameters

All internal parameters are configured within the launch files. The most common parameters are:

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | The frequency of point cloud publishing (Hz). Recommended values: 5.0, 10.0, 20.0, 50.0. Max: 100.0. | 10.0    |
| multi_topic  | 0: All LiDARs publish on a single topic.<br>1: Each LiDAR publishes on its own unique topic. | 0       |
| xfer_format  | The point cloud message format.<br>0: Livox PointCloud2 (PointXYZRTLT)<br>1: Livox CustomMsg | 0       |

  **Note :**

  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Point Cloud Data Formats:***

1. **Livox PointCloud2 (PointXYZRTLT)**:

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # Reflectivity value, 0.0~255.0
uint8   tag             # Livox tag
uint8   line            # Laser number in lidar
float64 timestamp       # Timestamp of the point
```

2. **Livox CustomMsg**:

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of the first point
uint32          point_num  # Total number of points
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved
CustomPoint[]   points     # Point cloud data
```

&ensp;&ensp;&ensp;&ensp;**CustomPoint** format:

```c
uint32  offset_time     # Offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # Reflectivity, 0~255
uint8   tag             # Livox tag
uint8   line            # Laser number in lidar
```

## 4. LiDAR Config

LiDAR configurations (IP, port, data type, etc.) are set via a JSON-style config file located in the `config` directory. The `user_config_path` parameter in the launch files specifies which config file to use.

For details on the JSON configuration format, please refer to the examples in the `config` folder and the official [HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description).

## 5. Supported LiDAR list

* HAP
* Mid360
* (more types are coming soon...)

## 6. FAQ

### 6.1 No point cloud is displayed in RViz?

Please check the "Global Options -> Fixed Frame" field in RViz. Set the value to `livox_frame` and ensure the "PointCloud2" display is enabled with the correct topic.

### 6.2 "cannot open shared object file liblivox_sdk_shared.so"?

The linker cannot find the installed Livox SDK library. Please add its location to your environment.

* For the current terminal:
  ```shell
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
  ```

* To make it permanent for your user:
  ```shell
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
  source ~/.bashrc
  ```
