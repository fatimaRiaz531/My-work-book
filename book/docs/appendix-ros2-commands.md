---
sidebar_position: 6
---

# Appendix B: ROS 2 Command Reference

Quick reference for essential ROS 2 commands used throughout the course.

## Node Management

```bash
# List running nodes
ros2 node list

# Get info about a node
ros2 node info /node_name

# Start a node
ros2 run package_name executable_name

# Launch a launch file
ros2 launch package_name launch_file.py
```

## Topic Management

```bash
# List all topics
ros2 topic list

# See topic message structure
ros2 topic info /topic_name

# Echo (print) topic messages
ros2 topic echo /topic_name

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Check message frequency and bandwidth
ros2 topic hz /topic_name
ros2 topic bw /topic_name
```

## Service Management

```bash
# List all services
ros2 service list

# See service definition
ros2 service type /service_name

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## Parameter Management

```bash
# List parameters on a node
ros2 param list

# Get a parameter value
ros2 param get /node_name parameter_name

# Set a parameter value
ros2 param set /node_name parameter_name value

# Dump all parameters to file
ros2 param dump /node_name > params.yaml

# Load parameters from file
ros2 param load /node_name params.yaml
```

## Package Management

```bash
# Find a package
ros2 pkg find package_name

# List all packages
ros2 pkg list

# Create a new package
ros2 pkg create --build-type ament_cmake package_name

# Build packages
colcon build

# Build a specific package
colcon build --packages-select package_name

# Source the setup file
source install/setup.bash
```

## Debugging and Introspection

```bash
# Visualize nodes and topics (ROS graph)
ros2 run rqt_graph rqt_graph

# Record and playback bags
ros2 bag record -a  # Record all topics
ros2 bag play rosbag2_2023_01_01-00_00_00

# Check message rates
ros2 run rqt_plot rqt_plot

# Inspect and call services with GUI
ros2 run rqt_service_caller rqt_service_caller
```

## Common Message Types

```bash
# Geometry messages
ros2 msg show geometry_msgs/msg/Point
ros2 msg show geometry_msgs/msg/Pose

# Sensor messages
ros2 msg show sensor_msgs/msg/Image
ros2 msg show sensor_msgs/msg/PointCloud2
ros2 msg show sensor_msgs/msg/Imu

# Standard messages
ros2 msg show std_msgs/msg/Float32
ros2 msg show std_msgs/msg/String
```

## Environment Variables

```bash
# Set the ROS 2 domain ID (for multiple robots)
export ROS_DOMAIN_ID=42

# Set logging level
export RCL_LOG_LEVEL=DEBUG

# ROS 2 network configuration
export ROS_LOCALHOST_ONLY=1  # For single machine only
```

## Helpful Utilities

```bash
# Check ROS 2 installation
ros2 --version

# Check configuration
ros2 config show

# List all installed packages
apt list --installed | grep ros

# Find ROS 2 installation directory
echo $ROS_DISTRO
which ros2
```

---

**For more details:** https://docs.ros.org/en/humble/

**Last Updated:** December 7, 2025
