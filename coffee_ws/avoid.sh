ros2 launch livox_ros_driver2 rviz_MID360_launch.py
#ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args   -r cloud_in:=/livox/lidar   -r scan:=/scan   -p target_frame:=base_laser   -p min_height:=-0.05   -p max_height:=1.0   -p range_min:=0.15   -p range_max:=5.0

