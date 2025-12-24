# aruco_tracking_ros

ROS2 Humble, Ubuntu 22.04
Cài đặt các pks cần thiết 
sudo apt-get update && sudo apt-get install -y \
     ros-humble-ros2-controllers \
     ros-humble-gazebo-ros \
     ros-humble-gazebo-ros-pkgs \
     ros-humble-ros2-control \
     ros-humble-gazebo-ros2-control \
     ros-humble-joint-state-publisher-gui \
     ros-humble-joy \
     ros-humble-joy-teleop \
     ros-humble-turtlesim \
     ros-humble-robot-localization \
     ros-humble-tf-transformations

clone repos về máy : git clone ....
clone repos aruco pose estimation về cùng folder src : https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation

Chạy chương trình: 
cd your_ws
colcon build
. install/setup.bash
Terminal 1: ros2 launch bumperbot_bringup simulated_robot.launch.py
Terminal 2: ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
Terminal 3: ros2 run bumperbot_controller control.py

Lưu ý cần tạo 1 box dán aruco_marker trong gazebo (Gemini), chỉnh sửa đúng thông tin của marker trong file config của aruco_pose_estimation và trong file control.py
