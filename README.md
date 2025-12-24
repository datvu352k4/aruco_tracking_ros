# 🤖 ROS 2 ArUco Tracking (Humble)

Dự án mô phỏng robot bám theo ArUco Marker sử dụng ROS 2 Humble và Gazebo.

## 1. Cài đặt (Installation)

**Yêu cầu:** Ubuntu 22.04, ROS 2 Humble.

### Cài đặt Dependencies
```
sudo apt update && sudo apt install -y \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joy-teleop \
    ros-humble-turtlesim \
    ros-humble-robot-localization \
    ros-humble-tf-transformations
```
Build Workspace
Bash

# 1. Tạo workspace và clone repos
```
mkdir -p ~/aruco_tracking_ws/src
cd ~/aruco_tracking_ws/src
git clone <URL_REPO_CUA_BAN>  # Thay bằng link repo của bạn
git clone [https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation)
```

# 2. Build
```
cd ~/aruco_tracking_ws
colcon build --symlink-install
source install/setup.bash
```
2. Cấu hình (Configuration)
⚠️ Quan trọng:

Gazebo: Phải tạo một Box có dán ảnh ArUco (xem phần Phụ lục bên dưới).

Config: Đảm bảo thông số trong file src/ros2_aruco_pose_estimation/config/aruco_parameters.yaml khớp với model trong Gazebo:

marker_size: Kích thước thật (mét).

marker_id: ID của ảnh marker.

camera_frame: Tên frame camera của robot.

3. Hướng dẫn chạy (Usage)
Mở 3 Terminal, chạy lần lượt (nhớ source install/setup.bash ở mỗi terminal):
```
Terminal 1: Khởi động Mô phỏng

Bash

ros2 launch bumperbot_bringup simulated_robot.launch.py
Terminal 2: Chạy thuật toán nhận diện

Bash

ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
Terminal 3: Chạy Controller

Bash

ros2 run bumperbot_controller control.py
```
📦 Phụ lục: Tạo ArUco Box trong Gazebo
Tạo thư mục: ~/.gazebo/models/my_aruco_box/ Cấu trúc file cần có:
```
my_aruco_box/
├── model.config
├── model.sdf
└── materials
    ├── scripts/marker.material
    └── textures/marker.png  <-- (File ảnh ArUco của bạn)
```
1. model.config
```
<?xml version="1.0"?>
<model>
  <name>My ArUco Box</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>
  <description>Simple ArUco Box</description>
</model>
```
2. materials/scripts/marker.material
```
material Aruco/Marker {
  technique {
    pass {
      texture_unit { texture marker.png }
    }
  }
}
```
3. model.sdf
```
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="my_aruco_box">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
        <material>
          <script>
            <uri>model://my_aruco_box/materials/scripts</uri>
            <uri>model://my_aruco_box/materials/textures</uri>
            <name>Aruco/Marker</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```
Sau đó vào Gazebo -> Insert -> Chọn "My ArUco Box".
