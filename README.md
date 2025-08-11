
# Chương trình mô phỏng động học cho cụm bánh xoay
Description: Mô phỏng cụm bánh xoay (swerve drive) trong ROS1, nhận dữ liệu điều khiển từ tâm robot, gửi tốc độ cho 2 bánh, hiển thị vị trí robot trên RViz.



## Installation

```bash
git clone https://github.com/<tài_khoản>/swerve_description.git
cd swerve_description
chmod +x scripts/*.py
```


    
## Usage/Examples

```javascript
1. Chạy file urdf của mô hình mô phỏng
roslaunch swerve_description view_swerve.launch

2. Chạy chương trình tf từ odom->base_link
roslaunch swerve_description robot_pose.launch

3. Chạy chương trình động học mô hình (cmd_vel -> vận tốc của từng bánh)
roslaunch swerve_description kinematic.launch

4. Chương trình điều khiển từ bàn phím
roslaunch teleop_keyboard teleop.launch
```

