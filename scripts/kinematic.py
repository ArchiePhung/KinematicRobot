#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rospy
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from std_msgs.msg import Int16, Header      # Để gửi dữ liệu dạng mảng số nguyên
from sensor_msgs.msg import JointState
import time
import math
# from math import pi                           # Sử dụng số pi cho tính toán góc và vận tốc

class ControlMotorByKinematic():
    def __init__(self):
        # Khởi tạo node ROS tên 'swerve_module_controller' 
        rospy.init_node('swerve_module_controller', anonymous=False)
        self.rate = rospy.Rate(30)  # Tần số thực hiện vòng lặp là 30 Hz

        # Hướng mặc định của động cơ trái và phải
        # self.dir_motorLeft = 0
        # self.dir_motorRight = 1

        # Các tham số vật lý của hệ thống
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)  # m

        # Vị trí các module (m) so với tâm robot
        self.modules = [
            {
                "name": "front",
                "pos_x": 0.5, "pos_y": -0.5,
                "joint_steer": "joint_steering_front",
                "joint_drive": "joint_wheel_front",
                "steer_angle": 0.0,
                "wheel_angle": 0.0
            },
            {
                "name": "behind",
                "pos_x": -0.5, "pos_y": 0.5,
                "joint_steer": "joint_steering_behind",
                "joint_drive": "joint_wheel_behind",
                "steer_angle": 0.0,
                "wheel_angle": 0.0
            }
        ]

        self.last_time = rospy.Time.now()

        # Publisher
        self.pub_joint = rospy.Publisher("/joint_states", JointState, queue_size=10)

        # Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVel_callback)
        self.data_cmdVel = Twist() 
        self.is_cmdVel = 0 

        rospy.loginfo("Swerve drive controller started.")

    # Hàm callback khi có dữ liệu mới từ topic /cmd_vel
    def cmdVel_callback(self, data):
        self.data_cmdVel = data
        self.is_cmdVel = 1

    # Chuẩn hóa góc về [-pi, pi]
    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    # Hàm điều khiển chính
    def run(self):
        while not rospy.is_shutdown():
            if self.is_cmdVel == 1:
                # Tính toán giá trị vận tốc cho từng cụm bánh
                now = rospy.Time.now()
                dt = (now - self.last_time).to_sec()
                self.last_time = now

                js = JointState()
                js.header = Header()
                js.header.stamp = now
                js.name = []
                js.position = []
                js.velocity = []

                for module in self.modules:
                    if abs(self.data_cmdVel.linear.x) > 1e-4 or abs(self.data_cmdVel.linear.y) > 1e-4 or abs(self.data_cmdVel.angular.z) > 1e-4:
                        # Tính vận tốc tại module
                        Vx_wheel = self.data_cmdVel.linear.x - self.data_cmdVel.angular.z * module["pos_y"]
                        Vy_wheel = self.data_cmdVel.linear.y + self.data_cmdVel.angular.z * module["pos_x"]

                        # # Góc lái
                        # module["steer_angle"] = math.atan2(Vy_wheel, Vx_wheel)

                        # # Vận tốc quay bánh (rad/s)
                        # wheel_speed_linear = math.sqrt(Vx_wheel**2 + Vy_wheel**2)
                        # wheel_speed_rad = wheel_speed_linear / self.wheel_radius

                        # Tính góc lái mới
                        new_angle = math.atan2(Vy_wheel, Vx_wheel)

                        new_angle = self.normalize_angle(new_angle)
                        old_angle = self.normalize_angle(module["steer_angle"])

                        # Chênh lệch góc
                        angle_diff = self.normalize_angle(new_angle - old_angle)

                        wheel_speed_linear = math.sqrt(Vx_wheel**2 + Vy_wheel**2)
                        wheel_speed_rad = wheel_speed_linear / self.wheel_radius

                        # Nếu lệch > 90°, quay bánh ngược lại và điều chỉnh góc lái
                        if abs(angle_diff) > math.pi/2:
                            new_angle = self.normalize_angle(new_angle + math.pi)
                            wheel_speed_rad = -wheel_speed_rad  # đổi chiều quay bánh

                        # Giới hạn tốc độ quay của góc lái (rad/s)
                        max_steer_speed = math.radians(90)  # 90°/s

                        # Chênh lệch cần xoay
                        diff = self.normalize_angle(new_angle - module["steer_angle"])

                        # Giới hạn delta xoay theo max_steer_speed
                        max_step = max_steer_speed * dt
                        if abs(diff) > max_step:
                            diff = max_step if diff > 0 else -max_step

                        module["steer_angle"] = self.normalize_angle(module["steer_angle"] + diff)
                    else:
                        wheel_speed_rad = 0

                    # Tích lũy góc quay bánh
                    module["wheel_angle"] += wheel_speed_rad * dt

                    # Thêm vào JointState
                    js.name.extend([module["joint_steer"], module["joint_drive"]])
                    js.position.extend([module["steer_angle"], module["wheel_angle"]])
                    # js.velocity.extend([0.0, wheel_speed_rad])

                self.pub_joint.publish(js)

            self.rate.sleep()

# Hàm main để khởi động chương trình
def main():
    print('Starting main program')
    controlMT = ControlMotorByKinematic()
    controlMT.run()

# Chạy chương trình nếu script được gọi trực tiếp
if __name__ == '__main__':
    main()