#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
import math
import numpy as np


class RoverOdometryPublisher:
    RADIUS = 0.135
    WHEELBASE = 8.9

    def __init__(self):
        rospy.init_node('rover_odometry_publisher', anonymous=True)
        
        # Tekerlek hareket verilerini takip etmek için gerekli olan değişkenler
        self.wheel_velocity_left = 0.0
        self.wheel_velocity_right = 0.0

        # Odometry mesajını yayınlamak için gerekli olan publisher
        self.odom_publ = rospy.Publisher('/wheel_odom', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)


        # Motor geri bildirim topic'lerini dinlemek için subscriber'lar
        rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, self.left_motor_callback)
        rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, self.right_motor_callback)

        self.pose_msg = PoseStamped()
        self.path_msg = Path()


    def left_motor_callback(self, data):
        # Sol motor geri bildirimi işleme
        self.wheel_velocity_left = data.data[0]
        # RPM olarak aldığımız tekerlek verilerini m/s cinsinden hıza çevirme
        self.wheel_velocity_left = 2 * math.pi * self.RADIUS * (self.wheel_velocity_left / 60)

    def right_motor_callback(self, data):
        # Sağ motor geri bildirimi işleme
        self.wheel_velocity_right = data.data[0]
        # RPM olarak aldığımız tekerlek verilerini m/s cinsinden hıza çevirme
        self.wheel_velocity_right = 2 * math.pi * self.RADIUS * (self.wheel_velocity_right / 60)  

    def calculate_odometry(self):
        dt = 10.0 / 10.0  # Zaman adımı, Hz cinsinden
        v_left = self.wheel_velocity_left
        v_right = self.wheel_velocity_right

        # Hareket modelini kullanarak pozisyon ve yönelimi güncelleme
        v = self.RADIUS / 2.0 * (v_left + v_right)
        omega = self.RADIUS / self.WHEELBASE * (v_right - v_left)
        theta = self.pose_msg.pose.orientation.z + omega * dt

        x = self.pose_msg.pose.position.x + v * dt * math.cos(theta)
        y = self.pose_msg.pose.position.y + v * dt * math.sin(theta)
        

        # Quaternion oluşturma
        quaternion = Quaternion(*quaternion_from_euler(0, 0, theta))

        return x, y, theta, quaternion

    def publish_odometry(self):
        # Odometry mesajını oluşturma
        x, y, theta, quaternion = self.calculate_odometry()

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pozisyon bilgilerini doldurma
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion

        # Hız formulleri
        odom_msg.twist.twist.linear.x = self.RADIUS / 2.00 * (self.wheel_velocity_left + self.wheel_velocity_right) 
        odom_msg.twist.twist.linear.y = self.RADIUS / 2.00 * (self.wheel_velocity_left + self.wheel_velocity_right) 
        odom_msg.twist.twist.angular.z = self.RADIUS / self.WHEELBASE * (self.wheel_velocity_right - self.wheel_velocity_left) 

        # Odometry mesajını yayınlama
        self.odom_publ.publish(odom_msg)

        # Pose mesajını güncelle
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_msg.header.frame_id = "odom"
        self.pose_msg.pose = odom_msg.pose.pose

        # Path mesajını güncelle ve yayınla
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "odom"
        self.path_msg.poses.append(self.pose_msg)
        self.path_pub.publish(self.path_msg)

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        rover_odom_publisher = RoverOdometryPublisher()
        rover_odom_publisher.spin()
    except rospy.ROSInterruptException:
        pass

