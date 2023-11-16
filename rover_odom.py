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

    def left_motor_callback(self, data):
        # Sol motor geri bildirimi işleme
        self.wheel_velocity_left = data.data[0]  

    def right_motor_callback(self, data):
        # Sağ motor geri bildirimi işleme
        self.wheel_velocity_right = data.data[0]  

    def calculate_odometry(self):
        # Tekerlek hareket verilerini kullanarak pozisyon ve yönelimi hesaplama
        # Varsayılan olarak sıfır 
        x = 0.0
        y = 0.0
        theta = 0.0

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

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        pose_stamped = PoseStamped()
        pose_stamped.pose = odom_msg.pose.pose
        pose_stamped.header = path_msg.header

        path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

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

    def spin(self):
        rate = rospy.Rate(10)  # 1 Hz

        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        rover_odom_publisher = RoverOdometryPublisher()
        rover_odom_publisher.spin()
    except rospy.ROSInterruptException:
        pass
