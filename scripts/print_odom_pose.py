#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import tf

def odometry_callback(msg):
    # 從 /odom_baselink 提取位置 (x, y)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 提取四元數並轉換為歐拉角 (yaw)
    orientation_q = msg.pose.pose.orientation
    quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # 打印 (x, y, yaw)
    rospy.loginfo(f"x: {x}, y: {y}, yaw: {yaw}")

def main():
    rospy.init_node("odom_printer", anonymous=True)

    # 訂閱 /odom_baselink
    rospy.Subscriber("/odom_baselink", Odometry, odometry_callback)

    rospy.loginfo("Node started: Printing /odom_baselink (x, y, yaw)")
    rospy.spin()

if __name__ == "__main__":
    main()
