#!/usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import argparse

def move_straight(distance):
    rospy.init_node('straight_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    linear_speed = 0.2  #線速度
    
    vel_msg.linear.x = linear_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # 计算需要的时间
    duration = distance / linear_speed
    
    start_time = rospy.Time.now().to_sec()
    
    rate = rospy.Rate(10)  # 10 Hz
    while (rospy.Time.now().to_sec() - start_time) < duration:
        velocity_publisher.publish(vel_msg)
        rate.sleep()
    
    # 停止机器人
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move the robot straight for a specified distance.')
    parser.add_argument('--d', type=float, default=1.0, help='The distance to move the robot in meters. Default is 1 meter.')
    args = parser.parse_args()

    try:
        move_straight(args.d)
    except rospy.ROSInterruptException:
        pass
