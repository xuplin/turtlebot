#!/usr/bin/env python3
#coding=utf-8

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_straight_path():
    rospy.init_node('straight_path_publisher', anonymous=True)
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # 创建一个新的路径消息
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"  # 设置坐标系为odom

        # 添加机器人当前位置到路径中
        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.Time.now()
        start_pose.header.frame_id = "odom"  # 设置坐标系为odom
        start_pose.pose.position.x = 0  # 替换为起始点的x坐标
        start_pose.pose.position.y = 0  # 替换为起始点的y坐标
        start_pose.pose.orientation.w = 1  # 默认朝向
        path_msg.poses.append(start_pose)

        # 添加直线终点到路径中
        end_pose = PoseStamped()
        end_pose.header.stamp = rospy.Time.now()
        end_pose.header.frame_id = "odom"  # 设置坐标系为odom
        end_pose.pose.position.x = 3  # 替换为终点的x坐标
        end_pose.pose.position.y = 0  # 替换为终点的y坐标
        end_pose.pose.orientation.w = 1  # 默认朝向
        path_msg.poses.append(end_pose)

        # 发布路径消息
        path_publisher.publish(path_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_straight_path()
    except rospy.ROSInterruptException:
        pass

