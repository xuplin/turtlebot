#!/usr/bin/env python3
#coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from math import pi
import argparse
import os
from datetime import datetime
import random




#-----定點計時------#
#=============================#
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
#=============================#
def stop_and_record_time(velocity_publisher, log_file):
    # 停止機器人
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    
    # 紀錄當前時間
    current_time = rospy.Time.now().to_sec()
    real_current_time = datetime.fromtimestamp(current_time)
    current_time_str = "Recorded time at: %s\n" % real_current_time.strftime("%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(current_time_str.strip())
    
    # 寫入文件
    log_file.write(current_time_str)
    
    
    # 等待時間(要停多久)
    rospy.sleep(0.01)



#-----前進------#
#=============================#
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>#
#=============================#
def move_in_arc(radius,angle,velocity_publisher,log_file,segangle,linear_speed):
    vel_msg = Twist()

    #場地設置
    #radius = 2.0  # 圓的半徑
    #linear_speed = 0.2  #線速度
    angular_speed = linear_speed / radius  #角速度
    angle_to_travel = angle * pi/180  # 要走的總弧度 
    
    # 速度設置
    vel_msg.linear.x = linear_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = angular_speed

    #計算總時間
    duration = angle_to_travel / abs(angular_speed)
    
    # 開始時間
    start_time = rospy.Time.now().to_sec()
    real_start_time = datetime.fromtimestamp(start_time)
    start_time_str = "Start moving at: %s\n" % real_start_time.strftime("%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(start_time_str.strip())
    
    log_file.write(start_time_str)
    rate = rospy.Rate(10)  # 10 Hz
    last_record_time = start_time
    angle_moved = 0
    n_degrees = segangle * pi / 180  # 将n度转换为弧度
    

    #總距離
    while (rospy.Time.now().to_sec() - start_time) < duration and not rospy.is_shutdown(): 
        velocity_publisher.publish(vel_msg)
        current_time = rospy.Time.now().to_sec()
        angle_moved += angular_speed * (current_time - last_record_time)
        last_record_time = current_time
        #比較當前角度與切割角度
        if angle_moved >= n_degrees:
            stop_and_record_time(velocity_publisher, log_file)
            angle_moved = 0
        
        rate.sleep()
    
    # stop move
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    #停止時間
    end_time = rospy.Time.now().to_sec()
    real_end_time = datetime.fromtimestamp(end_time)
    end_time_str = "Stopped moving at: %s\n" % real_end_time.strftime("%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(end_time_str.strip())
    log_file.write(end_time_str)
    #log_file.flush() #new
    #return start_time_str, end_time_str
    

#-----後退------#
#=============================#
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<#
#=============================#
def move_back_in_arc(radius, angle, velocity_publisher,log_file,segangle,linear_speed):
    vel_msg = Twist()
    #radius = 2.0  # 圓的半徑
    #linear_speed = 0.2  # 線速度
    angular_speed = linear_speed / radius  # 角速度
    
    angle_to_travel = angle * pi / 180  # 要走的弧度
    
    # 設置速度，反方向行駛
    vel_msg.linear.x = -linear_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -angular_speed
    
    duration = angle_to_travel / abs(angular_speed)
    
    start_time = rospy.Time.now().to_sec()
    real_start_time = datetime.fromtimestamp(start_time)
    start_time_str = "Start moving back at: %s\n" % real_start_time.strftime("%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(start_time_str.strip())

    log_file.write(start_time_str)
    rate = rospy.Rate(10)  # 10 Hz
    last_record_time = start_time
    angle_moved = 0
    n_degrees = segangle * pi / 180  
    
    while (rospy.Time.now().to_sec() - start_time) < duration and not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        current_time = rospy.Time.now().to_sec()
        angle_moved += abs(angular_speed) * (current_time - last_record_time)
        last_record_time = current_time
        
        if angle_moved >= n_degrees:
            stop_and_record_time(velocity_publisher, log_file)
            angle_moved = 0
        rate.sleep()

    # stop move
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    #停止時間
    end_time = rospy.Time.now().to_sec()
    real_end_time = datetime.fromtimestamp(end_time)
    end_time_str = "Stopped moving back at: %s\n" % real_end_time.strftime("%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(end_time_str.strip())
    log_file.write(end_time_str)
    #log_file.flush()
    #return start_time_str, end_time_str

#-----時間------#
#=============================#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#=============================#
def move_and_return(radius, angle, repetitions,segangle,linear_speed):
    rospy.init_node('arc_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    nowangle=0

    #文件路徑
    log_path = os.path.expanduser('~/6g_log')
    os.makedirs(log_path, exist_ok=True)
    log_file_path = os.path.join(log_path, 'logfile.txt')

    with open(log_file_path, 'a') as log_file:
        for i in range(repetitions):
            rospy.loginfo(f"Iteration  {i+1}  of {repetitions}")
            log_file.write(f"Iteration {i+1} of {repetitions}\n")

            #隨機前進角度
            possible_angles=[angle for angle in [5,10,15,20,25,30,35,40,45,50,55,60] if angle+ nowangle <= 60]
            random_forward_angle = random.choice(possible_angles)
            nowangle+=random_forward_angle
            rospy.loginfo(f"Generated random angle: {random_forward_angle} degrees")
            log_file.write(f"Generated random angle: {random_forward_angle} degrees\n")
            
            #隨機後退角度
            move_in_arc(radius, random_forward_angle,velocity_publisher, log_file,segangle,linear_speed)
            rospy.sleep(0.1)#休息

            possible_angles = [angle for angle in [5,10,15,20,25,30,35,40,45,50,55,60] if angle <= nowangle]
            random_backward_angle = random.choice(possible_angles)
            nowangle -= random_backward_angle
            rospy.loginfo(f"Generated random angle: {random_backward_angle} degrees")
            log_file.write(f"Generated random angle: {random_backward_angle} degrees\n")

            move_back_in_arc(radius, random_backward_angle, velocity_publisher, log_file,segangle,linear_speed)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move the robot in an arc.')
    parser.add_argument('--r', type=float, default=2,required=False, help='Radius of the arc (in meters)') # 圓的半徑
    parser.add_argument('--g', type=float, default=60,required=False, help='Angle to travel (in degrees)') #總角度
    parser.add_argument('--n', type=int, default=1, required=False, help='Number of repetitions') #來回次數
    parser.add_argument('--c', type=int, default=5, required=False, help='Number of sugment angle') #間隔角度
    parser.add_argument('--s', type=float, default=0.2, required=False, help='Number of speed') #速度
    args = parser.parse_args()
    try:
        
        move_and_return(args.r, args.g, args.n, args.c, args.s)
    except rospy.ROSInterruptException:
        pass

