#!/usr/bin/env python3
#coding=utf-8

import rospy
from datetime import datetime

def test_timer_precision(sleep_time):
    rospy.init_node('timer_test')
    start_time = datetime.now()
    rospy.sleep(sleep_time)
    end_time = datetime.now()
    elapsed_time = end_time - start_time
    rospy.loginfo(f"Requested sleep time: {sleep_time} seconds")
    rospy.loginfo(f"Actual sleep time: {elapsed_time.total_seconds()} seconds")

if __name__ == '__main__':
    try:
        test_timer_precision(0.000001)  # 测试1微秒的睡眠时间
        test_timer_precision(0.001)     # 测试1毫秒的睡眠时间
    except rospy.ROSInterruptException:
        pass
