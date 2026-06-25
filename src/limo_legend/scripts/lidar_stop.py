#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Duration, Float64
from sensor_msgs.msg import LaserScan
import math

class LidarObjectDetector:
    def __init__(self):
        rospy.init_node('lidar_stop', anonymous=False)
        self.warn_pub = rospy.Publisher("/limo/lidar_warn", String, queue_size=3)
        self.obsatcle_time_pub=rospy.Publisher("/limo/lidar_time", Duration, queue_size=5)
        self.timer_pub = rospy.Publisher("/limo/lidar/timer", Float64, queue_size=5)
        self.sub_ls = rospy.Subscriber(rospy.get_param("lidar_topic_name", "/scan"), LaserScan, self.lidar_callback)
        self.E_STOP_MIN_ANGLE_DEG = -10.0
        self.E_STOP_MAX_ANGLE_DEG = 10.0
        self.E_STOP_DISTANCE_METER = 0.3
        self.E_STOP_COUNT = 5
        self.Warning_Status = False
        self.timer = rospy.get_time() + 5
        self.Warning_first_time = rospy.Time.from_sec(0.0)
        self.Safe_first_time = rospy.Time.now()

    def lidar_callback(self, _data):
        cnt = 0
        angle_rad = [_data.angle_min + i * _data.angle_increment for i, _ in enumerate(_data.ranges)]
        angle_deg = [180 / math.pi * angle for angle in angle_rad]
        for i, angle in enumerate(angle_deg):
            if self.E_STOP_MIN_ANGLE_DEG <= angle <= self.E_STOP_MAX_ANGLE_DEG and 0.0 < _data.ranges[i] < self.E_STOP_DISTANCE_METER:
                cnt += 1
        if cnt >= self.E_STOP_COUNT:
            if not self.Warning_Status:
                self.Warning_Status = True
                self.Warning_first_time = rospy.Time.now()
            self.warn_pub.publish("Warning")
            rospy.logdebug("Object Detected!! Warning!!")
        else:
            if self.Warning_Status:
                self.Warning_Status = False
                self.Safe_first_time = rospy.Time.now()
            self.Warning_first_time = rospy.Time.now()
            self.warn_pub.publish("Safe")
            self.timer = rospy.get_time() + 5 # 현재 ROS 시간보다 5초 더 많은 시간을 timer에 저장 (5초 동안 장애물을 인식한 경우 특정 동작을 수행하기 위함)
            self.timer_pub.publish(float(self.timer))
            rospy.logdebug("Safe!!")

def run():
    new_class = LidarObjectDetector()
    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
