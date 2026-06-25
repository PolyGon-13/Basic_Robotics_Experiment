#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, roslaunch, os
from pathlib import Path
from std_msgs.msg import Int32, String, Bool, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
import math

class LimoController:
    def __init__(self):
        rospy.init_node('control', anonymous=True) # rospy에 코드의 이름을 알려줌. 'control'의 이름의 노드가 복수로 실행 가능하게 함.
        self.limo_mode = "ackermann" # limo의 모드를 저장
        self.BASE_SPEED = 0.3 # limo의 기본 주행 속도
        self.LATERAL_GAIN = float(5 * 0.001) # 차선에 따라 회전할 정도를 지정하는 값
        self.REF_X = 80 # 왼쪽 차선의 Pixel 값
        self.REF_X2 = 240 # 오른쪽 차선의 Pixel 값
        self.left_receiveimage = False # 이미지 데이터를 받아오기 시작하면 True로 전환
        self.distance_left = 0 # 왼쪽 차선과의 거리
        self.distance_right = 0 # 오른쪽 차선과의 거리
        self.last_time = rospy.get_time() # imu 센서 데이터를 이용한 수학적 계산에서 dt를 구할 때 사용하기 위한 시간 정보
        self.heaviside = False # imu 센서에서 기울어짐을 감지하였을 때 True로 전환
        self.angular_y = 0 # y축을 기준으로 기울어진 정도를 저장
        self.accel_bool = False # lane_detect.py로부터 받아옴. 두 차선을 감지(가속 구간) 신호를 받으면 True로 전환
        self.lane_connected = False # lane_detect.py로부터 받아옴. 왼쪽 차선 정보가 2번째 카메라(오른쪽 차선)로 넘어가는 시점에 True로 전환
        self.override_twist = False # ar_marker.py로부터 받아옴. 마커 감지 유무를 저장
        self.park_bool = False # ar_marker.py로부터 받아옴. 주차 마커 감지 유무를 저장
        self.e_stop = "Safe" # lidar_stop.py로부터 받아오는 라이다 감지 결과 저장
        self.lidar_timer = 0.0 # lidar_stop.py로부터 받아오는 라이다가 장애물을 감지한 시점부터 흐른 시간 정보
        self.launch = dict()
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/imu",Imu, self.imu_callback)
        rospy.Subscriber("/limo/lane_left", Int32, self.lane_left_callback)
        rospy.Subscriber("/limo/lane_right", Int32, self.lane_right_callback)
        rospy.Subscriber("/limo/lane_connect", Bool, self.lane_connect_callback)
        rospy.Subscriber("/limo/lane/accel", Bool, self.lane_accel_callback)
        rospy.Subscriber("/limo/marker/cmd_vel", Twist, self.marker_cmd_vel_callback)
        rospy.Subscriber("/limo/marker/bool", Bool, self.marker_bool_callback)
        rospy.Subscriber("/limo/marker/park", Bool, self.marker_park_bool_callback)
        rospy.Subscriber("/limo/lidar_warn", String, self.lidar_warning_callback)
        rospy.Subscriber("/limo/lidar/timer", Float64, self.lidar_timer_callback)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback) # 0.03초마다 self.drive_callback 메서드 실행

    # 코드 내에서 launch 파일을 실행시키기 위해 필요한 메서드
    def roslaunch(self, filename):
        if filename in self.launch:
            return
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        path = Path(os.path.abspath(__file__)).parent.parent.joinpath("launch/%s"%filename)
        self.launch[filename] = roslaunch.parent.ROSLaunchParent(uuid, [str(path)])
        self.launch[filename].start()
    
    # 리모 모드 설정
    def limo_status_callback(self, _data):
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                # rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                # rospy.loginfo("Mode Changed --> Differential Drive")

    # lane_detect.py로부터 받아온 왼쪽 차선 정보 처리
    def lane_left_callback(self, _data):
        self.left_receiveimage = True # 이미지 데이터를 받았음을 알림 -> marker.launch 파일 실행
        if _data.data == -1: # 왼쪽 차선 인식X
            self.distance_left = 0
        else: # 왼쪽 차선 인식
            self.distance_left = self.REF_X - _data.data # 거리 계산

    # lane_detect.py로부터 받아온 오른쪽 차선 정보 처리
    def lane_right_callback(self, _data):
        if _data.data == -1: # 오른쪽 차선 인식X
            self.distance_right = 0
        else: # 오른쪽 차선 인식
            self.distance_right = self.REF_X2 - _data.data # 거리 계산

    # lane_detect.py로부터 받아온 왼쪽 차선이 2번째 카메라(오른쪽 차선)에 침범했는 여부를 변수에 저장
    def lane_connect_callback(self, _data):
        self.lane_connected = _data.data

    # lane_detect.py로부터 받아온 두 차선을 인식했는지 여부를 변수에 저장
    def lane_accel_callback(self, _data):
        self.accel_bool = _data.data
            
    # lidar_stop.py로부터 받아온 라이다의 장애물 인식 여부 정보 변수에 저장
    def lidar_warning_callback(self, _data):
        self.e_stop = _data.data

    # lidar_stop.py로부터 받아온 라이다가 장애물을 인식한 시점부터 흐른 시간 변수에 저장
    def lidar_timer_callback(self, _data):
        self.lidar_timer = _data.data

    # ar_marker.py로부터 받아온 마커에 따른 주행 데이터를 변수에 저장
    def marker_cmd_vel_callback(self, _data):
        self.new_drive_data = _data

    # ar_marker.py로부터 받아온 마커 인식 여부를 변수에 저장
    def marker_bool_callback(self, _data):
        self.override_twist = _data.data
        
    # ar_marker.py로부터 받아온 주차 마커 인식 여부를 변수에 저장
    def marker_park_bool_callback(self, _data):
        self.park_bool = _data.data    
    
    # imu 센서로부터 받아온 값들을 이용해 로봇의 기운 정도 계산 (합성곱 이용)
    def imu_callback(self, msg):
        dt = rospy.get_time() - self.last_time 
        self.last_time = rospy.get_time()
        self.angular_y += msg.angular_velocity.y * dt
        self.angular_y *= 1 - dt # 지속적인 오차 누적을 피하기 위해 해당 값을 0으로 수렴시키 위한 값을 곱해줌
     
    # 각각의 코드와 메서드들에서 처리한 주행 데이터를 처리 및 퍼블리시
    def drive_callback(self, _event):
        drive_data = Twist() # drive_data를 Twist 메시지 형태로 선언
        # 기본 동작
        drive_data.linear.x = self.BASE_SPEED
        drive_data.angular.z = (self.distance_left + self.distance_right) * self.LATERAL_GAIN # 왼쪽 차선과 오른쪽 차선과의 거리를 이용한 각속도 계산

        try:
            # 라인 겹침 처리
            if self.lane_connected == True: # 왼쪽 차선이 2번째 카메라(오른쪽 차선)에 침범한 경우
                drive_data.angular.z = self.distance_left * self.LATERAL_GAIN # 오른쪽 카메라의 계산값은 무시하고 왼쪽 차선 정보를 이용

            # 마커 감지 유무에 따른 마커 동작
            if self.override_twist == True: # 마커를 인식한 경우
                # if self.e_stop != "Warning": # 라이다가 장애물을 감지하지 않았을 경우 (표지판을 장애물로 인식하는 경우가 있어서 주석처리함)
                drive_data = self.new_drive_data # 마커 동작을 수행
            elif self.lane_connected == False and self.accel_bool == True: # 왼쪽 차선이 2번째 카메라(오른쪽 차선)에 침범하지 않았으며 두 차선을 인식한 경우
                if abs(drive_data.angular.z) < 0.3 and self.park_bool == False:
                    # 계산된 각속도가 0.3보다 작을 때(가속을 하면 안되는 구간에서도 두 차선을 인식하는 경우가 발생하기 때문에 사용)
                    # 또한 주차 마커를 인식하지 않은 경우(교차로 구간에서 가속을 하는 구간이 발생하는데 주차 모션 제어가 방해가 됨)
                    drive_data.linear.x *= 1.3 # 기존 속도의 1.3배로 달림

            # IMU 센서 동작
            if abs(self.angular_y) > 0.05:
                self.heaviside = True
            elif abs(self.angular_y) < 0.05:
                self.heaviside = False
            
            # 라이다 동작
            if self.lidar_timer < rospy.get_time(): # 라이다가 장애물을 감지한 시점에서 흐른 시간(ros현재 시간 + 5)보다 현재 ros시간이 큰 경우 = 5초가 지난 경우
                # print("lidar_stop")
                drive_data.linear.x = 0.0
                drive_data.angular.z = -3.0 # 제자리 회전할 각속도
            elif self.e_stop == "Warning": # 라이다가 장애물을 감지한 경우
                drive_data.linear.x = 0.0
                drive_data.angular.z = 0.0
            elif self.heaviside == True: # imu 센서가 로봇의 기울어짐을 감지한 경우
                drive_data.linear.x /= 2
                drive_data.angular.z = 0.0
            
            # 리모 모드에 따른 동작
            if self.limo_mode == "diff": # differential mdoe인 경우 (주황색)
                self.drive_pub.publish(drive_data)
            elif self.limo_mode == "ackermann": # ackermann mode인 경우 (초록색)
                pass # 원래 추가적인 계산처리가 있지만 ackermann mode를 사용하지 않기 때문에 배제

        except Exception as e:
            rospy.logwarn(e)
            
def run():
    new_class = LimoController()
    while not rospy.is_shutdown():
        if new_class.left_receiveimage: # 카메라에서 이미지를 받아온 후 aruco_maker 관련 노드를 실행 (카메라 충돌 방지)
            new_class.roslaunch("marker.launch")

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
