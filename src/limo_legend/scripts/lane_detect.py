#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Bool, Float64
import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detect")
        self.YELLOW_LANE_LOW_TH = np.array([0, 90, 100]) # hls 색 영역의 최솟값 설정
        self.YELLOW_LANE_HIGH_TH = np.array([60, 220, 255]) # hls 색 영역의 최댓값 설정
        self.cvbridge = CvBridge() # 실시간 스트리밍
        self.viz = rospy.get_param("~visualization", True)
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
        self.distance_pub1 = rospy.Publisher("/limo/lane_left", Int32, queue_size=5)
        self.distance_pub2 = rospy.Publisher("/limo/lane_right", Int32, queue_size=5)
        self.lane_connect_pub = rospy.Publisher("/limo/lane_connect", Bool, queue_size=5)
        self.lane_accel_pub = rospy.Publisher("/limo/lane/accel", Bool, queue_size=5)
        self.gtan_pub = rospy.Publisher("/limo/lane/gtan", Float64, queue_size=5)
    
    # 이미지 자르기
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        return _img[420:480, :] # 이미지 가로로 자르기
    
    # 노란색 부분 추출
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLOW_LANE_LOW_TH, self.YELLOW_LANE_HIGH_TH)
        return mask_yellow
    
    # 차선의 무게중심 계산
    def calcLaneDistance(self, _img=np.ndarray(shape=(480, 640))):
        try:
            M = cv2.moments(_img)
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])
        except:
            self.x = -1
            self.y = -1
        return self.x

    # 왼쪽 차선이 오른쪽 threshold 이미지에 표시될 경우 처리
    def lane_connect(self, thresholded_image, thresholded_image2):
        connected = np.sum(thresholded_image[:,319]) > 1 and np.sum(thresholded_image2[:,0]) > 1
        # 왼쪽 아래 카메라의 가장 오른쪽 한 줄을 적분하여 1보다 크면(차선이 존재하면) True
        # 오른쪽 아래 카메라의 가장 왼쪽 한 줄을 적분하여 1보다 크면(차선이 존재하면) True
        # 위 두 개가 동시에 참인 경우(왼쪽 차선이 두 카메라 모두에 인식된 경우) True를 connected에 전달
        self.lane_connect_pub.publish(connected)

    # 두 차선이 인식될 경우 True를 반환 (가속 구간)
    def lane_speed(self, thresholded_image, thresholded_image2):
        accel = np.sum(thresholded_image[0,:]) > 1 and np.sum(thresholded_image2[0,:]) > 1
        # 왼쪽 아래 카메라의 가장 윗부분 가로 한 줄을 적분하여 1보다 크면(차선이 존재하면) True
        # 오른쪽 아래 카메라의 가장 윗부분 가로 한 줄을 적분하여 1보다 크면(차선이 존재하면) True
        # 위 두 개가 동시에 참인 경우(양쪽 차선이 모두 존재하는 직진 구간인 경우) True를 accel에 전달
        self.lane_accel_pub.publish(accel)

    def global_tan(self, _img):
        _img = np.where(_img, True, False) # 흑백 처리한 이미지는 0 또는 255를 이용하므로 0 또는 1로 변환
        if np.sum(_img) == 0: # 이미지의 모든 픽셀 값 합산. 합이 0이면 이미지에 처리할 대상이 없는 것을 의미
            return 0
        x_range = np.arange(-320, 320) # x축 범위 설정. 이미지의 중심을 기준으로 왼쪽부터 오른쪽까지의 위치
        y_range = np.arange(180, 240) # y축 범위 설정
        matrix_x = _img * x_range # 이미지의 각 픽셀에 가로축 값들을 곱함. 픽셀 위치에 따른 가중치를 부여
        matrix_y = (_img.T * y_range).T # 이미지의 세로줄에 가중치를 부여하기 위해 전치행렬 이용
        x_not_zero = np.where(matrix_x, True, False)
        matrix_x = np.where(x_not_zero, matrix_x, 1)
        matrix_atan = np.where(x_not_zero, np.arctan(matrix_y / matrix_x), 0) # matrix_x가 참이면 arctan 연산결과를, 거짓이면 0을 반환
        # 각 픽셀에서 y축 값과 x축 값의 비율 (픽셀의 기울기)를 계산하고 arctan를 이용해 탄젠트 값(기울기)을 각도(라디안 단위)로 변환
        gtan = np.sum(matrix_atan) / np.sum(_img) # 차선의 평균 기울기 계산 (이미지 전체에서 감지된 차선의 기울기 / 이미지 내에서 차선으로 감지된 픽셀의 총 수)
        return gtan
    
    # 화면에 출력
    def visResult(self):
        pass
        # cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        # cv2.imshow("lane_original", self.frame)
        # cv2.imshow("lane_thresholded_left", self.thresholded_image)
        # cv2.imshow("lane_thresholded_right", self.thresholded_image2)
        # cv2.waitKey(1)

    def image_topic_callback(self, img):
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.thresholded_image_original = self.colorDetect(self.cropped_image)
        self.thresholded_image = self.thresholded_image_original[:, 0:320] # 흑백처리된 가로로 자른 이미지를 세로로 자르기 (왼쪽 차선 카메라)
        self.thresholded_image2 = self.thresholded_image_original[:, 320:640] # 흑백처리된 가로로 자른 이미지를 세로로 자르기 (오른쪽 차선 카메라)
        self.left_distance = self.calcLaneDistance(self.thresholded_image)
        self.right_distance = self.calcLaneDistance(self.thresholded_image2)
        self.lane_connect(self.thresholded_image, self.thresholded_image2)
        self.lane_speed(self.thresholded_image, self.thresholded_image2)
        self.lane_gtan = self.global_tan(self.thresholded_image_original)
        self.distance_pub1.publish(self.left_distance)
        self.distance_pub2.publish(self.right_distance)
        self.gtan_pub.publish(self.lane_gtan)
        
        if self.viz:
            self.visResult()
            
def run():
    new_class = LaneDetection()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
