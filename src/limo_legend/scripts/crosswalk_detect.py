#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class CrossWalkDetector:
    def __init__(self):
        rospy.init_node("crosswalk_detect")
        self.cvbridge = CvBridge()
        self.WHITE_LANE_LOW = np.array([0, 100, 0])
        self.WHITE_LANE_HIGH = np.array([170, 255, 255])
        self.RHO = float(100 * 0.01)
        self.THETA = 1
        self.THRESHOLD = 20
        self.CROSS_WALK_DETECT_TH = 20
        self.crosswalk_bool = False
        self.viz = rospy.get_param("~visualization", True)
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.Image_CB)
        self.distance_pub = rospy.Publisher("/limo/crosswalk/distance", Int32, queue_size=5)

    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        self.crop_size_x = 360
        self.crop_size_y = 60
        return _img[420:480, 170:530]

    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_white = cv2.inRange(hls, self.WHITE_LANE_LOW, self.WHITE_LANE_HIGH)
        return mask_white

    def edgeDetect(self, _img=np.ndarray(shape=(480, 640))):
        return cv2.Canny(_img, 0, 360)

    def houghLineDetect(self, _img=np.ndarray(shape=(480, 640))):
        new_img = _img.copy()
        self.lines = cv2.HoughLinesP(new_img, self.RHO, self.THETA * np.pi / 180, self.THRESHOLD, minLineLength=10, maxLineGap=5)

        if self.lines is None:
            self.line_num = 0
            self.crosswalk_bool = False
        else:
            self.line_num = len(self.lines)

            if self.line_num >= self.CROSS_WALK_DETECT_TH:
                self.crosswalk_bool = True
            else:
                self.crosswalk_bool = False
            for i in range(self.lines.shape[0]):
                pt1 = (self.lines[i][0][0], self.lines[i][0][1])
                pt2 = (self.lines[i][0][2], self.lines[i][0][3])
                cv2.line(self.cropped_image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)

    def calcCrossWalkDistance(self, _img):
        if self.line_num >= self.CROSS_WALK_DETECT_TH:
            try:
                M = cv2.moments(_img)
                self.x = int(M['m10']/M['m00'])
                self.y = int(M['m01']/M['m00'])
            except:
                self.x = -1
                self.y = -1
            return self.y
        else:
            self.x = 0
            self.y = 0
            return -1

    def visResult(self):
        pass
        # if not self.x <= 0 and not self.y <= 0:
            # cv2.line(self.cropped_image, (0, self.y), (self.crop_size_x, self.y), (0, 255, 255), 20)
        # cv2.imshow("crosswalk_cropped", self.cropped_image)
        # cv2.imshow("crosswalk_thresholded", self.thresholded_image)
        # cv2.imshow("crosswalk_edge", self.edge_image)
        # cv2.waitKey(1)

    def Image_CB(self, img):
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.edge_image = self.edgeDetect(self.thresholded_image)
        self.houghLineDetect(self.edge_image)
        self.crosswalk_distance = self.calcCrossWalkDistance(self.thresholded_image)
        self.distance_pub.publish(self.crosswalk_distance)

        if self.viz:
            self.visResult()

def run():
    new_class = CrossWalkDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
