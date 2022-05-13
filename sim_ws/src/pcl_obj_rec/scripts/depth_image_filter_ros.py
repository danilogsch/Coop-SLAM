#!/usr/bin/env python
"""
Purpose of the file: subscribe to a topic called /image_raw of type sensor_msgs/Image
Apply filter to the resulting image
"""
from __future__ import print_function
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SubThenFilter:
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/depth/image_rect_raw_conv", Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher("/depth_registered/image_rect_filtered", Image, queue_size=1)
        self.bridge = CvBridge()
        self.median_blur_size = 5
        self.use_median_blur = True

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            print(e)

        cv_image = np.nan_to_num(cv_image)
        if self.use_median_blur:
            #cv_image = cv2.medianBlur(cv_image, self.median_blur_size)
            #print(cv2)
            pass
            #cv_image = cv2.bilateralFilter(cv_image, 5, 8, 0.2)

        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            data.data = msg.data
            data.header.stamp = rospy.Time.now()
            self.pub.publish(data)
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("depth_filter", anonymous=True)
    sf = SubThenFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
cv2.destroyAllWindows()