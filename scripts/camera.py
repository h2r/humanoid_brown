#Author is Skye Thompson, rory_thompson@brown.edu if you have questions
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import time
import numpy as np

class TakePhoto:
    def __init__(self, topic):

        self.bridge = CvBridge()
        self.image_received = False
        self.topic = topic

        # Connect image topic

        self.image_sub = rospy.Subscriber(self.topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    depth_camera = TakePhoto('/multisense/depth')
    color_camera = TakePhoto('/multisense/left/image_rect_color')

	 # The name is saved as the time the image was taken
    timestr = time.strftime("%Y%m%d-%H%M%S")

    color_img_title = "color_{}.png".format(timestr)
    depth_img_title = "depth_{}.png".format(timestr)

    if depth_camera.take_picture(depth_img_title):
        rospy.loginfo("Saved image " + depth_img_title)
    else:
        rospy.loginfo("No images received")

    if color_camera.take_picture(color_img_title):
        rospy.loginfo("Saved image " + color_img_title)
    else:
        rospy.loginfo("No images received")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)