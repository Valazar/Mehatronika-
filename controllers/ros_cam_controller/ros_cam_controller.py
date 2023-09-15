#!/usr/bin/env python3
"""my_ROS_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from controller import Robot
import os
import cv_bridge
import cv2
import numpy as np


# create the Robot instance.
robot = Robot()
timeStep = int(256)
robot.step(timeStep)

opencv_bridge = cv_bridge.CvBridge()

print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.init_node('webots_cam')
rate = rospy.Rate(30)

pub = rospy.Publisher('webots_camera', Image, queue_size=10)
print('Running the control loop')

camera = robot.getDevice('camera')
camera.enable(timeStep)

img_msg = Image()

while robot.step(timeStep) != -1 and not rospy.is_shutdown():
    webots_img = camera.getImage()
    image = np.frombuffer(webots_img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image = image[:,:,:3]
    ros_img = opencv_bridge.cv2_to_imgmsg(image, encoding="passthrough")
    
    ros_img.header.seq = 0
    ros_img.header.stamp = rospy.get_rostime()
    ros_img.header.frame_id = 'camera_link'

    pub.publish(ros_img)
    rate.sleep()