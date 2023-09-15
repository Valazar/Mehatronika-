#!/usr/bin/env python3
from cv2 import waitKey
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge as cvb 
from cv_bridge import CvBridgeError
import numpy as np
from enum import Enum
import time 
import std_msgs.msg as msg 
import math
import matplotlib.pyplot as plt

class TableFieldType(Enum):
    FIELD_EMPTY = 0
    FIELD_OBSTACLE = 1
    FIELD_ENDING1 = 2
    FIELD_ENDING2 = 3
    FIELD_ROBOT = 4
    FIELD_OBJECT = 5

class CameraFilter:

    def __init__(self):
        self.cvb = cvb.CvBridge()

    def cut_image(self, data):
        try:
            cv_image = self.cvb.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv_image[50:670, 80:1000]
        cv2.imshow('original image',cv_image)
        cv2.waitKey(delay=1)
        rospy.loginfo("slikano")
        return cv_image

    def filter_image(self, data, lower, upper, name):
        hsv = cv2.cvtColor(data, cv2.COLOR_BGR2HSV)
        filteredImage = cv2.inRange(hsv, lower, upper)
        cv2.waitKey(delay=1)
        return filteredImage

    def populate_matrix(self, filteredImage, matrix, obstacleType):
        height = filteredImage.shape[0]
        width = filteredImage.shape[1]
        
        for i in range(width):
            for j in range(height):
                if filteredImage[j][i]:
                    matrix[j][i] = obstacleType
        return matrix

class CameraNode:
    matrix = np.zeros((620, 920))

    def __init__(self):
        rospy.init_node("camera_filter")
        rospy.loginfo("Starting camera_filter node.")

        self.camera_sub = rospy.Subscriber("webots_camera", Image, callback=self.callback_camera)

        self.cf_class = CameraFilter()

    def downscale_matrix(self, matrix):
        downscaledMatrix = np.zeros((31, 46))
        for i in range(31):
            for j in range(46):
                if matrix[i * 20][j * 20] == 1:
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            new_i = i + dx
                            new_j = j + dy
                            
                            # Proveri da li su novi indeksi u granicama matrice
                            if 0 <= new_i < 31 and 0 <= new_j < 46:
                                downscaledMatrix[new_i][new_j] = 1
                else:
                    downscaledMatrix[i][j] = matrix[i * 20][j * 20]
        return downscaledMatrix

    def printMatrix(self, matrix):
        for i in range(31):
            for j in range(46):
                print(int(matrix[i][j]), end=" ")
            print(" ")

    def callback_camera(self, image):
        cutImage = self.cf_class.cut_image(image)
        filteredImage = self.cf_class.filter_image(cutImage, np.array([22, 93, 0]), np.array([45, 255, 255]), 'yellow')
        self.matrix = self.cf_class.populate_matrix(filteredImage, self.matrix, TableFieldType.FIELD_OBSTACLE.value)
        #objekat(svetlo plava)
        filteredImage = self.cf_class.filter_image(cutImage, np.array([87,85,185]), np.array([180, 255, 255]), 'lightblue')
        self.matrix = self.cf_class.populate_matrix(filteredImage, self.matrix, TableFieldType.FIELD_OBJECT.value)
        #cilj1(crvena)
        filteredImage = self.cf_class.filter_image(cutImage, np.array([0, 50, 50]), np.array([12, 255, 255]), 'red')
        self.matrix = self.cf_class.populate_matrix(filteredImage, self.matrix, TableFieldType.FIELD_ENDING1.value) 
        #cilj2(ljubicasta)
        filteredImage = self.cf_class.filter_image(cutImage, np.array([125, 50, 50]), np.array([150, 255, 255]), 'purple')
        self.matrix = self.cf_class.populate_matrix(filteredImage, self.matrix, TableFieldType.FIELD_ENDING2.value)       
        #robot(plava)
        filteredImage = self.cf_class.filter_image(cutImage, np.array([118, 50, 50]), np.array([125, 255, 255]), 'blue')
        self.matrix = self.cf_class.populate_matrix(filteredImage, self.matrix, TableFieldType.FIELD_ROBOT.value)  
        
        downscaled_matrix = self.downscale_matrix(self.matrix)
        self.printMatrix(downscaled_matrix)

        np.savetxt('/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/matricaa.txt', downscaled_matrix, fmt='%.1d')
        self.matrix = np.zeros((620, 920))
        
if __name__ == "__main__":
    node = CameraNode()
    rospy.spin()
