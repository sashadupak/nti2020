#!/usr/bin/env python
# coding: utf-8

import time
import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
import tf.transformations
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from numpy import *

"""
Wripper for ros contolll of drone via python
"""


class drone_client():

    def __init__(self, ):
        # super(drone_client,self).__init__()
        """
        """
        self.current_position = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.cv_image = False


        self.bridge = CvBridge()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_nav_clb)
        self.image_sub = rospy.Subscriber("/d400/color/image_raw", Image, self.image_clb)
        self.goal_pub = rospy.Publisher("/goal", PoseStamped,queue_size=10)
        self.gripper_pub = rospy.Publisher("gripper", Bool, queue_size=10)

        self.flag = 0
        self.basket = None
        self.cubes = None
        self.c = 0

    def current_nav_clb(self, data):
        """
        callback of current position
        :param data:
        :return:
        """
        self.current_position = data.pose.position

        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                                                    data.pose.orientation.y,
                                                                                    data.pose.orientation.z,
                                                                                    data.pose.orientation.w))

    def image_clb(self, data):
        """
        get image from camera
        :param data:
        :return:
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def SetGoal(self, pose, yaw):
        """
        :param x: pose
        :param yaw:
        :return:
        """
        goal_msgs = PoseStamped()
        goal_msgs.header.frame_id = "map"
        goal_msgs.header.stamp = rospy.Time.now()
        goal_msgs.pose.position = pose

        quat = tf.transformations.quaternion_from_euler(0.,0.,yaw)
        goal_msgs.pose.orientation.x = quat[0]
        goal_msgs.pose.orientation.x = quat[1]
        goal_msgs.pose.orientation.x = quat[2]
        goal_msgs.pose.orientation.x = quat[3]
        self.goal_pub.publish(goal_msgs)

    def SetGripper(self, state):
        """
        open/close gripper

        :param state: true - is close, false - is open
        :return:
        """
        self.gripper_pub.publish(state)

    #def takeoff(self, height):
    def setup(self, x, y, z, w):
        dist = sqrt((x - self.current_position.x)**2 + (y - self.current_position.y)**2 + (z - self.current_position.z)**2)
        self.tp = dist/0.16
        self.t0 = time.time()
        self.x0 = self.current_position.x
	#print "1self.x0: ", self.x0
        self.y0 = self.current_position.y
        self.z0 = self.current_position.z

    def set_position(self, x, y, z, w):
        if (x!=self.current_position.x or y!=self.current_position.y or z!=self.current_position.z):          
            if (time.time() - self.t0 < self.tp):
                t = time.time() - self.t0
                pose1 = Point()
                pose1.x = ((x - self.x0)/2)*(1 - cos(pi*t/self.tp)) + self.x0
                pose1.y = ((y - self.y0)/2)*(1 - cos(pi*t/self.tp)) + self.y0
                pose1.z = ((z - self.z0)/2)*(1 - cos(pi*t/self.tp)) + self.z0
                #print "pose1:", pose1
                #print "x:",pose1.x
                #print "self.x0:",self.x0
                print "flag: ",self.flag
                self.SetGoal(pose1, w)
            else:
                self.flag += 1
                time.sleep(1)
        else:
            self.flag+=1

    def get_basket(self):
        img = self.cv_image
        print "get basket"
        img = self.cv_image
        if img is None:
            print "no image"
            return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_min = array((0, 0, 242), uint8)
        hsv_max = array((92, 255, 255), uint8)
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        edged = cv2.Canny(thresh, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        _,cnts, h = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        r_meters = 0.2
        min_radius = 640*r_meters/(2*self.current_position.z*tan(radians(69/2)))
        min_radius = 75*min_radius/100
        x = -1
        y = -1
        for c in cnts:
            print "next cnt"
            if cv2.contourArea(c) < (2*pi*min_size):
                print "small"
                continue
            (x,y),radius = cv2.minEnclosingCircle(c)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(img,center,radius,(0,255,0),2)
            cv2.circle(img, center, 2, (0, 255, 0),2)
            if radius > min_radius:
                x = int(x) #- 640/2
                y = int(y) #- 480/2
                basket = [x, y]
                max_radius = radius
                cv2.circle(img, (basket[0], basket[1]), 3, (255, 0, 0), 2)
                basket[0] += 0.09
                basket[0] = (basket[0]/640)*2*h*tan(radians(69/2))
                basket[1] = (basket[1]/480)*2*h*tan(radians(42/2))
                h=self.current_position.z
                basket[0] = (basket[0])*h*0.00175
                basket[1] = (basket[1])*h*0.00175
        cv2.imwrite("basket.jpg", img)
        cv2.imwrite("thresh.jpg", thresh)
        cv2.imwrite("edged.jpg", edged)
        if x == -1:
            return None
        else:
            return (x - 640/2, y - 480/2)
    
    def get_cubes(self):
        self.c+=1
        print "get cubes"
        img = self.cv_image
        if img is None:
            print "no image"
            return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_min = array((0, 0, 242), uint8)
        hsv_max = array((92, 255, 255), uint8)
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        edged = cv2.Canny(thresh, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        _,cnts, h = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        num = -1
        r_meters = 0.06
        min_size = 640*r_meters/(2*self.current_position.z*tan(69/2))
        min_size = 85*min_size/100
        min_radius = 640*r_meters/(2*self.current_position.z*tan(radians(69/2)))
        min_radius = 85*min_radius/100
        for c in cnts:
            print "next cnt"
            if cv2.contourArea(c) < (2*pi*min_size):
                print "small"
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = int0(box)
            cv2.drawContours(img,[box],0,(0,0,255),2)
            area = sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2) * sqrt((box[3][0] - box[0][1])**2 + (box[3][1] - box[1][1])**2)
            if (area > min_size) and (area < pi*min_radius**2):
                print "normal size"
                num += 1
                centers.append([(box[0][0] +  box[1][0] + box[2][0] + box[3][0])/4 - 640/2, (box[0][1] +  box[1][1] + box[2][1] + box[3][1])/4 - 480/2])
                cv2.circle(img,(centers[num][0]+640/2, centers[num][1]+480/2), 3,(0,0,0),2)
                centers[num][0] += 0.09
                h=self.current_position.z
                centers[num][0] *= -1
                centers[num][1] *= -1
                print "centers pixel" + str(centers[num][0]) + " " + str(centers[num][1])
                centers[num][0] = (centers[num][0])*h*0.00175
                centers[num][1] = (centers[num][1])*h*0.00175
                print "centers meters" + str(centers[num][0]) + " " + str(centers[num][1])
        cv2.imwrite("cubes"+str(self.c)+".jpg", img)
        cv2.imwrite("thresh"+str(self.c)+".jpg", thresh)
        cv2.imwrite("edged"+str(self.c)+".jpg", edged)
        if len(centers) == 0:
            return None
        else:
            return centers

    def rotate(self, x, y, z):
        print "rotete"
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        Rx = mat([[1, 0, 0],
             [0, cos(roll), -sin(roll)],
             [0, sin(roll), cos(roll)]])
        Ry = mat([[cos(pitch), 0, sin(pitch)],
             [0, 1, 0],
             [-sin(pitch), 0, cos(pitch)]])
        Rz = mat([[cos(yaw), -sin(yaw), 0],
             [sin(yaw), cos(yaw), 0],
             [0, 0, 1]])
        R = Rz*Ry*Rx
        X = R*mat([[x], [y], [z]])
        X[0] += self.current_position.x
        X[1] += self.current_position.y
        #X[2] += self.current_position.z
        return X
