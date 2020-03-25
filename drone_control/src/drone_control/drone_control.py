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

"""
Wripper for ros contolll of drone via python
"""


class drone_client():

    def __init__(self, ):
        # super(drone_client,self).__init__()
        """
        """
        self.current_position = Point()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.cv_image = False


        self.bridge = CvBridge()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_nav_clb)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_clb)
        self.goal_pub = rospy.Publisher("/goal", PoseStamped,queue_size=10)
        self.gripper_pub = rospy.Publisher("gripper", Bool, queue_size=10)

        self.flag = 0

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
    def setup(self, x, y, z, w)
        dist = sqrt((x - self.current_position.x)**2 + (y - current_position.y)**2 + (z - current_position.z)**2)
        tp = dist/0.1
        t0 = time.time()

    def set_position(self, x, y, z, w):            
        if (time.time() - t0 < tp):
            t = time.time() - t0
            for j in range(3):
                Qpos[j] = ((point[j] - Qpos0[j])/2)*(1 - cos(pi*t/tp)) + Qpos0[j]
            pose = Point()
            pose.x = Qpos[0]
            pose.y = Qpos[1]
            pose.z = Qpos[2]
            self.SetGoal(pose, w)
        else:
            self.flag = 2
            time.sleep(1)
