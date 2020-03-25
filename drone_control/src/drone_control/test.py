#!/usr/bin/env python
# coding: utf-8

from drone_control import *
from geometry_msgs.msg import PoseStamped
import rospy

rospy.init_node("test_node", anonymous=10)

nav = PoseStamped()
nav.pose.position.x = 1

nav.pose.orientation.z = 0.75
nav.pose.orientation.w = 0.75

drone = drone_client()
rate = rospy.Rate(10)
drone.flag = 0

while (rospy.is_shutdown() is False):

    if (drone.current_position.x < 0) or (drone.current_position.x > 2.5) or (drone.current_position.y > 0) or (drone.current_position.y < -2.5) or (drone.current_position.z > 2):
        continue

    if drone.flag == 0:
        drone.setup(1.25, -1.25, 2, 0)
        drone.flag = 1
    elif drone.flag == 1: 
        drone.set_position(1.25, -1.25, 2, 0)
    elif drone.flag == 2:
        try:
            imwrite("2_meters_height", drone.cv_image)
        except:
            continue
        else:
            drone.flag = 3
            break 
    rate.sleep()



    ###

    #drone.SetGoal(nav.pose.position, 1.56)
    #drone.SetGripper(False)

    #print "current_position:", drone.current_position
