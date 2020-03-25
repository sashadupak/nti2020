#!/usr/bin/env python
# coding: utf-8

from drone_control2 import *
from geometry_msgs.msg import PoseStamped
import rospy

rospy.init_node("test_node", anonymous=10)

nav = PoseStamped()
nav.pose.position.x = 1
nav.pose.position.y = -1
nav.pose.position.z = 1

nav.pose.orientation.z = 0.75
nav.pose.orientation.w = 0.75

drone = drone_client()
rate = rospy.Rate(10)
drone.flag = 4 #set one of the sectors

tim = 0
while drone.current_position is None:
    tim+=0.1
    time.sleep(0.1)
    print(tim)
drone.SetGoal(drone.current_position, drone.yaw)
sh = open("data.txt", 'w')

while (rospy.is_shutdown() is False):

    if (drone.current_position.x < 0) or (drone.current_position.x > 2.5) or (drone.current_position.y > 0) or (drone.current_position.y < -2.5) or (drone.current_position.z > 2):
        continue
	
    if drone.flag == 0:#take off
        if drone.current_position is None:
           continue
        point = Point()
        cv2.imwrite("0.jpg", drone.cv_image)
        point.x = 1.5
        point.y = -1.5
        point.z = 1
        drone.setup(point.x, point.y, point.z, 0)
        drone.SetGripper(False)
        drone.flag = 1
    elif drone.flag == 1:
        drone.set_position(point.x,point.y,point.z, 0)
        drone.SetGripper(False)
        print "point:", point
	
    elif drone.flag == 2: #Точка 1
        if drone.current_position is None:
           continue
        point = Point()
        cv2.imwrite("1.jpg", drone.cv_image)
        point.x = 0.8
        point.y = -0.8
        point.z = 1
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag = 3
    elif drone.flag == 3: #Точка 1
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 4: #Точка 2
        if drone.current_position is None:
           continue
        point = Point()
        cv2.imwrite("2.jpg", drone.cv_image)
        point = Point()
        point.x = 0.8
        point.y = -2.2
        point.z = 1
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag = 5
    elif drone.flag == 5: #Точка 2
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 6: #Точка 3
        if drone.current_position is None:
           continue
        point = Point()
        cv2.imwrite("photos/4.jpg", drone.cv_image)
        #point.x = 2.2
        #point.y = -2.2
        #point.z = 1
        #drone.setup(point.x, point.y, point.z, 0)
        #drone.flag = 9
        cv2.imwrite("3.jpg", drone.cv_image)
        drone.cubes=drone.get_cubes()
        drone.flag=100
    elif drone.flag == 7: #Точка 3
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 8: #Точка 4
        #cv2.imwrite("photos/4.jpg", drone.cv_image)
        #point.x = 2.2
        #point.y = -2.2
        #point.z = 1
        #drone.setup(point.x, point.y, point.z, 0)
        #drone.flag = 9
        drone.cubes=drone.get_cubes()
        drone.flag=100
    elif drone.flag == 9: #Точка 4
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 10: #Точка центр
        cv2.imwrite("photos/5.jpg", drone.cv_image)
        point.x = 1.5
        point.y = -1.5
        point.z = 1
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag = 11
    elif drone.flag == 11: #Точка центр
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 12: #Точка посадка
        cv2.imwrite("photos/6.jpg", drone.cv_image)
        point.x = 1.5
        point.y = -1.5
        point.z = 0
        drone.SetGripper(False)
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag=13 #Точка посадка
    elif drone.flag == 13: 
        drone.SetGripper(False)
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag == 100: #Точка кубик 2 метра
        globCoord = []
        #dist=[]
        #drone.cubes=drone.get_cubes()
        #for xc, yc in drone.cubes:
        #    dist.append(sqrt(xc**2+yc**2))
        #    print str(dist) +"  "+str(xc)+"  "+ str(yc)
        indexOfMin=drone.cubes.index(min(drone.cubes))#dist.index(min(dist))
        globCoord = drone.rotate(drone.cubes[indexOfMin][0], drone.cubes[indexOfMin][1], drone.current_position.z)
        point.x = globCoord[0]

        point.y = globCoord[1]
        point.z = globCoord[2]
        print "!!!"+str(point.x)+"!!"+str(point.y)+"!!"+str(point.z)+"!!!"
        drone.setup(point.x, point.y, drone.current_position.z, 0)
        drone.flag = 101
    elif drone.flag == 101: #Точка кубик 2 метра
        drone.set_position(point.x,point.y,drone.current_position.z, 0)
        drone.flag = 104
    elif drone.flag == 102: #Точка кубик 0.7 метра
        globCoord = []
        drone.cubes=drone.get_cubes()
        if drone.cubes is None:
            drone.flag = 0
            continue
        indexOfMin=drone.cubes.index(min(drone.cubes))#dist.index(min(dist))
        globCoord = drone.rotate(drone.cubes[indexOfMin][0], drone.cubes[indexOfMin][1], drone.current_position.z)
        point.x = globCoord[0]
        point.y = globCoord[1]
        if(drone.current_position.z > 0.7):
            point.z = 0.7
        else:
            point.z=drone.current_position.z
        print "!!!"+str(point.x)+"!!"+str(point.y)+"!!"+str(point.z)+"!!!"
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag=103 
    elif drone.flag==103: #Точка кубик 0.7 метра
        drone.set_position(point.x,point.y,point.z, 0)

    elif drone.flag==104: #Точка кубик 0.4 метра
        globCoord = []
        dist=[]
        #drone.cubes=drone.get_cubes()
        if drone.cubes is None:
            #drone.flag = 0
            continue
        indexOfMin=drone.cubes.index(min(drone.cubes))#dist.index(min(dist))
        globCoord = drone.rotate(drone.cubes[indexOfMin][0], drone.cubes[indexOfMin][1], drone.current_position.z)
        point.x = globCoord[0]
        point.y = globCoord[1]
        point.z = 0.4
        print "!!!"+str(point.x)+"!!"+str(point.y)+"!!"+str(point.z)+"!!!"
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag=105
    elif drone.flag==105: #Точка кубик 0.4 метра
        drone.set_position(point.x,point.y,point.z, 0)
    elif drone.flag==106: #Точка кубик 0 метров
        globCoord = []
        dist=[]
        #drone.cubes=drone.get_cubes()
        if drone.cubes is None:
            #drone.flag = 0
            continue
        indexOfMin=drone.cubes.index(min(drone.cubes))#dist.index(min(dist))
        globCoord = drone.rotate(drone.cubes[indexOfMin][0], drone.cubes[indexOfMin][1], drone.current_position.z)
        point.x = globCoord[0]
        point.y = globCoord[1]
        point.z = -0.1
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag=107
    elif drone.flag==107: #Точка кубик 0 метров
        drone.set_position(point.x,point.y,point.z, 0)
    elif drone.flag==108: #Точка с кубиком 2 метра
        drone.SetGripper(True)
        point.z = 2
        drone.setup(point.x, point.y, point.z, 0)
        drone.flag=109
    elif drone.flag==109: #Точка с кубиком 2 метра
        drone.SetGripper(True)
        drone.set_position(point.x,point.y,point.z, 0)
    elif drone.flag == 110:
        drone.SetGripper(False)
        try:
            pass
        except:
            continue
        else:
            drone.flag = 110
            break 
    rate.sleep()



    ###

    #drone.SetGoal(nav.pose.position, 1.56)
    #drone.SetGripper(False)

    print "current_position:", drone.current_position
