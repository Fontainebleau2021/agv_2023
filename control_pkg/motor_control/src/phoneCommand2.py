#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import rospy
import sys, select, termios, tty
import cv2
import os
import numpy as np
from threading import Thread
from customProtocol import *
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Int32, Float64MultiArray, UInt8MultiArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import threading
import sys
import time
from TcpServerHandler import *
from customProtocol import *

from nav_msgs.msg import OccupancyGrid

K_vel = 1000.0
#K_vel = 400.0
#K_omega = 1500.0
K_omega = 6000.0
vel_last = 0.0
omega_last = 0.0
v_last=0.0
sendPosArray = [0xff, 0xfe, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
CarCurPosBuffLen = 10
CarCurPosBuff = []      # car current pos reserve in this buffer
timeCnt = 0
# 0 for system init;1 for mapping;2 for location
state=0

def speedCacu(msg):
    global K_vel
    global K_omega
    global vel_last
    global omega_last
    v = 0.0
    omega = 0.0
    #停止
    if (msg[0] == 0):
        v = 0.0
        omega = 0.0
    #前进 或 后退
    elif (msg[0] == 1 ):
        v = msg[1]/K_vel
        omega = 0.0
    elif (msg[0] == 2 ):
        v = -msg[1]/K_vel
        omega = 0.0
    #左转
    elif (msg[0] == 3):
        v = vel_last
        omega = msg[1]/K_omega
    #右转
    elif (msg[0] == 4):
        v = vel_last
        omega = -msg[1] / K_omega
    #摇杆
    elif (msg[0] == 5):
        v = msg[1]/K_vel
        omega = msg[2] / K_omega
    #记录速度值
    vel_last = v
    vel_omega = omega
    return v, omega

def currentPoseUpdateCallback(msg):
    global CarCurPosBuff
    curPos = msg.data
    #print(curPos)
    if(len(curPos) != 3):
        return
    x = int(curPos[0])
    y = int(curPos[1])
    angle = int(curPos[2])
    CarCurPosBuff.append([x, y, angle])
    if(len(CarCurPosBuff) > CarCurPosBuffLen):
        CarCurPosBuff.pop(0)

def sendCarCurPos():
    global sendPosArray
    global CarCurPosBuff
    if(len(CarCurPosBuff) == 0):
        print("no car position to send")
        return
    curPos = CarCurPosBuff.pop(0)
    x = int(curPos[0])
    y = int(curPos[1])
    angle = int(curPos[2])
    sendPosArray[4] = (x >> 8) & 0xff
    sendPosArray[5] = x & 0xff
    sendPosArray[6] = (y >> 8) & 0xff
    sendPosArray[7] = y & 0xff
    sendPosArray[8] = (angle >> 8) & 0xff
    sendPosArray[9] = angle & 0xff
    crc = calc_crc(sendPosArray, 10)
    sendPosArray[10] = (crc >> 8) & 0xff
    sendPosArray[11] = crc & 0xff
    TcpServer.send(bytearray(sendPosArray))

def thread_job():
    rospy.spin()
    
def imageCallBack(msg):
    if(state != 2):
        return
    global image
    image=br.imgmsg_to_cv2(msg)
    img=cv2.imencode(".jpg",image)[1].tobytes()
    #print('11111111111')
    TcpServer.sendMatImage(img)
    #cv2.imwrite('test.jpg',image)
    
def mapCallBack(msg):
    if(state != 1):
        return
    print("get Map")
    global map
    map=msg.data
    info=msg.info
    map=np.array(map)
    # print(map.shape)
    map=map.reshape((info.height,info.width))
    row,col=map.shape
    imgM=np.zeros((row,col))
    for i in range(row):
        for j in range(col):
            if(map[i,j]==-1):
                imgM[i,j]=150
            else:
                imgM[i,j]=int((100-map[i,j])/100.0*255.0)
    # print(map.shape)
    imgM=cv2.flip(imgM, 0)
    # cv2.imshow("test",imgM)
    # cv2.waitKey(0)
    # print(type(imgM))
    img=cv2.imencode(".jpg",imgM)[1].tobytes()
    
    TcpServer.sendMatImage(img)

def flagCallBack(msg):
    global flag
    flag=msg.data
    # if(flag==1):
    #     rospy.loginfo("warning,a person in the front of the car")

if __name__ == '__main__':
    rospy.init_node('velocity', anonymous=True)
    rospy.loginfo("TCP init.......")
    # Watcher()
    turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    robot_pose_pub = rospy.Publisher('/robot_pose', Float64MultiArray, queue_size=10)               # 单目标点
    robot_multi_pose_pub = rospy.Publisher('/robot_multi_pose', Float64MultiArray, queue_size=10)   # 多目标点
    photo_upload_notify_pub = rospy.Publisher('/photo_upload_notify', Int32, queue_size=10)         # 跟踪实时图片上传通知  0:停止上传  1:开始上传
    map_upload_notify_pub = rospy.Publisher('/map_upload_notify', Int32, queue_size=10)
    stm32_charge_pub = rospy.Publisher('charge_cmd', UInt8MultiArray, queue_size=10)
     
    add_thread = threading.Thread(target = thread_job)
    add_thread.setDaemon(True)
    add_thread.start()

    br=CvBridge()
    rospy.Subscriber("/detector/images", Image,imageCallBack, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/detector/flag", Int16,flagCallBack, queue_size=1)
    rospy.Subscriber("/current_pose", Float64MultiArray, currentPoseUpdateCallback, queue_size=1)
    rospy.Subscriber("/map",OccupancyGrid,mapCallBack,queue_size=1)
    rate = rospy.Rate(8) 
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0

    TcpServer = TcpServerHandler('', 6000, customProtocol)
    TcpServer.setDaemon(True)
    TcpServer.start()

    while not rospy.is_shutdown():
        msg = TcpServer.recv()
        #rospy.Subscriber("/map",OccupancyGrid,mapCallBack,queue_size=1)
        if(msg != None):
            if (msg[0] == 6 or msg[0] == 7):
                if(msg[0] == 7):
                    msg.append(0)
                msg.pop(0)
                robotPos = Float64MultiArray(data=msg)
                robot_pose_pub.publish(robotPos)
                rospy.loginfo("Publsh robot Pos[%0.2f m/s, %0.2f rad/s, %0.2f rad]", msg[0], msg[1], msg[2])
            elif (msg[0] <= 5):
                (v, omega) = speedCacu(msg)
                if(v!=v_last):
                    v_temp=v_last
                    delta=v-v_temp
                    i=0
                    while i<2:
                        v_temp=v_temp+delta/2
                        vel_msg.linear.x = v_temp
                        vel_msg.angular.z = omega
                        turtle_vel_pub.publish(vel_msg)
                        time.sleep(1)
                        i+=1
                    
                v_last=v
                vel_msg.linear.x = v
                vel_msg.angular.z = omega
                # 发布消息
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)
            elif (msg[0] == 8):
                photo_upload_notify_pub.publish(msg[1])
                #location
                if(msg[1]==0):
                    state=0
                else:
                    state=2
            elif (msg[0] == 9):
                #mapping
                map_upload_notify_pub.publish(msg[1])
                if(msg[1]==0):
                    state=0
                else:
                    state=1
            elif (msg[0] == 0x0A or msg[0] == 0x0B):
                #[x1, y1, x2, y2,..., angle]
                if(msg[0] == 0x0A):
                    msg.append(0)
                msg.pop(0)
                robotPos = Float64MultiArray(data=msg)
                robot_multi_pose_pub.publish(robotPos)
            elif (msg[0]==0x12):
                charge_msg = UInt8MultiArray(data=msg)
                stm32_charge_pub.publish(charge_msg)

        timeCnt += 1
        # send car current position per second
        if(timeCnt % 8 == 0):
            sendCarCurPos()
        rate.sleep()
    TcpServer.close()
