#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from rospy.core import rospyinfo
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import csv
x = 0.0
y = 0.0
yaw = 0.0
dx=0
dy=0
odom_y=0
odom_x=0
rospy.init_node('car_odom')
pub = rospy.Publisher('/car', Path, queue_size=10)
rate=rospy.Rate(10)
path = Path()
path.header.frame_id = "map"
path.header.stamp = rospy.Time.now()
ang=0
with open('/home/hsk/catkin_ws/src/odom2/src/data.csv') as csv_file:
    row = csv.reader(csv_file, delimiter=',')

    next(row)
     # 读取首行
    encoder = [0,0,0,0]
    w = [0, 0, 0, 0] 
    last_enc = [0 for _ in range(4)]
    for r in row:
        encoder.append(float(r[4]))
        encoder.append(float(r[5]))
        encoder.append(float(r[6]))
        encoder.append(float(r[7]))
        for i in range(4):
            if (encoder[i] - last_enc[i] > 3.14):
                w[i] = encoder[i] - 2 * 3.1415926 - last_enc[i]
            elif (encoder[i] - last_enc[i] < -3.14):
                w[i] = encoder[i] + 2 * 3.1415926 - last_enc[i]
            else:

                w[i] = encoder[i] - last_enc[i]
        w[0] = 0.98284149 * (w[0] - 0.004916151)
        w[1] = 0.98708168 * (w[1] + 0.00114977)
        w[2] = 0.98601081 * (w[2] - 0.000181461)
        w[3] = 0.98734884 * (w[3] - 0.004097596)
        dy = (w[0] + w[1] + w[2] + w[3]) * 0.05 / 4
        dx = (-w[0] + w[1] + w[2] - w[3]) * 0.05 / 4
        yaw += (-w[0] + w[1] - w[2] + w[3]) / 0.2 * 0.05 / 4
        last_enc = encoder
        encoder = []
        
        odom_x += dx * math.cos(yaw) - dy * math.sin(yaw)
        odom_y += dx * math.sin(yaw) + dy* math.cos(yaw)
        
        
        # path.header.stamp = current_time
        
        pose = PoseStamped()
        pose.pose.position.x = odom_x
        pose.pose.position.y = odom_y
        pose.pose.position.z = 0
        # rospy.loginfo(yaw)
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        path.poses.append(pose)
        # msg.child_frame_id = "base_link"
        pub.publish(path)
        rate.sleep()
    


