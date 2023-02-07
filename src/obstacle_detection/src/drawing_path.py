#!/usr/bin/env python
# -*- coding: utf-8 -*-

from threading import local
import numpy as np
from std_msgs.msg import *
import rospy
import rospkg
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from waypoint_maker.msg import Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import warnings


linspace_num = 200

class drawing_path():
    def __init__(self):
        global linspace_num
        warnings.filterwarnings("ignore")
        rospy.init_node('linspace')

        track_path_pub = rospy.Publisher('/track_path', Path, queue_size = 1)
        is_track_path_pub = rospy.Publisher('/is_track_path', Bool, queue_size = 1)
        rospy.Subscriber('/waypoint_info', Waypoint, self.callback)
        
        self.x_poses = []
        self.y_poses = []

        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            track_path=Path()
            track_path.header.frame_id='velodyne'
            is_track_path = False

            if len(self.x_poses) > 2 and len(self.y_poses) > 2:
                is_track_path = True
                try:
                    output = np.polyfit(self.x_poses, self.y_poses, 2)
                except:
                    pass
                polyfit = np.poly1d(output)

                x_path = np.linspace(min(self.x_poses), max(self.x_poses), 200)
                y_path = polyfit(x_path)
    
                for i in range(linspace_num) :
                    tmp_pose = PoseStamped()
                    tmp_pose.pose.position.x = x_path[i]
                    tmp_pose.pose.position.y = y_path[i]
                    tmp_pose.pose.position.z = 0.0
                    tmp_pose.pose.orientation.x = 0
                    tmp_pose.pose.orientation.y = 0
                    tmp_pose.pose.orientation.z = 0
                    tmp_pose.pose.orientation.w = 1
                    track_path.poses.append(tmp_pose)

            track_path_pub.publish(track_path)
                
            is_track_path_pub.publish(is_track_path)

            rate.sleep()


    def callback(self, msg):
        self.x_poses = list(msg.x_arr)[0:msg.cnt]
        self.y_poses = list(msg.y_arr)[0:msg.cnt]
        # self.x_poses = list(msg.x_arr)[0:1]
        # self.y_poses = list(msg.y_arr)[0:1]

        self.x_poses.insert(0, -0.05)
        self.y_poses.insert(0, 0)

        self.cnt = msg.cnt


if __name__ == '__main__':
    try:
        drawing = drawing_path()
    except rospy.ROSInterruptException:
        pass