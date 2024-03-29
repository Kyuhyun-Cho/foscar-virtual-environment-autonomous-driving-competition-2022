#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math 
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseStamped,Point
import tf
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd


# Parameters
k = 0  # look forward gain
Lfc = 3 # [m] look-ahead distance
WB = 0.3  # [m] wheel base of vehicle
steering_angle_to_servo_offset = 0.5304

current_v = 1000
dynamic_velocity = 7

present_x, present_y, present_yaw = 0, 0, 0

path_x = []
path_y = []

current_index = 0

txt_line_cnt = 0

MAX_VELOCITY = 18

is_one_lap_finished = False

status_msg=EgoVehicleStatus()

def statusCB(data):
    global status_msg
    status_msg=data
    br = tf.TransformBroadcaster()
    br.sendTransform((status_msg.position.x, status_msg.position.y, status_msg.position.z),
                    tf.transformations.quaternion_from_euler(0, 0, status_msg.heading/180*math.pi),
                    rospy.Time.now(),
                    "gps",
                    "map")
    is_status=True 

class State:
    def __init__(self, x = 0, y = 0, yaw = 0, v = current_v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y

        return math.hypot(dx, dy)

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
 
    def search_target_index(self, state):
        global txt_line_cnt
        # To speed up nearest point search, doing it at only first time.  
        if self.old_nearest_point_index is None:
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]

            d = np.hypot(dx, dy)

            ind = np.argmin(d)
            self.old_nearest_point_index = ind

        else:
            ind = self.old_nearest_point_index

            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])

                if distance_this_index < distance_next_index + 1:
                    break
                
                if (ind + 1) < len(self.cx):
                    ind = ind + 1
                else:
                    ind = ind 

                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * current_v + Lfc  # update look ahead distance


        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    curvature = 2.0 * math.sin(alpha) / Lf

    delta = math.atan2(2.0 * WB * math.sin(alpha), Lf) * 180/math.pi

    return delta, ind, tx, ty

# def read_txt(file_name):
#         full_file_name="/home/foscar/VEAC_2023/src/wecar_ros"+"/path/"+file_name
#         openFile = open(full_file_name, 'r')
#         out_path=Path()
        
#         out_path.header.frame_id='/map'
#         line=openFile.readlines()
#         for i in line :
#             tmp=i.split()
#             read_pose=PoseStamped()
#             read_pose.pose.position.x=float(tmp[0])
#             read_pose.pose.position.y=float(tmp[1])
#             read_pose.pose.position.z=float(tmp[2])
#             read_pose.pose.orientation.x=0
#             read_pose.pose.orientation.y=0
#             read_pose.pose.orientation.z=0
#             read_pose.pose.orientation.w=1
#             out_path.poses.append(read_pose)
        
#         openFile.close()
#         return out_path


# def findLocalPath_ref(ref_path,status_msg):
#     out_path=Path()
#     current_x=status_msg.position.x
#     current_y=status_msg.position.y
#     current_waypoint=0
#     min_dis=float('inf')

#     for i in range(len(ref_path.poses)) :
#         dx=current_x - ref_path.poses[i].pose.position.x
#         dy=current_y - ref_path.poses[i].pose.position.y
#         dis=math.sqrt(dx*dx + dy*dy)
#         if dis < min_dis :
#             min_dis=dis
#             current_waypoint=i


#     if current_waypoint+50 > len(ref_path.poses) :
#         last_local_waypoint= len(ref_path.poses)
#     else :
#         last_local_waypoint=current_waypoint+50

#     out_path.header.frame_id='map'
#     for i in range(current_waypoint,last_local_waypoint) :
#         tmp_pose=PoseStamped()
#         tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
#         tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
#         tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
#         tmp_pose.pose.orientation.x=0
#         tmp_pose.pose.orientation.y=0
#         tmp_pose.pose.orientation.z=0
#         tmp_pose.pose.orientation.w=1
#         out_path.poses.append(tmp_pose)

#     return out_path,current_waypoint


def findLocalPath(path_x, path_y, state_x, state_y): ## global_path와 차량의 status_msg를 이용해 현재waypoint와 local_path를 생성 ##
    global current_index, previous_index, txt_line_cnt

    current_x = state_x
    current_y = state_y
    min_dis = float('inf')

    for i in range(txt_line_cnt) :
        dx = current_x - path_x[i]
        dy = current_y - path_y[i]
        dis = math.sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis = dis
            current_index = i
    # previous_index = current_index

    if current_index == txt_line_cnt:
        current_index = 0


if __name__ == '__main__':
    rospy.init_node("pure_pursuit", anonymous=True)

    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, statusCB)

    global_path = read_txt("test_path.txt")

    global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
    local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher

    motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
    servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
    motor_msg = Float64()
    servo_msg = Float64()

    rate = rospy.Rate(60)

    # Path Setting
    f = open('/home/foscar/VEAC_2023/src/wecar_ros/path/test_path.txt' , mode = 'r')

    line = f.readline()
    first_line = line.split()
    path_x.append(float(first_line[0]))
    path_y.append(float(first_line[1]))

    while line:
        line = f.readline()
        tmp = line.split()

        txt_line_cnt += 1

        if len(tmp) != 0:
            path_x.append(float(tmp[0]))
            path_y.append(float(tmp[1]))
    f.close()
    
    path_x *= 2
    path_y *= 2
   

    count = 0

    while not rospy.is_shutdown():
        state = State(x = status_msg.position.x, y = status_msg.position.y, yaw = status_msg.heading/180*math.pi, v = current_v)

        local_path, current_waypoint = findLocalPath_ref(global_path, status_msg)

        findLocalPath(path_x, path_y, state.x, state.y)


        if len(path_x) != 0:
            # Calc control input
            target_course = TargetCourse(path_x, path_y)
            target_ind, lf = target_course.search_target_index(state)

            di, target_ind, target_x, target_y = pure_pursuit_steer_control(state, target_course, target_ind)
        


            if count==300 : ## global path 출력
                global_path_pub.publish(global_path)
                count=0
            count+=1



            motor_msg = 3000
            servo_msg = -1 * di / 30 + 0.5
            print(servo_msg)

            motor_pub.publish(motor_msg)
            servo_pub.publish(servo_msg)

        rate.sleep()