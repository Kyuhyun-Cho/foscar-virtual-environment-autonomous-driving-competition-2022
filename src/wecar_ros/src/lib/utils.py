#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import *
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi

class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        openFile.close()
        return out_path
      

# global_path에서  local_path 따로 구분하기
def findLocalPath(ref_path,status_msg):
    out_path=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)

        # WeBot 기준 가장 가까운 waypoint 식별하기
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i

    # 가장 가까운 waypoint에서 +50번째에 있는 waypoint까지를 하나의 local_path로 간주하기
    if current_waypoint+50 > len(ref_path.poses) : # 만약 가장 가까운 waypoint에서 +50번째에 있는 waypoint를 찾고자 했으나 전체 global_path 인덱스를 초과한다면 도착점에 가까워진 것이므로 global_path의 마지막 waypoint를 local_path의 종점으로 활용
        last_local_waypoint = len(ref_path.poses)
    else :
        last_local_waypoint = current_waypoint+50

    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path, current_waypoint


class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            
            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan


class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.lfd=1
        self.min_lfd= 0.5
        self.max_lfd= 5
        self.vehicle_length=0.3
        self.steering=0
        
    def getPath(self,msg):
        self.path=msg  #nav_msgs/Path 
    
    def getEgoStatus(self, msg, is_track_path):
        if is_track_path:
            self.current_vel=msg.velocity.x  #kph
            self.vehicle_yaw=0
            self.current_postion.x=0
            self.current_postion.y=0
            self.current_postion.z=0
        else:
            self.current_vel=msg.velocity.x  #kph
            self.vehicle_yaw=msg.heading/180*pi   # rad
            self.current_postion.x=msg.position.x
            self.current_postion.y=msg.position.y
            self.current_postion.z=msg.position.z

    def steering_angle(self):
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False

        for i in self.path.poses :
            path_point=i.pose.position
            dx= path_point.x - vehicle_position.x
            dy= path_point.y - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
            if rotated_point.x > 0 :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                # print(dis, self.lfd)
                if dis>= self.lfd :
                    
                    self.lfd=self.current_vel / 1.8
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    break

        theta=atan2(rotated_point.y,rotated_point.x)
        # print("TARGET_X: ", rotated_point.x, "TARGET_Y: ", rotated_point.y)
        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi #deg
            # print(self.steering)
            return self.steering, path_point.x, path_point.y
        else : 
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi
            # print(self.steering)
            return self.steering, path_point.x, path_point.y


########################  lattice  ########################
def latticePlanner(ref_path, vehicle_status, current_lane):
    out_path = []
    selected_lane = -1 # 기본값 -1
    lattic_current_lane = current_lane #  == 1
    look_distance=int(vehicle_status[3] * 3.6 * 0.2 * 2)

    if look_distance < 4 :
        look_distance = 4    # min 5m
    if look_distance > 7 :
        look_distance = 7    # max 8m

    if len(ref_path.poses) > look_distance :
        global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y) # local_path 첫번째 waypoint
        global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y) # local_path 두번쨰 waypoint
        global_ref_end_point = (ref_path.poses[look_distance].pose.position.x, ref_path.poses[look_distance].pose.position.y) # local_path look ahead distance 위치에 존재하는 waypoint
        
        theta=atan2(global_ref_start_next_point[1]-global_ref_start_point[1], global_ref_start_next_point[0]-global_ref_start_point[0]) # y편차, x편차
        translation=[global_ref_start_point[0], global_ref_start_point[1]]

        t=np.array([[cos(theta), -sin(theta),translation[0]], [sin(theta),cos(theta),translation[1]], [0,0,1]])
        det_t=np.array([[t[0][0],t[1][0], -(t[0][0]*translation[0]+t[1][0]*translation[1])], [t[0][1],t[1][1], -(t[0][1]*translation[0]+t[1][1]*translation[1]) ], [0,0,1]])

        world_end_point=np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
        local_end_point=det_t.dot(world_end_point)
        world_ego_vehicle_position=np.array([[vehicle_status[0]],[vehicle_status[1]],[1]])
        local_ego_vehicle_position=det_t.dot(world_ego_vehicle_position)

        # lattice 간격
        lane_off_set=[0.35, 0, -0.35]
        # lane_off_set=[0.35, 0, -0.35]
        local_lattice_points=[]

        ############################# lattice path 생성  #############################
        for i in range(len(lane_off_set)):
            local_lattice_points.append([local_end_point[0][0], local_end_point[1][0]+lane_off_set[i], 1])
            
        for end_point in local_lattice_points :
            lattice_path=Path()
            lattice_path.header.frame_id='map'
            x=[]
            y=[]
            x_interval=0.5
            xs=0
            xf=end_point[0]
            ps=local_ego_vehicle_position[1][0]

            pf=end_point[1]
            x_num=xf/x_interval

            for i in range(xs,int(x_num)) : 
                x.append(i*x_interval)
            
            a=[0.0, 0.0, 0.0, 0.0]
            a[0]=ps
            a[1]=0
            a[2]=3.0*(pf-ps)/(xf*xf)
            a[3]=-2.0*(pf-ps)/(xf*xf*xf)

            for i in x:
                result=a[3]*i*i*i + a[2]*i*i + a[1]*i + a[0]
                y.append(result)

            for i in range(0,len(y)) :
                local_result=np.array([[x[i]], [y[i]], [1]])
                global_result=t.dot(local_result)

                read_pose=PoseStamped()
                read_pose.pose.position.x=global_result[0][0]
                read_pose.pose.position.y=global_result[1][0]
                read_pose.pose.position.z=0
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                lattice_path.poses.append(read_pose)

            out_path.append(lattice_path)
        ############################# lattice path 생성  #############################

        add_point_size=int(vehicle_status[3]*4*3.6)
        # print('add point',add_point_size)
        if add_point_size>len(ref_path.poses)-2:
            add_point_size=len(ref_path.poses)

        # lattice path 크기
        elif add_point_size < 30 :
            add_point_size = 30
        
        for i in range(look_distance,add_point_size):
            if i+1 < len(ref_path.poses):
                tmp_theta=atan2(ref_path.poses[i+1].pose.position.y-ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x-ref_path.poses[i].pose.position.x)
                
                tmp_translation=[ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],[tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],[0,0,1]])

                for lane_num in range(len(lane_off_set)) :
                    local_result=np.array([[0],[lane_off_set[lane_num]],[1]])
                    global_result=tmp_t.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x=global_result[0][0]
                    read_pose.pose.position.y=global_result[1][0]
                    read_pose.pose.position.z=0
                    read_pose.pose.orientation.x=0
                    read_pose.pose.orientation.y=0
                    read_pose.pose.orientation.z=0
                    read_pose.pose.orientation.w=1
                    out_path[lane_num].poses.append(read_pose)
                    
        # lattice path 가중치
        lane_weight=[1, 0, 99999] #reference path 
        # collision_bool=[False, False, False]

        selected_lane = lane_weight.index(min(lane_weight)) # 최소 가중치를 갖는 lattice path 선택하기

    else :
        selected_lane=-1    

    return out_path, selected_lane

########################  lattice  ########################