#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi


class wecar_planner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher 
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber

        #def
        self.is_status=False ## WeBot 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.steering_angle_to_servo_offset=0.5 ## servo moter offset
        self.rpm_gain = 4616
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        

        #class
        path_reader=pathReader('wecar_ros') ## 경로 파일의 위치
        pure_pursuit=purePursuit() ## purePursuit import
        self.cc = cruiseControl(0.5, 1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo = vaildObject() ## 장애물 유무 확인 
        pid=pidController() ## pidController import
        

        #read path
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        
        vel_planner=velocityPlanning(10,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,30)
        

        
        #time var
        count=0
        rate = rospy.Rate(30) # 30hz
                                           
        #  0 1 2 3 4 5 6 -> 총 7개의 lattice 중 가운데 lattice == 3
        lattice_current_lane = 3

        while not rospy.is_shutdown():
            print(self.is_status , self.is_obj)
                        
            if self.is_status==True  and self.is_obj==True: ## WeBot 상태, 장애물 상태 점검
                ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                
                # objectInfoCB함수를 통해 얻은 전체 장애물 리스트를 vo.all_object라는 리스트에 옮겨 담는 과정
                self.vo.get_object(self.object_num, self.object_info[0], self.object_info[1], self.object_info[2], self.object_info[3]) # 장애물 (개수, 타입, x좌표, y좌표, 속도)

                # WeBot의 현재 위치 값과 vo.all_object에 있는 장애물들의 정보를 연산하여 global_object와 local_object로 나누는 과정            
                global_obj, local_obj = self.vo.calc_vaild_obj([self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi]) # WeBot (x좌표, y좌표, 헤딩값)


                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi, self.status_msg.velocity.x/3.6]
                lattice_path, selected_lane = latticePlanner(local_path, global_obj, vehicle_status, lattice_current_lane)
                lattice_current_lane = selected_lane

                # 최소 가중치를 갖는 lattice path 선택하기 
                if selected_lane != -1: 
                    local_path = lattice_path[selected_lane]                
                
                # lattice path visualization을 위한 path publish 과정
                if len(lattice_path)==7:                    
                    for i in range(1,8):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                # 최종적으로 선택한 local path에 pure pursuit 적용
                self.cc.checkObject(local_path, global_obj, local_obj)

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 WeBot status 적용

                self.steering=pure_pursuit.steering_angle()
                
                self.cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg) ## advanced cruise control 적용한 속도 계획

                self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset # 조향값
                self.motor_msg = self.cc_vel *self.rpm_gain /3.6 # 속도값
            
                local_path_pub.publish(local_path) ## Local Path 출력

                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
                self.print_info()
            
            if count==300 : ## global path 출력
                global_path_pub.publish(self.global_path)
                count=0
            count+=1
            
            rate.sleep()


    def print_info(self):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        print('heading :{} deg'.format(self.status_msg.heading))

        print('--------------------object-------------------------')
        print('object num :{}'.format(self.object_num))
        for i in range(0,self.object_num) :
            print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3]))

        print('--------------------controller-------------------------')
        print('target vel_planning :{} km/h'.format(self.cc_vel))
        print('target steering_angle :{} deg'.format(self.steering))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))


    def statusCB(self,data): ## Vehicl Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True
                    
        # print(self.status_msg.yaw)

    def objectInfoCB(self,data): ## Object information Subscriber
        self.object_num = data.num_of_npcs + data.num_of_obstacle + data.num_of_pedestrian
        object_type=[] # 전체 장애물 타입 리스트: 보행자 0, NPC 차량 1, 정적장애물 2
        object_pose_x=[] # 전체 장애물 x 좌표 리스트
        object_pose_y=[] # 전체 장애물 y 좌표 리스트
        object_velocity=[] # 전체 장애물 속도 리스트

        # 전체 장애물의 정보를 타입 상관없이 우선 각 리스트에 저장하기        
        for num in range(data.num_of_npcs) : # npc = NPC 차량
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)
        
        for num in range(data.num_of_obstacle) : # obstacle = 정적 장애물 
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :  # pedstrian = 보행자
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        # self.object_info == 전체 장애물의 타입 리스트, x 좌표 리스트, y 좌표 리스트, 속도 리스트를 포함하는 리스트
        self.object_info=[object_type, object_pose_x, object_pose_y, object_velocity]

        # self.is_obj == 필드 위에 객체가 있는지 판단하는 boolean형 변수. self.object_info 리스트에 정보가 있을 때만 True여야 하는데 여기서는 그냥 True를 디폴트값으로 넣어버림. 수정 필요 
        self.is_obj=True
    
if __name__ == '__main__':
    try:
        kcity_pathtracking=wecar_planner()
    except rospy.ROSInterruptException:
        pass