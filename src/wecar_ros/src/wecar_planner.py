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
from lib.utils import pathReader,findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
from obstacle_detection.msg import Boundingbox
import tf
import time
from math import cos,sin,sqrt,pow,atan2,pi

class Point():
    def __init__(self, min_x, min_y, max_x, max_y):
        self.x1 = min_x
        self.y1 = min_y
        self.x2 = max_x
        self.y2 = max_y


class wecar_planner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]

        # Publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher 
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        
        # Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLightCB) ## TrafficLight
        rospy.Subscriber("/rt_obs_position", Boundingbox, self.rtObstacleCB)
        rospy.Subscriber("/dy_obs_position", Boundingbox, self.dyObstacleCB)
        rospy.Subscriber("/st_obs_position", Boundingbox, self.stObstacleCB)

        self.is_status=False ## WeBot 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.steering_angle_to_servo_offset=0.5 ## servo moter offset
        self.rpm_gain = 4616
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        
        # Mission Area Point FORMAT: min_x, min_y, max_x, max_y
        self.traffic_area = Point(6.85, -2.5, 7.5, -1.95)
        self.rotary_area = Point(10.0, -2.7, 15.57, 3.25)
        self.rotary_stop_area = Point(12.1, 1.6, 12.8, 2.0)
        self.dynamic_obs_area = Point(1.3, 4.0, 6.0, 5.75)

        # Traffic Mission Parameter
        self.traffic_greenlight = False

        # Rotary Mission Parameter
        self.rt_obstacle_x = 0
        self.rt_obstacle_y = 0
        self.rt_obstacle_dis = 0

        # Dynamic Obstacle Mission Parameter
        self.is_dynamic = False
        self.dy_obstacle_x = 0
        self.dy_obstacle_y = 0
        self.dy_obstacle_dis = 0
        self.y_array = []

        # Static Obstacle Mission Parameter
        self.st_obstacle_x = 0
        self.st_obstacle_y = 0
        self.st_obstacle_dis = 0
        self.st_obstacle_list = []

        self.isStopped = False


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
        lattice_current_lane = 1

        while not rospy.is_shutdown():
            # print(self.is_status , self.is_obj)
            
            # Path 교체
            if self.isTimetoChangePath():
                self.path_name = "second"
                self.global_path=path_reader.read_txt(self.path_name+".txt")

            if self.is_status==True  and self.is_obj==True: ## WeBot 상태, 장애물 상태 점검

                ################################ SECOND MAP CHANGE ################################
                
                ################################ SECOND MAP CHANGE ################################

                ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                
                # objectInfoCB함수를 통해 얻은 전체 장애물 리스트를 vo.all_object라는 리스트에 옮겨 담는 과정
                self.vo.get_object(self.object_num, self.object_info[0], self.object_info[1], self.object_info[2], self.object_info[3]) # 장애물 (개수, 타입, x좌표, y좌표, 속도)

                # WeBot의 현재 위치 값과 vo.all_object에 있는 장애물들의 정보를 연산하여 global_object와 local_object로 나누는 과정            
                global_obj, local_obj = self.vo.calc_vaild_obj([self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi]) # WeBot (x좌표, y좌표, 헤딩값)


                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi, self.status_msg.velocity.x/3.6]
                lattice_path, selected_lane = latticePlanner(local_path, self.st_obstacle_list, vehicle_status, lattice_current_lane)
                lattice_current_lane = selected_lane

                # 최소 가중치를 갖는 lattice path 선택하기 
                if selected_lane != -1: 
                    if self.isMissionArea(self.rotary_area.x1, self.rotary_area.y1, self.rotary_area.x2, self.rotary_area.y2):
                        local_path = lattice_path[1]
                    else:
                        local_path = lattice_path[selected_lane]                

                # lattice path visualization을 위한 path publish 과정
                if len(lattice_path)==3:                    
                    for i in range(1,4):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                # 최종적으로 선택한 local path에 pure pursuit 적용
                self.cc.checkObject(local_path, global_obj, local_obj)

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 WeBot status 적용

                self.steering=pure_pursuit.steering_angle()
                
                # self.cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg) ## advanced cruise control 적용한 속도 계획
                self.cc_vel = 10

                # 조향 값 확인 : rostopic echo /sensors/servo_position_command -> data
                # range : 0.0 ~ 1.0 (straight 0.5)
                self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset # 조향값

                # 속도 값 확인 : rostopic echo /sensors/core -> state -> speed
                # range : 0 ~ 2260.15e6
                # 실제 대입값과 차이 있음
                # command   topic
                # 500       450
                # 1000      878
                # 2000      1858
                # 3000      2231
                # 3500      2255
                # print("-------------------------------------------------")
                # print(abs(self.servo_msg - 0.5))
                # print("-------------------------------------------------")
                # self.motor_msg = self.cc_vel *self.rpm_gain /3.6 # 속도값

                servo_degree = abs(self.servo_msg - 0.5)
                if servo_degree > 0.2:
                    self.motor_msg = 1200
                elif servo_degree > 0.1:
                    self.motor_msg = 1300
                else: self.motor_msg = 1500 # 2500
                
                # 신호등 구간
                if self.isMissionArea(self.traffic_area.x1, self.traffic_area.y1, self.traffic_area.x2, self.traffic_area.y2):
                    if self.traffic_greenlight == False:
                        self.motor_msg = 0

                # 로터리 구간
                if self.isMissionArea(self.rotary_area.x1, self.rotary_area.y1, self.rotary_area.x2, self.rotary_area.y2):
                    if self.isMissionArea(self.rotary_stop_area.x1, self.rotary_stop_area.y1, self.rotary_stop_area.x2, self.rotary_stop_area.y2):
                        if not self.isStopped:
                            for i in range(1000):
                                self.publishMotorServoMsg(0, self.servo_msg)
                                time.sleep(0.001)

                        self.isStopped = True

                    if (self.rt_obstacle_x != 0 or self.rt_obstacle_y != 0):
                        rt_dis = sqrt(self.rt_obstacle_x ** 2 + self.rt_obstacle_y ** 2)
                        if (rt_dis < 0.8):
                            self.motor_msg = self.motor_msg * 0.0
                        elif (rt_dis < 1.0):
                            self.motor_msg = self.motor_msg * 0.2
                        elif (rt_dis < 1.2):
                            self.motor_msg = self.motor_msg * 0.4
                
                # 동적 장애물 구간
                if self.isMissionArea(self.dynamic_obs_area.x1, self.dynamic_obs_area.y1, self.dynamic_obs_area.x2, self.dynamic_obs_area.y2):
                    self.motor_msg = 800

                    if  0 < self.dy_obstacle_x < 3 :
                        self.y_array.append(self.dy_obstacle_y)
                        if len(self.y_array) >= 7 :
                            self.y_array.sort()
                            if(abs(self.y_array[-2] - self.y_array[2]) > 0.1) :
                                self.is_dynamic = True
                    else :
                        self.is_dynamic = False
                        self.y_array = []
                    
                    if self.is_dynamic :
                        self.publishMotorServoMsg(0, self.servo_msg)
                        continue

                # 정적 장애물 구간

                       
                local_path_pub.publish(local_path) ## Local Path 출력

                self.publishMotorServoMsg(self.motor_msg, self.servo_msg)
                # self.print_info()
            
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

    def calcDistance(self, x, y):
        dx = self.status_msg.position.x - x
        dy = self.status_msg.position.y - y
        distance = sqrt(dx ** 2 + dy ** 2)

        return distance


    def isMissionArea(self, x1, y1, x2, y2):
        if (x1 <= self.status_msg.position.x <= x2) and (y1 <= self.status_msg.position.y <= y2):
            return True
        else:
            return False
     
        
    def isTimetoChangePath(self):
        second_map_x = 0.527515172958
        second_map_y = -5.40659809113
        dis = self.calcDistance(second_map_x, second_map_y)

        if dis < 0.3:
            return True

 
    def trafficLightCB(self, data) :    
        # only green light -> go straight
        # green light == 16
        if self.isMissionArea(self.traffic_area.x1, self.traffic_area.y1, self.traffic_area.x2, self.traffic_area.y2):
            if data.trafficLightStatus == 16 : 
                self.traffic_greenlight = True
            else : 
                self.traffic_greenlight = False

    
    def rtObstacleCB(self, data):
        self.rt_obstacle_x = data.x
        self.rt_obstacle_y = data.y
        self.rt_obstacle_dis = data.distance

    def dyObstacleCB(self, data):
        self.dy_obstacle_x = data.x
        self.dy_obstacle_y = data.y
        self.dy_obstacle_dis = data.distance

    def stObstacleCB(self, data):
        self.st_obstacle_list = []
        self.st_obstacle_x = data.x
        self.st_obstacle_y = data.y
        self.st_obstacle_dis = data.distance

        self.st_obstacle_list.append(data.x)
        self.st_obstacle_list.append(data.y)
        self.st_obstacle_list.append(data.distance)


    def publishMotorServoMsg(self, motor_msg, servo_msg):
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

if __name__ == '__main__':
    try:
        kcity_pathtracking=wecar_planner()
    except rospy.ROSInterruptException:
        pass