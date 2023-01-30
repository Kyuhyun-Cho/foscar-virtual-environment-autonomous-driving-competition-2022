#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,os
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus
from lib.utils import pathReader,findLocalPath,purePursuit,latticePlanner
from obstacle_detection.msg import Boundingbox
import tf
import time
from math import *

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
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.local_path_pub= rospy.Publisher('/local_path', Path, queue_size=1) ## local_path publisher
        self.target_waypoint_pub = rospy.Publisher('/target_waypoint', Marker, queue_size=1)
    
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        
        # Subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLightCB) ## TrafficLight
        rospy.Subscriber("/rt_obs_position", Boundingbox, self.rtObstacleCB)
        rospy.Subscriber("/dyst_obs_position", Boundingbox, self.dystObstacleCB)

        rospy.Subscriber("/is_track_path", Bool, self.isTrackPathCB)
        rospy.Subscriber("/track_path", Path, self.trackPathCB)
   

        self.is_status=False ## WeBot 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.steering_angle_to_servo_offset=0.5 ## servo moter offset
        self.target_x = 0
        self.target_y = 0
        self.rpm_gain = 4616
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        

        # Mission Area Point FORMAT: min_x, min_y, max_x, max_y
        self.traffic_area = Point(6.85, -2.5, 7.5, -1.95)
        self.rotary_area = Point(10.0, -2.7, 15.57, 3.5)
        self.rotary_stop_area = Point(12.1, 1.8, 12.8, 2.2) # 좀 뒤로 미룰 필요 있음
        self.dynamic_obs_area = Point(1.72, 4.0, 6.73, 5.72)
        self.static_obs_area = Point(-13.66, -5.74, -6.71, -4.65)
        self.track_area = Point(-20.00, -6.00, -12.50, 6.00)
        self.track_area_2 = Point(-13.50, 4.50, -7.00, 6.00)



        # Traffic Mission Parameter
        self.traffic_greenlight = False


        # Rotary Mission Parameter
        self.rt_obstacle_x = 0
        self.rt_obstacle_y = 0
        self.rt_obstacle_dis = 0
        self.is_rotary_stopped = False


        # Dynamic & Static Obstacle Mission Parameter
        self.is_dynamic = False
        self.is_static = False

        self.obstacle_x = 0
        self.obstacle_y = 0
        self.obstacle_dis = 0
        self.obstacle_y_list = []

        self.finish_detection = False

        self.current_lane = 1

        # Track Mission Paraemter
        self.is_track_path = False
        self.track_path = Path()
        self.is_track_stopped = False
        self.track_motor_msg = 0

        # Class
        path_reader=pathReader('wecar_ros') ## 경로 파일의 위치
        pure_pursuit=purePursuit() ## purePursuit import
        

        # Read path
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

 
        # Time var
        count = 0
        rate = rospy.Rate(30) # 30hz
                                           
        #  0 1 2 -> 총 3개의 lattice 중 가운데 lattice == 1
        lattice_current_lane = 1
        while not rospy.is_shutdown():
            if self.is_status==True: ## WeBot 상태, 장애물 상태 점검

                ################################ SECOND MAP CHANGE ################################
                # Path 교체
                # if self.isTimetoChangePath():
                #     self.path_name = "second"
                #     self.global_path=path_reader.read_txt(self.path_name+".txt")
                ################################ SECOND MAP CHANGE ################################

                ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 


                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi, self.status_msg.velocity.x/3.6]
                lattice_path, selected_lane = latticePlanner(local_path, vehicle_status, lattice_current_lane)
                lattice_current_lane = selected_lane

                # 최소 가중치를 갖는 lattice path 선택하기 
                if selected_lane != -1: 
                    if self.isMissionArea(self.rotary_area.x1, self.rotary_area.y1, self.rotary_area.x2, self.rotary_area.y2):
                        local_path = lattice_path[1]
                    
                    elif (self.isMissionArea(self.dynamic_obs_area.x1, self.dynamic_obs_area.y1, self.dynamic_obs_area.x2, self.dynamic_obs_area.y2) or 
                    self.isMissionArea(self.static_obs_area.x1, self.static_obs_area.y1, self.static_obs_area.x2, self.static_obs_area.y2)):
                        local_path = lattice_path[self.current_lane]

                    else:
                        local_path = lattice_path[selected_lane]
                        self.current_lane = 1     

                # lattice path visualization을 위한 path publish 과정
                if len(lattice_path)==3:                    
                    for i in range(1,4):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg, self.is_track_path) ## pure_pursuit 알고리즘에 WeBot status 적용

                self.steering, self.target_x, self.target_y = pure_pursuit.steering_angle()
        

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
                else: self.motor_msg = 2500 # 2500

                
                ################################################################ 트랙 구간 ################################################################
                if self.isMissionArea(self.track_area.x1, self.track_area.y1, self.track_area.x2, self.track_area.y2) or self.isMissionArea(self.track_area_2.x1, self.track_area_2.y1, self.track_area_2.x2, self.track_area_2.y2):
                    if self.is_track_path:  
                        local_path = self.track_path
                        self.local_path_pub.publish(local_path)
                        self.visualizeTargetPoint(self.target_x, self.target_y)
                        
                        if self.track_motor_msg > 990:
                            self.track_motor_msg = 990
                        self.motor_msg = self.track_motor_msg + 10
                        self.track_motor_msg = self.motor_msg
                        # print("TRACK SPEED: ", self.track_motor_msg)


                        pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                        pure_pursuit.getEgoStatus(self.status_msg, self.is_track_path) ## pure_pursuit 알고리즘에 WeBot status 적용

                        self.steering, self.target_x, self.target_y = pure_pursuit.steering_angle()
                        self.servo_msg = self.steering * 0.021 + self.steering_angle_to_servo_offset
                        
                        # if not self.is_track_stopped:
                        #     for i in range(1000):
                        #         self.local_path_pub.publish(local_path)
                        #         self.visualizeTargetPoint(self.target_x, self.target_y)
                        #         self.publishMotorServoMsg(0, self.servo_msg)
                        #         time.sleep(0.001)
                            
                        #     self.is_track_stopped = True
                        #     continue
                        
                        # if self.target_x == 0 and self.target_y == 0:
                        #     self.publishMotorServoMsg(0, self.servo_msg)
                        # else:

                        self.publishMotorServoMsg(self.motor_msg, self.servo_msg)

                        # self.local_path_pub.publish(local_path)
                        self.visualizeTargetPoint(self.target_x, self.target_y)
                        
                        rate.sleep()
                        continue
                ################################################################ 트랙 구간 ################################################################



                ######################################################### 동적 + 정적 장애물 구간 #########################################################
                if (self.isMissionArea(self.dynamic_obs_area.x1, self.dynamic_obs_area.y1, self.dynamic_obs_area.x2, self.dynamic_obs_area.y2) or 
                    self.isMissionArea(self.static_obs_area.x1, self.static_obs_area.y1, self.static_obs_area.x2, self.static_obs_area.y2)):
                    self.motor_msg = 800
                    if 0.5 < self.obstacle_x < 1.0 and self.finish_detection == False :
                        self.publishMotorServoMsg(0, self.servo_msg)
                        self.obstacle_y_list.append(self.obstacle_y)
                        # print("우선 멈춤")
                        if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                            self.finish_detection = True
                            self.obstacle_y_list.sort()
                            # print(self.obstacle_y_list)
                            print(abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]))
                            if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08 :
                                self.is_dynamic= True
                            elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                                self.is_static = True
                        continue

                    if self.is_dynamic :
                        if -0.25 < self.obstacle_y < 0.25 :
                            self.publishMotorServoMsg(0, self.servo_msg)
                            self.obstacle_y_list = []
                            print("동적 멈춤")
                            continue
                        else :
                            self.is_dynamic = False
                            self.obstacle_y_list = []

                    elif self.is_static:
                        if self.finish_detection == True : 
                            if self.current_lane == 1:
                                self.current_lane = 0
                                local_path = lattice_path[self.current_lane]

                            elif self.current_lane == 0:
                                self.current_lane = 1
                                local_path = lattice_path[self.current_lane]
                            # print(abs(self.status_msg.position.y - lattice_path[self.current_lane].poses[-1].pose.position.y))
                            # while( abs(self.status_msg.position.y - lattice_path[self.current_lane].poses[-1].pose.position.y) > 0.39):
                            #     pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                            #     pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 WeBot status 적용
                            #     self.steering=pure_pursuit.steering_angle()
                            #     self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset
                            #     self.publishMotorServoMsg(self.motor_msg, self.servo_msg)
                            #     print("정적 경로변경중")
                            self.is_static = False
                            self.obstacle_y_list = []
                            continue
                    

                    ## 장애물을 지나갔다는 판단
                    if self.obstacle_x < 0.5 :
                        self.finish_detection = False
                        self.obstacle_y_list = []
                ######################################################### 동적 + 정적 장애물 구간 #########################################################



                ############################################################### 신호등 구간 ###############################################################
                if self.isMissionArea(self.traffic_area.x1, self.traffic_area.y1, self.traffic_area.x2, self.traffic_area.y2):
                    if self.traffic_greenlight == False:
                        self.motor_msg = 0

                    # 브레이크 Flag 초기화    
                    self.is_track_stopped = False
                    self.is_rotary_stopped = False
                    self.track_motor_msg = 100
                ############################################################### 신호등 구간 ###############################################################



                ############################################################### 로터리 구간 ###############################################################         
                if self.isMissionArea(self.rotary_area.x1, self.rotary_area.y1, self.rotary_area.x2, self.rotary_area.y2):
                    self.motor_msg = 1000
                    if self.isMissionArea(self.rotary_stop_area.x1, self.rotary_stop_area.y1, self.rotary_stop_area.x2, self.rotary_stop_area.y2):
                        if not self.is_rotary_stopped:
                            for i in range(1000):
                                self.publishMotorServoMsg(0, self.servo_msg)
                                time.sleep(0.001)

                            self.is_rotary_stopped = True

                    if (self.rt_obstacle_x != 0 or self.rt_obstacle_y != 0):
                        rt_dis = sqrt(self.rt_obstacle_x ** 2 + self.rt_obstacle_y ** 2)
                        if (rt_dis < 1.0):
                            self.motor_msg = self.motor_msg * 0.0
                        elif (1.0 <= rt_dis < 1.2):
                            self.motor_msg = self.motor_msg * 0.2
                        elif (1.2 <= rt_dis < 1.4):
                            self.motor_msg = self.motor_msg * 0.4
                ############################################################### 로터리 구간 ###############################################################



                # Local Path 출력
                self.local_path_pub.publish(local_path)
                self.visualizeTargetPoint(self.target_x, self.target_y)
                self.publishMotorServoMsg(self.motor_msg, self.servo_msg)
                # self.print_info()
            
            # global path 출력
            if count==300 : 
                self.global_path_pub.publish(self.global_path)
                count=0
            count+=1
            
            rate.sleep()


    def print_info(self):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        print('heading :{} deg'.format(self.status_msg.heading))

        print('--------------------controller-------------------------')
        print('target steering_angle :{} deg'.format(self.steering))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))


    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True


    # def calcDistance(self, x, y):
    #     dx = self.status_msg.position.x - x
    #     dy = self.status_msg.position.y - y
    #     distance = sqrt(dx ** 2 + dy ** 2)

    #     return distance

            
    # def isTimetoChangePath(self):
    #     second_map_x = 0.527515172958
    #     second_map_y = -5.40659809113
    #     dis = self.calcDistance(second_map_x, second_map_y)

    #     if dis < 0.3:
    #         return True


    def isMissionArea(self, x1, y1, x2, y2):
        if (x1 <= self.status_msg.position.x <= x2) and (y1 <= self.status_msg.position.y <= y2):
            return True
        else:
            return False
     

    def trafficLightCB(self, data) :    
        if self.isMissionArea(self.traffic_area.x1, self.traffic_area.y1, self.traffic_area.x2, self.traffic_area.y2):
            if data.trafficLightStatus == 16 : # green light == 16
                self.traffic_greenlight = True
            else : 
                self.traffic_greenlight = False

    
    def rtObstacleCB(self, data):
        self.rt_obstacle_x = data.x
        self.rt_obstacle_y = data.y
        self.rt_obstacle_dis = data.distance


    def dystObstacleCB(self, data):
        self.obstacle_x = data.x
        self.obstacle_y = data.y
        self.obstacle_dis = data.distance


    def isTrackPathCB(self, data):
        self.is_track_path = data.data
        if self.isMissionArea(self.track_area.x1, self.track_area.y1, self.track_area.x2, self.track_area.y2) or self.isMissionArea(self.track_area_2.x1, self.track_area_2.y1, self.track_area_2.x2, self.track_area_2.y2):
            if self.is_track_path:
                self.is_track_path = True
        else:
            self.is_track_path = False


    def trackPathCB(self, data):
        self.track_path = data 


    def publishMotorServoMsg(self, motor_msg, servo_msg):
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)


    def visualizeTargetPoint(self, x, y):
        # print(x, y)
        target_waypoint = Marker()
        target_waypoint.header.frame_id = "/velodyne"
        target_waypoint.id = 1001
        target_waypoint.type = target_waypoint.SPHERE
        target_waypoint.action = target_waypoint.ADD
        target_waypoint.scale.x = 0.1
        target_waypoint.scale.y = 0.1
        target_waypoint.scale.z = 0.1
        target_waypoint.color.a = 1.0
        target_waypoint.color.r = 0.6
        target_waypoint.color.g = 0.7
        target_waypoint.color.b = 0.8
        target_waypoint.pose.orientation.w = 1.0
        target_waypoint.pose.position.x = x
        target_waypoint.pose.position.y = y
        target_waypoint.pose.position.z = 0.0
        target_waypoint.lifetime = rospy.Duration(0.1)
        self.target_waypoint_pub.publish(target_waypoint)

if __name__ == '__main__':
    try:
        kcity_pathtracking=wecar_planner()
    except rospy.ROSInterruptException:
        pass