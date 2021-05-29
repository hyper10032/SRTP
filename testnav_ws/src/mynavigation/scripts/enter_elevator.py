#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tf
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import time
import cv2


class EnterElevator:
    def __init__(self):
        self.num = 0  # 60-120cm内的点数量
        # self.marker_z = 100
        # self.marker_x = 100
        self.laser_sub = rospy.Subscriber(
            '/scan', LaserScan, self.laserCallback)
        self.enter_sub = rospy.Subscriber(
            '/enter', Pose, self.enterCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist,
                                       queue_size=1)
        self.pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        self.k = 999
        self.t = 999
        self.tf = tf.TransformListener()

    def laserCallback(self, msg):
        self.num = 0
        loc = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)*5/12, len(msg.ranges)*7/12):
            r = msg.ranges[i]
            if r >= 0.2 and r <= 2:
                self.num += 1            
        for i in range(len(msg.ranges)*5/12, len(msg.ranges)*7/12,6):
            a = angle_min + angle_increment * i
            r = msg.ranges[i]
            # if r >= 0.6 and r <= 1:
            #     self.num += 1
            # if r >= 0.2 and r <= 2:
            if r >= 0.2:
                # loc.append([math.cos(a) * r, math.sin(a) * r])
                loc.append([math.sin(a) * r, math.cos(a) * r])
        # print(len(loc))
        if len(loc) > 2:
            # print(loc)
            loc = np.array(loc)
            output = cv2.fitLine(loc, cv2.DIST_L2, 0, 0.01, 0.01)
            self.k = output[1][0] / output[0][0]
            self.t = output[3][0]-self.k*output[2][0]
            # dist = math.fabs(self.t)/math.sqrt(1+self.k*self.k)
            # print(self.k, self.t,dist)
            # print(self.k, self.t)

    def enterCallback(self, msg):
        if msg.position.z == 1:
            # # while rospy.get_param("mystate") != 1:
            # #     if rospy.get_param("myshut") == 1:
            # #         return
            # 左转
            t = 0.001
            for i in range(3800):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = 0.4
                self.vel_pub.publish(mycmd)
                time.sleep(0.001)
            time.sleep(1.5)
            # # 前进
            while True:
                # dist = math.fabs(self.t)/math.sqrt(1+self.k*self.k)
                try:
                    # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
                    #                          rmath.fabspTransform('/map', '/base_footprint',
                    #                                      rospy.Time(0))
                    (self.trans,
                     self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
                                                         rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException,
                        tf.ExtrapolationException):
                    print("get tf error!")
                # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
                y = self.trans[1]
                print(y)
                if math.fabs(2.47-y) <= 0.02:
                    break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = -0.4
                    mycmd.angular.z = 0
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.001)
            # time.sleep(0.5)
            # 右转
            for i in range(3000):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = -0.4
                self.vel_pub.publish(mycmd)
                time.sleep(t)
            time.sleep(1.5)
            while True:
                print(self.k,self.t)
                if math.fabs(self.k-(-0.05)) < 0.01:
                    time.sleep(t)
                    if math.fabs(self.k-(-0.05)) < 0.01:
                        print("stop")
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = 0
                    mycmd.angular.z = -0.3
                    self.vel_pub.publish(mycmd)
                    time.sleep(t)
            # 进电梯
            time.sleep(1)
            while True:
                print(self.num)
                if self.num < 30:
                    time.sleep(0.001)
                    if self.num < 30:
                        while True:
                            print(self.k,self.t)
                            mycmd = Twist()
                            mycmd.linear.x = -0.5
                            mycmd.angular.z = 0
                            self.vel_pub.publish(mycmd)
                            # print(self.num)
                            time.sleep(0.001)
                            if math.fabs(self.t)< 1.2:
                                time.sleep(0.01)
                                if math.fabs(self.t)< 1.2:
                                    break
                        break
            # 左转
            for i in range(3000):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = 0.4
                self.vel_pub.publish(mycmd)
                time.sleep(0.001)
            time.sleep(1.5)
            while True:
                print(self.k,self.t)
                if math.fabs(self.k) < 0.02:
                    time.sleep(0.001)
                    if math.fabs(self.k) < 0.02:
                        print("stop")
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = 0
                    mycmd.angular.z = 0.3
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.1)                  
            # 前进
            print("kkk")
            while True:
                print(self.k,self.t)
                if math.fabs(self.t)<0.4:
                    time.sleep(0.001)
                    if math.fabs(self.t)<0.4:
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = -0.3
                    mycmd.angular.z = 0
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.001) 
            # 左转
            print("lllll")
            for i in range(2200):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = 0.4
                self.vel_pub.publish(mycmd)
                time.sleep(0.001)
            time.sleep(1.5)
            while True:
                print(self.k,self.t)
                if math.fabs(self.k-(-0.37)) < 0.12:
                    time.sleep(0.001)
                    if math.fabs(self.k-(-0.37)) < 0.12:
                        # print("stop")
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = 0
                    mycmd.angular.z = 0.2
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.1) 
            # # 前进
            # while True:
            #     if self.t<0.4:
            #         time.sleep(0.001)
            #         if self.t<0.4:
            #             break
            #     else:
            #         mycmd = Twist()
            #         mycmd.linear.x = -0.4
            #         mycmd.angular.z = 0
            #         self.vel_pub.publish(mycmd)
            #         time.sleep(0.001)
            rospy.set_param("mystate", 2)
        else:
            # 左转
            for i in range(3700):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = 0.4
                self.vel_pub.publish(mycmd)
                time.sleep(0.001)
            time.sleep(1.5)
            while True:
                print(self.k,self.t)
                if math.fabs(self.k) < 0.02:
                    time.sleep(0.001)
                    if math.fabs(self.k) < 0.02:
                        print("stop")
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = 0
                    mycmd.angular.z = 0.3
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.1)            
            # 前进
            while True:
                print(self.k,self.t)
                if math.fabs(self.t)<0.85:
                    time.sleep(0.001)
                    if math.fabs(self.t)<0.85:
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = -0.3
                    mycmd.angular.z = 0
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.001) 
            # 右转
            for i in range(3000):
                mycmd = Twist()
                mycmd.linear.x = 0
                mycmd.angular.z = -0.4
                self.vel_pub.publish(mycmd)
                time.sleep(0.001)
            time.sleep(1.5)
            while True:
                print(self.k,self.t)
                if math.fabs(self.k) < 0.02:
                    time.sleep(0.001)
                    if math.fabs(self.k) < 0.02:
                        print("stop")
                        break
                else:
                    mycmd = Twist()
                    mycmd.linear.x = 0
                    mycmd.angular.z = -0.3
                    self.vel_pub.publish(mycmd)
                    time.sleep(0.1)    
            # 出电梯
            while True:
                print(self.num)
                if self.num < 30:
                    time.sleep(0.001)
                    if self.num < 30:
                        while True:
                            print(self.k,self.t)
                            mycmd = Twist()
                            mycmd.linear.x = -0.5
                            mycmd.angular.z = 0
                            self.vel_pub.publish(mycmd)
                            # print(self.num)
                            time.sleep(0.001)
                            if math.fabs(self.t)< 1.6:
                                time.sleep(0.01)
                                if math.fabs(self.t)< 1.6:
                                    break
                        break
            # while True:
            #     if self.num < 30:
            #         time.sleep(0.01)
            #         if self.num < 30:
            #             for i in range(num):
            #                 mycmd = Twist()
            #                 mycmd.linear.x = 0
            #                 mycmd.angular.z = -0.4
            #                 self.vel_pub.publish(mycmd)
            #                 time.sleep(0.001)
            #             # while True:
            #             #     mycmd = Twist()
            #             #     mycmd.linear.x = -0.6
            #             #     mycmd.angular.z = 0
            #             #     self.vel_pub.publish(mycmd)
            #             #     print(self.num)
            #             #     time.sleep(0.001)
            #             #     if self.num > 80:
            #             #         break
            #             break
            # # 左转
            # while True:
            #     try:
            #         # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #         #                          rospy.Duration(4.0))
            #         self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                                  rospy.Duration(4.0))
            #         # (self.trans,
            #         #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #         #                                      rospy.Time(0))
            #         (self.trans,
            #          self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                                              rospy.Time(0))
            #     except (tf.LookupException, tf.ConnectivityException,
            #             tf.ExtrapolationException):
            #         print("get tf error!")
            #     # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
            #     angle_base = math.asin(self.rot[2])/math.pi*180*2
            #     # print(str(angle_base)+"#####"+ str(math.fabs(90-math.fabs(angle_base)))+"#")
            #     if math.fabs(90-math.fabs(angle_base)) <= 4:
            #         break
            #     else:
            #         mycmd = Twist()
            #         mycmd.linear.x = 0
            #         mycmd.angular.z = 0.4
            #         self.vel_pub.publish(mycmd)
            #         time.sleep(0.001)
            # # 前进
            # while True:
            #     try:
            #         # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #         #                          rospy.Duration(4.0))
            #         self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                                  rospy.Duration(4.0))
            #         # (self.trans,
            #         #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #         #                                      rospy.Time(0))
            #         (self.trans,
            #          self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                                              rospy.Time(0))
            #     except (tf.LookupException, tf.ConnectivityException,
            #             tf.ExtrapolationException):
            #         print("get tf error!")
            #     # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
            #     y = self.trans[1]
            #     # print(y)
            #     if math.fabs(2.45-y) <= 0.02:
            #         break
            #     else:
            #         mycmd = Twist()
            #         mycmd.linear.x = -0.4
            #         mycmd.angular.z = 0
            #         self.vel_pub.publish(mycmd)
            #         time.sleep(0.001)
            # # 右转
            # while True:
            #     try:
            #         # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #         #                          rospy.Duration(4.0))
            #         self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                                  rospy.Duration(4.0))
            #         # (self.trans,
            #         #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #         #                                      rospy.Time(0))
            #         (self.trans,
            #          self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                                              rospy.Time(0))
            #     except (tf.LookupException, tf.ConnectivityException,
            #             tf.ExtrapolationException):
            #         print("get tf error!")
            #     # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
            #     angle_base = math.asin(self.rot[2])/math.pi*180*2
            #     # print(str(angle_base)+"#####"+ str(math.fabs(90-math.fabs(angle_base)))+"#")
            #     if math.fabs(0-math.fabs(angle_base)) <= 4:
            #         break
            #     else:
            #         mycmd = Twist()
            #         mycmd.linear.x = 0
            #         mycmd.angular.z = -0.4
            #         self.vel_pub.publish(mycmd)
            #         time.sleep(0.001)
            # # 出电梯，这时候需要考虑切换地图的问题，就随便选个x停止吧；其实也不知道是不是真的到站了
            # while True:
            #     if self.num < 30:
            #         time.sleep(0.01)
            #         if self.num < 30:
            #             while True:
            #                 try:
            #                     # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                     #                          rospy.Duration(4.0))
            #                     self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                                              rospy.Duration(4.0))
            #                     # (self.trans,
            #                     #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                     #                                      rospy.Time(0))
            #                     (self.trans,
            #                      self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                                                          rospy.Time(0))
            #                 except (tf.LookupException, tf.ConnectivityException,
            #                         tf.ExtrapolationException):
            #                     print("get tf error!")
            #                 # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
            #                 x = self.trans[0]
            #                 mycmd = Twist()
            #                 mycmd.linear.x = -0.6
            #                 mycmd.angular.z = 0
            #                 self.vel_pub.publish(mycmd)
            #                 # print(self.num)
            #                 time.sleep(0.001)
            #                 if math.fabs(0.45-x) <= 0.02:
            #                     break
            #             break
        return


def main():
    rospy.init_node('enter_elevator')
    EE = EnterElevator()
    rospy.spin()


if __name__ == '__main__':
    main()
