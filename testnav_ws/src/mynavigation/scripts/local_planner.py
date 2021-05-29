#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

# from dwa import *
# from dwa import DWAPlanner
from dwa2 import DWAPlanner

from threading import Lock, Thread
import time


def limitVal(minV, maxV, v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v


def mysubtract(x, y):
    if x-y < -180:
        return x-y+360
    if x-y > 180:
        return x-y-360
    return x-y


class LocalPlanner:
    def __init__(self):
        # self.arrive = 0.3  # standard for arrival self.arrive = 0.1
        self.arrive = 0.2
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.dwa = DWAPlanner()
        # self.max_speed = 0.8  # [m/s]
        # self.predict_time = 2  # [s]
        # self.threshold = self.max_speed * self.predict_time
        # 这个变量不知道干啥的
        self.threshold = 1.5

        self.laser_lock = Lock()
        self.lock = Lock()
        self.path = Path()
        self.tf = tf.TransformListener()
        # get path & initPlaning
        self.path_sub = rospy.Subscriber('/course_agv/global_path', Path,
                                         self.pathCallback)

        # self.vel_pub = rospy.Publisher('/webService/cmd_vel',
        #                                Twist,
        #                                queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist,
                                       queue_size=1)

        # mid_goal pub
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',
                                           PoseStamped,
                                           queue_size=1)

        # get laser & update obstacle
        # self.laser_sub = rospy.Subscriber('/scan_emma_nav_front', LaserScan,
        #                                   self.laserCallback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,
                                          self.laserCallback)
        self.planner_thread = None
        pass

    # update pose & update goal
    # self.plan_goal (in the robot frame)
    # self.goal_dis  (distance from the final goal)
    def updateGlobalPose(self):
        try:
            # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
            #                          rospy.Duration(4.0))
            self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
                                     rospy.Duration(4.0))
            # (self.trans,
            #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
            #                                      rospy.Time(0))
            (self.trans,
             self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
                                                 rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw
        # get nearest path node
        ind = self.goal_index
        self.goal_index = len(self.path.poses) - 1
        while ind < len(self.path.poses):
            p = self.path.poses[ind].pose.position
            dis = math.hypot(p.x - self.x, p.y - self.y)
            # print('mdgb;; ',len(self.path.poses),ind,dis)
            if dis < self.threshold:
                self.goal_index = ind
            ind += 1
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        # lgoal = self.tf.transformPose("/base_footprint", goal)
        lgoal = self.tf.transformPose("/base_footprint", goal)
        self.plan_goal = np.array(
            [lgoal.pose.position.x, lgoal.pose.position.y])
        self.goal_dis = math.hypot(
            self.x - self.path.poses[-1].pose.position.x,
            self.y - self.path.poses[-1].pose.position.y)
        # print(self.goal_dis)

    # get obstacle (in robot frame)
    def laserCallback(self, msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        # print("i am here!")
        self.ob = [[100, 100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment * i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a) * r, math.sin(a) * r])
        self.laser_lock.release()

    # update ob
    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob)
        self.laser_lock.release()

    # get path & initPlaning
    def pathCallback(self, msg):
        # print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initPlanning()
        self.lock.release()
        # if self.planner_thread == None:
        #     self.planner_thread = Thread(target=self.planThreadFunc)
        #     self.planner_thread.start()
        # pass
        self.planThreadFunc()

    def initPlanning(self):
        self.goal_index = 0
        self.vx = 0.0
        self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        # self.goal = np.array([cx[0], cy[0]])
        self.plan_cx, self.plan_cy = np.array(cx), np.array(cy)
        self.plan_goal = np.array([cx[-1], cy[-1]])
        self.plan_x = np.array([0.0, 0.0, 0.0, self.vx, self.vw])

    def planThreadFunc(self):
        print("running planning thread!!")
        while True:
            self.lock.acquire()
            self.planOnce()
            self.lock.release()
            if self.goal_dis < self.arrive:
                angle_goal = math.asin(
                    self.path.poses[-1].pose.orientation.z)/math.pi*180*2
                while True:
                    try:
                        # self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
                        #                          rospy.Duration(4.0))
                        self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
                                                 rospy.Duration(4.0))
                        # (self.trans,
                        #  self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
                        #                                      rospy.Time(0))
                        (self.trans,
                         self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
                                                             rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException,
                            tf.ExtrapolationException):
                        print("get tf error!")
                    # angle_dis = (math.asin(self.rot[2]) - angle_goal)*2*180/math.pi
                    angle_base = math.asin(self.rot[2])/math.pi*180*2
                    angle_dis = angle_base - angle_goal
                    print(angle_base, angle_goal)
                    # if math.fabs(angle_dis) < 5 or math.fabs(angle_dis) > 355:
                    if math.fabs(angle_base - 164) < 1.5:
                        break
                    else:
                        self.lock.acquire()
                        mycmd = Twist()
                        mycmd.linear.x = 0
                        mycmd.angular.z = 0.4
                        self.vel_pub.publish(mycmd)
                        self.lock.release()
                        time.sleep(0.001)
                print("arrive goal!")
                rospy.set_param("mystate", 1)
                break
            time.sleep(0.001)
        print("exit planning thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        # self.planning_thread = None
        pass

    def planOnce(self):
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [0.0, 0.0, 0.0, self.vx, self.vw]
        # Update obstacle
        self.updateObstacle()
        u = self.dwa.plan(self.plan_x, self.plan_goal, self.plan_ob)
        alpha = 0.5
        # self.vx = u[0] * alpha + self.vx * (1 - alpha)
        # self.vw = u[1] * alpha + self.vw * (1 - alpha)
        self.vx = u[0]
        self.vw = u[1]
        print("v, w: ", self.vx, self.vw)
        # self.vx, self.vw = 0, 0
        # print("mdbg; ",u)
        self.publishVel(zero=False)
        pass

    # send v,w
    def publishVel(self, zero=False):
        if zero:
            self.vx = 0
            self.vw = 0
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)


def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
