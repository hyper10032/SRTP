#!/usr/bin/env python
import rospy
import tf
import sys
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# from course_agv_nav.srv import Plan, PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

# from a_star import AStarPlanner
from rrt_star import RRT_Planner


class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        # self.plan_grid_size = 0.3
        # self.plan_robot_radius = 0.3
        self.plan_grid_size = 0.3
        self.plan_robot_radius = 0.2
        self.goalyaw = 0
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        # count to update map
        # self.map_count = 0

        self.tf = tf.TransformListener()

        # if new goal, then replan
        # self.goal_sub = rospy.Subscriber('/course_agv/goal', PoseStamped,
        #                                  self.goalCallback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                         self.goalCallback)
        # # if requested, then replan
        # self.plan_srv = rospy.Service('/course_agv/global_plan', Plan,
        #                               self.replan)
        # pub the path
        self.path_pub = rospy.Publisher('/course_agv/global_path',
                                        Path,
                                        queue_size=1)
        # get the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid,
                                        self.mapCallback)
        # if collision, then replan
        self.collision_sub = rospy.Subscriber('/collision_checker_result',
                                              Bool, self.collisionCallback)

        # self.updateMap()
        self.updateGlobalPose()
        pass

    def goalCallback(self, msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        self.goalyaw = msg.pose.orientation.z
        print("get new goal!!! ", self.plan_goal)
        self.replan(0)
        pass

    def collisionCallback(self, msg):
        self.replan(0)


# get start coordinates (in map frame)

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(),
                                     rospy.Duration(4.0))
            (self.trans,
             self.rot) = self.tf.lookupTransform('/map', '/base_footprint',
                                                 rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]
        # self.plan_sx = 0
        # self.plan_sy = 0
        print("debug start ", self.plan_sx, self.plan_sy)

    def replan(self, req):
        print('get request for replan!!!!!!!!')
        self.init_RRT_Star()
        self.updateGlobalPose()

        s_x, s_y, r_map, self.plan_rx, self.plan_ry = self.rrt.plan(
            self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        self.publishPath()
        res = True
        # return PlanResponse(res)
        return

    def init_RRT_Star(self):
        print("debugmap", self.map.info.width, self.map.info.height)
        map_data = np.array(self.map.data).reshape(
            (self.map.info.height, -1)).transpose()
        ########  100----obstacle   #######
        # ox, oy = np.nonzero(map_data < 0)
        ox, oy = np.nonzero(map_data > 0)
        # print("i am " + str(len(ox)))

        self.plan_ox = (ox * self.map.info.resolution +
                        self.map.info.origin.position.x).tolist()
        self.plan_oy = (oy * self.map.info.resolution +
                        self.map.info.origin.position.y).tolist()
        self.rrt = RRT_Planner(self.plan_ox, self.plan_oy, self.plan_grid_size,
                               self.plan_robot_radius)

    def mapCallback(self, msg):
        self.map = msg
        pass

    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map', GetMap)
            msg = getMap().map
        except (Exception):
            e = sys.exc_info()[0]
            print('Service call failed: %s' % e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def publishPath(self):
        print("markdebug ", self.plan_rx, self.plan_ry)
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            # pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i]
            # pose.pose.position.y = self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.x = self.plan_rx[i]
            pose.pose.position.y = self.plan_ry[i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = self.goalyaw
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)
        print('publish path!')


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass


if __name__ == '__main__':
    main()
