import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math
import numpy as np
import tf
"""
Dynamic window approach implementation with python
Reference: The Dynamic Window Approach to Collision Avoidance
https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
"""


class DWAPlanner():
    def __init__(self):
        # robot parameter
        # self.max_speed = 0.8  # [m/s]
        # self.max_rate = 1.2
        # self.max_as = 0.4
        # self.max_aw = 0.7
        # self.step = 0.01  # step for sampling

        # self.avoid_dist = 0.4

        # self.max_speed = 0.75  # [m/s]
        # self.max_rate = 0.75
        # self.max_as = 0.4
        # self.max_aw = 0.6
        # self.step = 0.006  # step for sampling

        # fianl config
        self.max_speed = 0.4  # [m/s]
        self.max_rate = 0.4
        self.max_as = 0.4
        self.max_aw = 0.4
        # self.step = 0.006  # step for sampling
        self.step = 0.008  # step for sampling



        self.avoid_dist=0.3


        # self.predict_time = 1.5  # [s]
        self.predict_time = 0.8  # [s]

        self.dt = 0.1  # step for sample in trajectory
        self.a_dt = 0.1  # size of dynamic window

        self.tf = tf.TransformListener()

        self.path_pub = rospy.Publisher('/course_agv/predict_path',
                                        Path,
                                        queue_size=1)

    # !!!!!ob is in robot frame
    def plan(self, x, goal, ob):
        # x: [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
        # search space : generate allowable v & yaw
        # maximize object function: find the best v & yaw
        # print(x)
        best_u = [0.0, 0.0]
        # max_value = -99999
        dw = self.dynamic_window(x)
        # best_traj = []

        # value for all traj
        all_value = []
        for v in np.arange(dw[0], dw[1], self.step):
            for w in np.arange(dw[2], dw[3], self.step):
                # print(v, w)
                trajectory = self.predict_trajectory(x, v, w)
                # check collision first
                # if not self.check_collision(trajectory, ob):
                value = [
                    v, w,
                    self.heading_obj_func(trajectory, x, goal),
                    self.clearance_obj_func(trajectory, ob),
                    self.velocity_obj_func(v), 0
                ]
                all_value.append(value)
                # value = 1 * self.heading_obj_func(
                #     trajectory, x, goal) + 0.5 * self.clearance_obj_func(
                #         trajectory, ob) + 0.5 * self.velocity_obj_func(v)
                # # print(value)
                # if value > max_value:
                #     best_u = [v, w]
                #     max_value = value
                #     best_traj = trajectory
        # if len(all_value) == 0:
        #     return [x[3], x[4]]
        V = np.array(all_value)
        # print(V[:, 2])
        sum_heading = sum(V[:, 2])
        sum_clearance = sum(V[:, 3])
        sum_velocity = sum(V[:, 4])

        for item in all_value:
            item[2] = item[2] / sum_heading
            item[3] = item[3] / sum_clearance
            item[4] = item[4] / sum_velocity
            item[5] = 1 * item[2] + 0.5 * item[3] + 0 * item[4]

        V = np.array(all_value)
        max_index = np.argmax(V[:, 5])
        best_u = [V[max_index][0], V[max_index][1]]
        # self.publishPath(best_traj)
        return best_u

    def dynamic_window(self, x):
        Vs = [-self.max_speed, self.max_speed, -self.max_rate, self.max_rate]
        Vd = [
            x[3] - self.max_as * self.a_dt, x[3] + self.max_as * self.a_dt,
            x[4] - self.max_aw * self.a_dt, x[4] + self.max_aw * self.a_dt
        ]
        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3])
        ]
        # print(dw)
        return dw

    def predict_trajectory(self, x_init, v, w):
        """
        Predict trajectory: return trajectory in predict time
        """
        # print(x_init)
        total_time = 0
        pos = np.array(x_init)
        trajectory = np.array(pos)
        while total_time <= self.predict_time:
            total_time += self.dt
            x = self.motion_model(x_init, v, w, total_time)
            trajectory = np.vstack((trajectory, x))
        # print("predict_traj")
        # print(trajectory)
        # self.publishPath(trajectory)
        return trajectory

    def check_collision(self, trajectory, ob):
        """
        Check Collision: return true if collision happens
        """
        # print("check_coll")
        v = trajectory[-1][3]
        w = trajectory[-1][4]
        min_dist = v * v / 2.0 / self.max_as  # shoulid check w too
        # print("the min_dist is " + str(min_dist))
        for traj in trajectory:
            for pos_ob in ob:
                if math.hypot(traj[0] - pos_ob[0],
                              traj[1] - pos_ob[1]) < self.avoid_dist:
                    # if math.hypot(traj[0] - pos_ob[0],
                    #               traj[1] - pos_ob[1]) < min_dist:
                    return True
        return False

    def heading_obj_func(self, trajectory, x, goal):
        """
        Target heading: heading is a measure of progress towards the goal location.
        It is maximal if the robot moves directly towards the target.
        """

        # # the yaw after the trajectory
        # yaw = trajectory[-1][2]
        # # print("the start after the trajectory " + str(trajectory[0][0]) + " " +
        #     #   str(trajectory[0][1]))

        # dx = goal[0] - trajectory[-1][0]
        # dy = goal[1] - trajectory[-1][1]

        # # the yaw towards the goal
        # # rad
        # yaw_goal = math.atan2(dy, dx)

        # return abs(math.pi - (yaw - yaw_goal))

        min_length = 1000
        for traj in trajectory:
            length = math.hypot(traj[0] - goal[0], traj[1] - goal[1])
            if length < min_length:
                min_length = length
        # print("heading_obj")
        # print(length)
        return 1.0 / min_length

    def clearance_obj_func(self, trajectory, ob):
        """
        Clearance: dist is the distance to the closest obstacle on the trajectory.
        The smaller the distance to an obstacle the higher is the robot's desire to move around it.
        """
        if self.check_collision(trajectory, ob):
            dist = -100
        else:
            dist = 100
            for traj in trajectory:
                for pos_ob in ob:
                    distance = math.hypot(traj[0] - pos_ob[0],
                                          traj[1] - pos_ob[1])
                    if distance < dist:
                        dist = distance
            # if dist > 10:
            #     dist = 10
        # print("clearance_obj")
        return dist

    def velocity_obj_func(self, v):
        """
        Velocity: vel is the forward velocity of the robot and supports fast movements.
        """
        velocity = abs(v)
        # print("velocity_obj")
        return velocity

    def motion_model(self, x, v, w, dt):
        # x: [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
        if w == 0:
            w = 0.0001

        r = v / w
        y = [0, 0, 0, 0, 0]
        y[0] = x[0] - r * math.sin(x[2]) + r * math.sin(x[2] + w * dt)
        y[1] = x[1] + r * math.cos(x[2]) - r * math.cos(x[2] + w * dt)
        y[2] = x[2] + w * dt
        y[3] = v
        y[4] = w
        return y

    def publishPath(self, traj):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(traj)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = traj[i][0]
            pose.pose.position.y = traj[i][1]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)
        # print('publish traj!')
        # rospy.sleep(1)
