from scipy.spatial import KDTree
import numpy as np
import random
import math


class RRT_Planner(object):
    def __init__(
            self,
            ox,  # in map frame
            oy,
            step_size,     # self.plan_grid_size=0.3
            robot_radius,  # robot_radius=0.3
            avoid_buffer=0.35,  # max is 0.6
            # avoid_buffer=0.6,  # max is 0.6
            iterations=1e4,
            heuristic_dist='Manhattan',
            N_SAMPLE=600,
            KNN=10,
            MAX_EDGE_LEN=1):
        self.ox = ox
        self.oy = oy
        self.N_SAMPLE = N_SAMPLE
        self.KNN = KNN
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.minx = min(ox)  #edge(-10~10)
        self.maxx = max(ox)
        self.miny = min(oy)
        self.maxy = max(oy)
        # print(self.minx, self.miny, self.maxx, self.maxy)

        self.robot_size = robot_radius
        self.avoid_dist = avoid_buffer  # in cllisionfree(),self.robot_size + self.avoid_dist deciding the obstacle
        self.step_size = step_size  #in steer() deciding the length of each steer

    def plan(self, start_x, start_y, goal_x, goal_y):
        obstree = self.obstacle_KDTree(self.ox, self.oy)
        sample_x, sample_y, road_map = self.CreateRoadMap(
            start_x, start_y, goal_x, goal_y, obstree)
        path_x, path_y = self.FindPath(sample_x, sample_y, road_map)
        path_x.reverse()
        path_y.reverse()
        return sample_x, sample_y, road_map, path_x, path_y

    def SampleFree(self, goal_x, goal_y):
        if np.random.rand() < 0.5:
            rand_x = (random.random() * (self.maxx - self.minx)) + self.minx
            rand_y = (random.random() * (self.maxy - self.miny)) + self.miny
        else:
            rand_x = goal_x
            rand_y = goal_y
        return rand_x, rand_y

    def Nearest(self, G_x, G_y, rand_x, rand_y):
        Gtree = KDTree(np.vstack((G_x, G_y)).T)
        distance, index = Gtree.query(np.array([rand_x, rand_y]))
        nearest_x, nearest_y = Gtree.data[index]
        return nearest_x, nearest_y, index

    def Steer(self, nearest_x, nearest_y, rand_x, rand_y):
        dx = rand_x - nearest_x
        dy = rand_y - nearest_y
        length = math.hypot(dx, dy)
        new_dx = dx * self.step_size / length
        new_dy = dy * self.step_size / length
        new_x = nearest_x + new_dx
        new_y = nearest_y + new_dy
        return new_x, new_y

    def Near(self, G_x, G_y, new_x, new_y):
        Gtree = KDTree(np.vstack((G_x, G_y)).T)
        distances, indexs = Gtree.query(np.array([new_x, new_y]), k=10)
        indexs = indexs[:len(G_x)]
        return indexs

    def CreateRoadMap(self, start_x, start_y, goal_x, goal_y, obstree):
        V_x, V_y = [start_x], [start_y]
        E = []
        C = [0]

        while True:
            rand_x, rand_y = self.SampleFree(goal_x, goal_y)
            nearest_x, nearest_y, nearest_index = self.Nearest(G_x=V_x,
                                                               G_y=V_y,
                                                               rand_x=rand_x,
                                                               rand_y=rand_y)
            new_x, new_y = self.Steer(nearest_x=nearest_x,
                                      nearest_y=nearest_y,
                                      rand_x=rand_x,
                                      rand_y=rand_y)
            if self.ObtacleFree(nearest_x, nearest_y, new_x, new_y, obstree):
                # find 10 nearest vertex in V
                near_indexs = self.Near(V_x, V_y, new_x, new_y)
                V_x.append(new_x)
                V_y.append(new_y)

                min_index = nearest_index
                min_cost = C[nearest_index] + math.hypot(
                    new_x - nearest_x, new_y - nearest_y)

                # rewrite parents
                for i in near_indexs:
                    if self.ObtacleFree(
                            new_x, new_y, V_x[i], V_y[i],
                            obstree) and C[i] + math.hypot(
                                new_x - V_x[i], new_y - V_y[i]) < min_cost:
                        min_index = i
                        min_cost = C[i] + math.hypot(new_x - V_x[i],
                                                     new_y - V_y[i])
                E.append([min_index, len(V_x) - 1])
                C.append(min_cost)

                # rewrite edges
                for i in near_indexs:
                    if self.ObtacleFree(
                            new_x, new_y, V_x[i], V_y[i],
                            obstree) and min_cost + math.hypot(
                                new_x - V_x[i], new_y - V_y[i]) < C[i]:
                        for j in range(len(E)):
                            if E[j][1] == i:
                                E[j][0] = len(V_x) - 1
                                C[i] = min_cost + math.hypot(
                                    new_x - V_x[i], new_y - V_y[i])
                                break

                if math.hypot(new_x - goal_x, new_y - goal_y) < self.step_size:
                    E.append([nearest_index, -1])
                    V_x.append(goal_x)
                    V_y.append(goal_y)
                    break
        return V_x, V_y, E

    def FindPath(self, V_x, V_y, E):
        path_x = [V_x[-1]]
        path_y = [V_y[-1]]
        p = -1
        while True:
            for i in range(len(E)):
                if E[i][1] == p:
                    p = E[i][0]
                    path_x.append(V_x[p])
                    path_y.append(V_y[p])
                    break
            if p == 0:
                break
        print(path_x)
        return path_x, path_y

    def obstacle_KDTree(self, ox, oy):
        obstacle_x = [-999999]
        obstacle_y = [-999999]
        for x in ox:
            obstacle_x.append(x)
        for y in oy:
            obstacle_y.append(y)
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        return obstree

    # self.ObtacleFree(nearest_x, nearest_y, new_x, new_y, obstree)
    def ObtacleFree(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:
            return False

        step_size = self.robot_size + self.avoid_dist
        steps = int(dis / step_size)
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return False
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        distance, index = obstree.query(np.array([nx, ny]))
        if distance <= step_size:
            return False

        return True


class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
