import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d, distance
import matplotlib.pyplot as plt
from collections import deque
# import random
import time

MACARON_TREAD = 1.5

class Voronoi_Diagram:
    def __init__(self, global_path = None, line_left = None, line_right = None, obs_xy = None):
        self.x1 = 955802.4345550193
        self.y = 1951238.3999692136
        self.obs = []
        self.obs.extend(line_left)
        self.obs.extend(line_right)
        self.obs = np.array(self.obs)
        self.voronoi = Voronoi(self.obs)
        self.checked_node = {}
        
        
        # 필요없는 선들 제거
        del_num = 0
        for i in range(len(self.voronoi.ridge_points)):
            if -1 <= self.voronoi.ridge_points[i][0] - self.voronoi.ridge_points[i][1] <= 1:
                self.voronoi.ridge_vertices.pop(i-del_num)
                del_num += 1
            elif self.voronoi.ridge_vertices[i-del_num][0] == -1 or self.voronoi.ridge_points[i-del_num][1] == -1:
                self.voronoi.ridge_vertices.pop(i-del_num)
                del_num += 1
        
        print(self.voronoi.ridge_vertices)
        
        # self.voronoi.ridge_vertices.sort(key = lambda x: (x[0],x[1]))
        
        # self.x = 50
        # self.y = 530        
        # self.global_path = global_path
        # self.obs = global_path
        # self.voronoi = Voronoi(self.global_path)
        
        # delnum = 0
        # for i in range(0,len(self.voronoi.ridge_points)):
        #     if -1 <= self.voronoi.ridge_points[i][0] - self.voronoi.ridge_points[i][1] <= 1:
        #         self.voronoi.ridge_vertices.pop(i-delnum)
        #         delnum += 1
        #     elif self.voronoi.ridge_vertices[i-delnum][0] == -1 or self.voronoi.ridge_vertices[i-delnum][1] == -1:
        #         self.voronoi.ridge_vertices.pop(i-delnum)
        #         delnum += 1
                
        # self.voronoi.ridge_vertices.sort(key = lambda x: (x[0],x[1]))
        
        
    def find_start(self):
        dis = [distance.euclidean([self.x1,self.y],p) for p in self.voronoi.vertices]
        min_ind = dis.index(min(dis))

        return min_ind
    
    def find_ridges(self, node_ind):
        self.checked_node[node_ind] = True
        ridges = []
        for ridge in self.voronoi.ridge_vertices:
            if ridge[0] == node_ind:
                ridges.append(ridge[1])
                self.checked_node[ridge[1]] = True
            elif ridge[1] == node_ind and not ridge[0] in self.checked_node.keys():
                ridges.append(ridge[0])
                self.checked_node[ridge[0]] = True
        
        return ridges
    
    def is_collision(self, cur_point):
        dis = [((self.voronoi.vertices[cur_point][0]-p[0])**2+(self.voronoi.vertices[cur_point][1]-p[1])**2)**0.5 for p in self.obs]
        return (np.array(dis) <= (MACARON_TREAD/2)).any()
    
    def get_path(self):
        self.voronoi.ridge_points.sort()
        start_point = self.find_start()
        root_point = start_point
        deq = deque([(root_point,start_point)])
        candidate_nodes = {}
        while True:
            if len(deq) <= 0:
                break
            [root_point, cur_point] = deq.popleft()
            collision = self.is_collision(cur_point)
            if collision:
                candidate_nodes[root_point] = []
                continue
            ridges = self.find_ridges(cur_point)
            
            if len(ridges) == 1:
                deq.append((root_point,ridges[0]))
            elif len(ridges) > 1:
                for r in ridges:
                    deq.append((r,r))
            if root_point in candidate_nodes:
                candidate_nodes[root_point].append(cur_point)
            else:
                candidate_nodes[root_point] = [cur_point]
                
        selected_points = []
        for root in candidate_nodes.keys():
            selected_points.extend(candidate_nodes[root])
        
        return selected_points
            
    def last_points(self):
        selected_points = self.get_path()
        points = []
        for p in selected_points:
            points.append(self.voronoi.vertices[p])
            
        last_points = np.array(points)
        return last_points
        
    def show(self):
        fig = voronoi_plot_2d(self.voronoi, show_vertices=True, line_colors='orange',
                      line_width=1, line_alpha=0.6, point_size=1)
        plt.show()
                
    def selected_show(self):
        points = self.last_points()
        plt.plot(points[:,0],points[:,1], 'ro', self.obs[:,0],self.obs[:,1], 'bo')
        plt.show()


def main():
    VD = Voronoi_Diagram(line_left = np.load(file = "wonline10203.npy"), line_right = np.load(file = "wonline20203.npy"))
    VD.selected_show()

    # randpoint = np.array(random.choices(range(1,5001),k=100))
    # point = randpoint.reshape(-1,2)
    # print(point)
    # start = time.time()
    # vor = Voronoi(point)
    # print(time.time()-start)
    # print(vor)
    # print(vor.vertices)
    # print(vor.ridge_vertices)
    # fig = voronoi_plot_2d(vor)
    # plt.show()
    
if __name__ == '__main__':
    main()
