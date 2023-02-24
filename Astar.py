from math import sin, cos, pi
import numpy as np
from scipy.spatial import distance
from Voronoi import Voronoi_Diagram
import matplotlib.pyplot as plt
import time


MAX_theta = np.deg2rad(28)
MIN_theta = -np.deg2rad(28)
thetaNum = 9 # 직선 경로도 선택할 수 있도록 홀수로 설정
MACARON_TREAD = 2
R = 0.9
Gp = 5
Hp = 25
Vp = 25
Op = 20

class Node:
    def __init__(self, xy, heading, pnode = None):
        self.pnode = pnode
        self.cnode = []
        
        self.x = xy[0]
        self.y = xy[1]
        self.heading = heading
        
        self.gcost = 0
        self.hcost = 0
        self.vcost = 0
        self.fcost = 0

class Astar:
    def __init__(self, global_path, margin, left_line = None, right_line = None):
        self.global_path = global_path
        self.left_line = left_line
        self.right_line = right_line

        self.cur_global_path = self.global_path[0:]
        self.cur_left_line = self.left_line[0:]
        self.cur_right_line = self.right_line[0:]
        
        self.open_list = []
        self.close_list = []
        
        self.margin = margin

    def cur_map_update(self, cur_map_ind, fin_map_ind):
        self.cur_global_path = self.global_path[cur_map_ind:fin_map_ind]
        self.cur_left_line = self.left_line[cur_map_ind:fin_map_ind]
        self.cur_right_line = self.right_line[cur_map_ind:fin_map_ind]
        
    def calc_gcost(self, node):
        if node.pnode:
            node.gcost = node.pnode.gcost-1
        else:
            node.gcost = 0
        return node.gcost
    
    def calc_hcost(self, node):
        node.hcost = distance.euclidean([node.x,node.y],self.xy_goal)
        return node.hcost
    
    def calc_vcost(self, node):
        v_dis = [((node.x-p[0])**2+(node.y-p[1])**2)**0.5 for p in self.VD]
        min_ind = v_dis.index(min(v_dis))
        minadd = min_ind + 6
        minmin = min_ind - 0
        if minadd >= len(v_dis):
            minadd = len(v_dis) - 1
        if minmin < 0:
            minmin = 0
        for i in range(minmin,minadd):
            node.vcost += v_dis[i]
        return node.vcost
    
    def calc_ocost(self, node):
        obs_dis = [((node.x-p[0])**2+(node.y-p[1])**2)**0.5 for p in self.obs]
        return min(obs_dis)
    
    def calc_fcost(self, node):
        node.fcost = Gp * self.calc_gcost(node) + Hp * self.calc_hcost(node) + Vp * self.calc_vcost(node) + Op * self.calc_ocost(node)
        if self.is_collision(node, self.obs):
            node.fcost = 9999999999999
        return node.fcost
    
    def GetNewNodes(self):
        for i in np.linspace(MIN_theta,MAX_theta,thetaNum):
            dtheta = self.curNode.heading + np.deg2rad(i)

            xd = self.curNode.x + R*cos(dtheta)
            yd = self.curNode.y + R*sin(dtheta)
            
            self.open_list.append(Node([xd,yd],dtheta,self.curNode))
            self.curNode.cnode.append(Node([xd,yd],dtheta,self.curNode))
            
    def is_collision(self, node, obs_xy):
        dis = [((node.x-p[0])**2+(node.y-p[1])**2)**0.5 for p in self.obs]
        return (np.array(dis) <= (MACARON_TREAD/2)).any()
    
    def generate_path(self, xi, yi, heading, obs_xy, xf, yf, cur_map_ind, fin_map_ind):
        
        self.cur_map_update(cur_map_ind, fin_map_ind)
        self.obs = []
        self.line = self.cur_left_line + self.cur_right_line
        self.obs.extend(obs_xy)
        self.obs.extend(self.line)
        self.Voronoi = Voronoi_Diagram(xi, yi, line_left = self.cur_left_line, line_right = self.cur_right_line, obs_xy =obs_xy)
        self.VD = self.Voronoi.last_points()

        self.open_list = []
        self.close_list = []
        
        self.xy_start = [xi, yi]
        self.xy_goal = [xf, yf]
        self.heading = heading
        
        self.curNode = Node(self.xy_start, self.heading)
        self.open_list.append(self.curNode)
        
        while self.open_list:
            
            self.curNode = min(self.open_list, key=lambda n: n.fcost)
            self.close_list.append(self.curNode)
            self.open_list.remove(self.curNode)
            
            if distance.euclidean([self.curNode.x,self.curNode.y],self.xy_goal) < self.margin:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                self.calc_fcost(n_node)
            
            print(len(self.open_list),self.curNode.x,self.curNode.y)
                
        return self.extracted_path()

    def extracted_path(self):
        path = [[self.curNode.x,self.curNode.y]]
        self.curNode = self.curNode.pnode
        
        while True:
            path.append([self.curNode.x,self.curNode.y])
            # print(self.curNode.fcost)
            if self.curNode.x == self.xy_start[0] and self.curNode.y == self.xy_start[1]:
                break
            
            self.curNode = self.curNode.pnode
        # print(path)
            
        return path

def main():
    global_path = [[20,i] for i in range(40)]
    margin = 2
    line_left = [[15,i] for i in range(40)]
    line_right = [[25,i] for i in range(40)]
    # obs_xy = []
    obs_xy = [[16,20],[17,20],[18,20],[19,20]]
    # obs_xy = [[16,10],[17,10],[18,10],[19,10],[24,30],[23,30],[22,30],[21,30],[20,30]]
    # line_left = [[15,i] for i in range(70)]
    # line_right = [[25,i] for i in range(70)]
    # obs_xy = [[16,20],[17,20],[18,20],[19,20],[20,20],[20,50],[21,50],[24,50],[23,50],[22,50]]
    
    astar = Astar(global_path, margin, left_line = line_left, right_line = line_right)
    t = time.time()
    selected_path = astar.generate_path(xi=20, yi=0, heading=pi/2, obs_xy=obs_xy, xf=20, yf=40, cur_map_ind=0, fin_map_ind=40)
    print(time.time()-t)

    obs = []
    obs.extend(line_left)
    obs.extend(line_right)
    obs.extend(obs_xy)
    obs = np.array(obs)
    plt.axis([0, 40, 0, 40])
    plt.plot(obs[:,0],obs[:,1], 'bo')
    selected_path = np.array(selected_path)
    plt.plot(selected_path[:,0],selected_path[:,1],'go')
    plt.show()
    
if __name__ == '__main__':
    main()
