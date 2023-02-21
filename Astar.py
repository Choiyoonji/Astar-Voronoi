from math import sin, cos
import numpy as np
from scipy.spatial import distance
import Voronoi

MAX_theta = 20
MIN_theta = -20
thetaNum = 15 # 직선 경로도 선택할 수 있도록 홀수로 설정
MACARON_TREAD = 1.5
R = 0.3
Gp = 1
Hp = 1
Vp = 1

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
        
        self.curNode = Node(self.xy_start, self.heading)
        self.open_list.append(self.curNode)
        
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
        node.hcost = distance.euclidean(node,self.xy_goal)
        return node.hcost
    
    def calc_vcost(self, node, v_point):
        v_dis = [((node-p[0])**2+(node-p[1])**2)**0.5 for p in v_point]
        node.vcost = min(v_dis)
        return node.vcost
    
    def calc_fcost(self, node):
        node.fcost = Gp * self.calc_gcost(node) + Hp * self.calc_hcost(node) + Vp * self.calc_vcost(node)
        return self.fcost
    
    def GetNewNodes(self):
        for i in np.linspace(MAX_theta,MIN_theta,thetaNum):
            dtheta = self.curNode.heading + i
            
            xd = self.curNode.x + R*sin(dtheta)
            yd = self.curNode.y + R*cos(dtheta)
            
            self.open_list.append(Node(xd,yd,dtheta,self.curNode))
            self.curNode.cnode.append(Node(xd,yd,dtheta,self.curNode))
            
    def is_collision(self, node, obs_xy):
        dis = [((node-p[0])**2+(node-p[1])**2)**0.5 for p in obs_xy]
        return (np.array(dis) <= (MACARON_TREAD/2)).any()
    
    def Astar(self, xi, yi, heading, obs_xy, xf, yf, cur_map_ind, fin_map_ind):
        
        self.cur_map_update(cur_map_ind, fin_map_ind)
        self.VD = Voronoi(xi, yi, self.cur_left_line, self.cur_right_line, obs_xy)

        self.open_list = []
        self.close_list = []
        
        self.xy_start = [xi, yi]
        self.xy_goal = [xf, yf]
        self.heading = heading
        
        self.curNode = Node(self.xy_start, self.heading)
        self.open_list.append(self.curNode)
        
        while self.open_list:
            
            self.open_list.sort(key = lambda x: x.fcost)

            self.curNode = self.open_list[0]
            self.close_list.append(self.open_list[0])
            self.open_list.pop(0)
            
            if distance.euclidean(self.curNode,self.xy_goal) < self.margin:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                collision = self.is_collision(n_node, obs_xy)
                if collision:
                    self.open_list.remove(n_node)
                    continue
                
                self.calc_fcost(n_node)
                
        return self.extracted_path()

    def extracted_path(self):
        path = [[self.curNode.x,self.curNode.y]]
        self.curNode = self.curNode.pnode
        
        while True:
            path.append([self.curNode.x,self.curNode.y])
            if self.curNode.x == self.xy_start[0] and self.curNode.y == self.xy_start[1]:
                break
            
            self.curNode = self.curNode.pnode
            
        return path