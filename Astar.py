from math import sin, cos
import numpy as np
from scipy.spatial import distance

MAX_theta = 20
MIN_theta = -20
thetaNum = 15 # 직선 경로도 선택할 수 있도록 홀수로 설정
R = 0.5
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
    def __init__(self, xi, yi, heading, obs_xy, global_path, xf, yf, margin):
        self.global_path = global_path
        
        self.open_list = []
        self.close_list = []
        
        self.xy_start = [xi, yi]
        self.xy_goal = [xf, yf]
        self.heading = heading
        
        self.curNode = Node(self.xy_start, self.heading)
        self.open_list.append(self.curNode)
        
        self.Voronoi_Lines = self.VD()
        
        self.obs_xy = obs_xy
        self.margin = margin
        
    def calc_gcost(self, node):
        if node.pnode:
            node.gcost = node.pnode.gcost-1
        else:
            node.gcost = 0
        return node.gcost
    
    def calc_hcost(self, node):
        node.hcost = distance.euclidean(node,self.xy_goal)
        return node.hcost
    
    def calc_vcost(self, node):
        return node.vcost
    
    def calc_fcost(self, node):
        node.fcost = Gp * self.calc_gcost(node) + Hp * self.calc_hcost(node) + Vp * self.calc_vcost(node)
        return self.fcost
    
    def VD(self):
        pass
    
    def GetNewNodes(self):
        for i in np.linspace(MAX_theta,MIN_theta,thetaNum):
            
            dtheta = self.curNode.heading + i
            
            xd = self.curNode.x + R*sin(dtheta)
            yd = self.curNode.y + R*cos(dtheta)
            
            self.open_list.append(Node(xd,yd,dtheta,self.curNode))
            self.curNode.cnode.append(Node(xd,yd,dtheta,self.curNode))
            
    def is_collision(self,node):
        pass
    
    def Astar(self, xi, yi, heading, obs_xy, xf, yf):
        
        self.open_list = []
        self.close_list = []
        
        self.xy_start = [xi, yi]
        self.xy_goal = [xf, yf]
        self.heading = heading
        
        self.curNode = Node(self.xy_start, self.heading)
        self.open_list.append(self.curNode)
        
        self.obs_xy = obs_xy
        
        self.Voronoi_Lines = self.VD()
        
        while self.open_list:
            
            self.open_list.sort(key = lambda x: x.fcost)

            self.curNode = self.open_list[0]
            self.close_list.append(self.open_list[0])
            self.open_list.pop(0)
            
            if distance.euclidean(self.curNode,self.xy_goal) < self.margin:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                collision = self.is_collision(n_node)
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
            if self.curNode.x == self.xy_goal[0] and self.curNode.y == self.xy_goal[1]:
                break
            
            self.curNode = self.curNode.pnode
            
        return path