from math import sin, cos
import numpy as np
import heapq

MAX_theta = 20
MIN_theta = -20
thetaNum = 15
R = 5

class Node:
    def __init__(self, x, y, heading, pnode = None):
        self.pnode = pnode
        self.cnode = []
        self.x = x
        self.y = y
        self.heading = heading
        self.gcost = 0
        self.hcost = 0
        self.vcost = 0
        self.fcost = 0


class Astar:
    def __init__(self, xi, yi, heading, obs_xy, global_path, xf, yf):
        self.global_path = global_path
        self.open_list = []
        self.close_list = []
        self.xi = xi
        self.yi = yi
        self.xf = xf
        self.yf = yf
        self.heading = heading
        self.curNode = Node(self.xi,self.yi,self.heading)
        self.open_list.append(self.curNode)
        self.Voronoi_Lines = []
        self.obs_xy = obs_xy
    
        
    def calc_gcost(self, node):
        if node.pnode == None:
            node.gcost = 0
        else:
            node.gcost = node.pnode.gcost-1
        return node.gcost
    
    def calc_hcost(self, node):
        return node.hcost
    
    def calc_vcost(self, node):
        return node.vcost
    
    def calc_fcost(self, node):
        
        node.fcost = self.calc_gcost(node) + self.calc_hcost(node) + self.calc_vcost(node)
        
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
    
    def Astar(self):
        if len(self.open_list) > 0: 
            self.open_list.sort(key = lambda x: x.fcost)
        
        self.close_list.append(self.open_list[0])
        self.open_list.pop(0)
            
        self.GetNewNodes(self.curNode)