import pygame
from math import sin, cos
import numpy as np
from scipy.spatial import distance
from Voronoi import Voronoi_Diagram
import matplotlib.pyplot as plt

MAX_theta = np.deg2rad(20)
MIN_theta = -np.deg2rad(20)
thetaNum = 31 # 직선 경로도 선택할 수 있도록 홀수로 설정
MACARON_TREAD = 2
R = 1
Gp = 1
Hp = 1
Vp = 1

pygame.init()

done = False
clock = pygame.time.Clock()

WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255,0,0)
BLUE = (0,0,255)
size = [1000,1000]
SCREEN = pygame.display.set_mode(size)

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
        node.vcost = min(v_dis)
        return node.vcost
    
    def calc_fcost(self, node):
        node.fcost = Gp * self.calc_gcost(node) + Hp * self.calc_hcost(node) + Vp * self.calc_vcost(node)
        return node.fcost
    
    def GetNewNodes(self):
        for i in np.linspace(MIN_theta,MAX_theta,thetaNum):
            dtheta = self.curNode.heading + np.deg2rad(i)
            
            xd = self.curNode.x + R*sin(dtheta)
            yd = self.curNode.y + R*cos(dtheta)
            pygame.draw.line(SCREEN, BLUE, [self.curNode.x,self.curNode.y], [xd,yd], 1)
            
            self.open_list.append(Node([xd,yd],dtheta,self.curNode))
            self.curNode.cnode.append(Node([xd,yd],dtheta,self.curNode))
            
    def is_collision(self, node, obs_xy):
        dis = [((node.x-p[0])**2+(node.y-p[1])**2)**0.5 for p in obs_xy]
        return (np.array(dis) <= (MACARON_TREAD/2)).any()
    
    def generate_path(self, xi, yi, heading, obs_xy, xf, yf, cur_map_ind, fin_map_ind):
        
        self.cur_map_update(cur_map_ind, fin_map_ind)
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
            
            self.open_list.sort(key = lambda x: x.fcost)

            self.curNode = self.open_list[0]
            self.close_list.append(self.open_list[0])
            self.open_list.pop(0)

            if self.curNode.pnode:
                pygame.draw.line(SCREEN, RED, [self.curNode.x,self.curNode.y], [self.curNode.pnode.x,self.curNode.y], 1)
            
            if distance.euclidean([self.curNode.x,self.curNode.y],self.xy_goal) < self.margin:
                break
            
            self.GetNewNodes()
            
            for n_node in self.open_list:
                collision = self.is_collision(n_node, obs_xy)
                if collision:
                    self.open_list.remove(n_node)
                    continue
                
                self.calc_fcost(n_node)
            
            print(len(self.close_list))
            pygame.display.flip()
                
        return self.extracted_path()

    def extracted_path(self):
        path = [[self.curNode.x,self.curNode.y]]
        self.curNode = self.curNode.pnode
        
        while True:
            path.append([self.curNode.x,self.curNode.y])
            if self.curNode.x == self.xy_start[0] and self.curNode.y == self.xy_start[1]:
                break
            
            self.curNode = self.curNode.pnode
        print(path)
            
        return path

def main():
    clock.tick(10)
    SCREEN.fill(WHITE)
    global_path = [[20,i] for i in range(100)]
    margin = 1
    line_left = [[15,i] for i in range(100)]
    line_right = [[25,i] for i in range(100)]
    obs_xy = [[16,20],[17,20],[18,20],[19,20],[20,80],[16,80],[17,80],[18,80],[19,80],[20,80],[24,50],[23,50],[22,50],[21,50],[20,50]]
    astar = Astar(global_path, margin, left_line = line_left, right_line = line_right)
    selected_path = astar.generate_path(xi=20, yi=0, heading=0, obs_xy=obs_xy, xf=20, yf=100, cur_map_ind=0, fin_map_ind=100)
    
    obs = []
    obs.extend(line_left)
    obs.extend(line_right)
    obs.extend(obs_xy)
    obs = np.array(obs)
    plt.plot(obs[:,0],obs[:,1], 'bo')
    selected_path = np.array(selected_path)
    plt.plot(selected_path[:,0],selected_path[:,1])
    plt.show()
    
if __name__ == '__main__':
    main()
