import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import random
import time

class Voronoi_Diagram:
    def __init__(self, global_path = None):
        self.global_path = global_path
        self.voronoi_lines = Voronoi(self.global_path)
        #plt.plot(self.global_path)
        #plt.show()
        
    def ccw(self,x1,x2,x3,y1,y2,y3):
        cross_product = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)
        if cross_product > 0:
            return 1
        elif cross_product < 0:
            return -1
        else:
            return 0
        
    def show(self):
        fig = voronoi_plot_2d(self.voronoi_lines, show_vertices=True, line_colors='orange',
                      line_width=1, line_alpha=0.6, point_size=1)
        plt.show()
    
#VD = Voronoi_Diagram(np.load(file = "bonseon.npy"))
#VD = Voronoi_Diagram(np.load(file = "yaeseon_xy.npy"))
#VD.show()

#randpoint = np.array(random.choices(range(1,5001),k=100))
#point = randpoint.reshape(-1,2)
#print(point)
#start = time.time()
#vor = Voronoi(point)
#print(time.time()-start)
#print(vor)
#print(vor.vertices)
#print(vor.ridge_vertices)
#fig = voronoi_plot_2d(vor)
#plt.show()