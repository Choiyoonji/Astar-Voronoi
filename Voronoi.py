import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
# import random
# import time

class Voronoi_Diagram:
    def __init__(self, global_path = None):
        self.global_path = global_path
        self.voronoi_lines = Voronoi(self.global_path)
        
        delnum = 0
        for i in range(0,len(self.voronoi_lines.ridge_points)):
            if 1 >= self.voronoi_lines.ridge_points[i][0]-self.voronoi_lines.ridge_points[i][1] >= -1:
                self.voronoi_lines.ridge_vertices.pop(i-delnum)
                delnum += 1
        
        #plt.plot(self.global_path)
        #plt.show()
        
    def show(self):
        fig = voronoi_plot_2d(self.voronoi_lines, show_vertices=True, line_colors='orange',
                      line_width=1, line_alpha=0.6, point_size=1)
        plt.show()
 
def main():
    pass
    # VD = Voronoi_Diagram(np.load(file = "bonseon.npy"))
    # VD = Voronoi_Diagram(np.load(file = "yaeseon_xy.npy"))
    # VD.show()

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