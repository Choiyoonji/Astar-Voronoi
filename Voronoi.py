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
    point = np.array([(4, 478) ,
    (18, 469) ,
    (29, 457) ,
    (40, 445) ,
    (49, 434) ,
    (54, 426) ,
    (62, 411) ,
    (72, 398) ,
    (81, 384) ,
    (87, 377) ,
    (96, 364) ,
    (103, 354) ,
    (109, 348) ,
    (114, 341) ,
    (122, 335) ,
    (128, 325) ,
    (138, 317) ,
    (145, 309) ,
    (150, 302) ,
    (154, 295) ,
    (162, 289) ,
    (169, 283) ,
    (174, 278) ,
    (178, 274) ,
    (186, 266) ,
    (196, 257) ,
    (204, 249) ,
    (210, 243) ,
    (216, 238) ,
    (221, 226) ,
    (231, 220) ,
    (241, 210) ,
    (250, 202) ,
    (258, 190) ,
    (269, 181) ,
    (275, 172) ,
    (285, 164) ,
    (292, 157) ,
    (298, 149) ,
    (306, 140) ,
    (314, 126) ,
    (322, 118) ,
    (332, 109) ,
    (350, 95) ,
    (359, 86) ,
    (366, 82) ,
    (371, 76) ,
    (377, 68) ,
    (387, 59) ,
    (398, 49) ,
    (406, 41) ,
    (414, 31) ,
    (426, 22) ,
    (435, 11) ,
    (82, 594) ,
    (92, 586) ,
    (99, 580) ,
    (106, 572) ,
    (115, 562) ,
    (122, 553) ,
    (130, 542) ,
    (141, 529) ,
    (147, 518) ,
    (158, 507) ,
    (166, 494) ,
    (178, 484) ,
    (186, 475) ,
    (196, 463) ,
    (205, 450) ,
    (216, 434) ,
    (230, 419) ,
    (238, 408) ,
    (247, 399) ,
    (255, 390) ,
    (261, 384) ,
    (272, 377) ,
    (280, 366) ,
    (291, 353) ,
    (301, 336) ,
    (311, 322) ,
    (319, 310) ,
    (330, 294) ,
    (338, 286) ,
    (346, 274) ,
    (353, 263) ,
    (363, 254) ,
    (375, 239) ,
    (379, 232) ,
    (386, 225) ,
    (394, 217) ,
    (402, 206) ,
    (408, 194) ,
    (415, 180) ,
    (426, 167) ,
    (434, 155) ,
    (442, 145) ,
    (451, 137) ,
    (462, 126) ,
    (474, 118) ,
    (480, 110) ,
    (486, 104) ,
    (490, 94) ,
    (498, 85) ,
    (504, 73) ,
    (512, 58) ,
    (518, 50) ,
    (526, 40) ,
    (532, 31) ,
    (539, 22) ,
    (545, 12) ,
    (442, 31) ,
    (450, 34) ,
    (458, 38) ,
    (465, 40) ,
    (472, 46) ,
    (483, 52) ,
    (415, 70) ,
    (422, 74) ,
    (429, 78) ,
    (438, 83) ,
    (445, 89) ,
    (449, 91) ,
    (453, 94) ,
    (462, 102) ,
    (470, 108) ,
    (335, 110) ,
    (338, 114) ,
    (353, 118) ,
    (367, 126) ,
    (381, 132) ,
    (334, 156) ,
    (347, 164) ,
    (363, 174) ,
    (377, 181) ,
    (384, 187) ,
    (394, 192) ,
    (262, 199) ,
    (272, 205) ,
    (282, 209) ,
    (296, 219) ,
    (307, 226) ,
    (320, 230) ,
    (329, 236) ,
    (226, 264) ,
    (230, 269) ,
    (238, 272) ,
    (250, 282) ,
    (265, 293) ,
    (278, 303) ,
    (290, 310) ,
    (297, 318) ,
    (166, 306) ,
    (175, 314) ,
    (179, 317) ,
    (193, 324) ,
    (206, 334) ,
    (221, 341) ,
    (234, 351)])
    VD = Voronoi_Diagram(point)
    VD.show()
    # pass
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