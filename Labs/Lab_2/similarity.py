import numpy as np
from PIL import Image
import sys
import matplotlib.pyplot as plt
from math import cos, sin, sqrt
# import KDTree library for comparison
from sklearn.neighbors import KDTree
# import scan data
# data is currently in the form of alternating distance arrays 
# and intensity arrays
# scan data for point 3
from point_3_datascan_formatted import point_3_data_all
# scan data for point 6
from point_6_datascan_formatted import point_6_data_all

class similarity():
    def __init__(self):
        # filter out intesity data from data set
        # self.point_3_data_filtered = [point_3_data[index] for index in range(0, len(point_3_data), 2)] # 190 arrays
        # self.point_6_data_filtered = [point_6_data[index] for index in range(0, len(point_6_data), 2)] # 132 arrays
        # generate histogram bins for the point 3 data
        self.selection_index = 120 # must be even
        self.point_3_data_filtered = point_3_data_all[self.selection_index]
        self.point_6_data_filtered = point_6_data_all[self.selection_index]
        # self.point_3_np_hist, _ = np.histogram(self.point_3_data_filtered, "auto")
        # self.point_6_np_hist, _ = np.histogram(self.point_6_data_filtered, "auto")

    def distance(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def similarity_check(self, simulated_dist, sim_angles, point_toogle: int, simulated_points, robot_pos):

        if point_toogle == 0:

        # setup coordinates for point 3
        
            point_x = [pol2cart(self.point_3_data_filtered[index], angle_array[index])[0] for index in
                        range(0, len(self.point_3_data_filtered))]
            point_y = [pol2cart(self.point_3_data_filtered[index], angle_array[index])[1] for index in
                        range(0, len(self.point_3_data_filtered))]
        else:
            point_x = [pol2cart(self.point_6_data_filtered[index], angle_array[index])[0] for index in
                        range(0, len(self.point_6_data_filtered))]
            point_y = [pol2cart(self.point_6_data_filtered[index], angle_array[index])[1] for index in
                        range(0, len(self.point_6_data_filtered))]


        # build the lidar cartian point cloud
        lidar_pointcloud_in_cartesian = []
        for index in range(len(point_x)):
            lidar_pointcloud_in_cartesian.append([point_x[index], point_y[index]])

        # dist_r = 3
        # new_data = []
        # new_point = []
        # i = 0
        # for index in self.point_3_data_filtered:
        #     if index < dist_r:
        #         new_data.append(index)
        #         new_point.append(point_3_x[i])
        #     i += 1
        #         #self.point_3_data_filtered.remove(index)
        # self.point_3_data_filtered = new_data
        # point_3_x = new_point

        # new_data = []
        # new_point = []
        # i = 0
        # for index in self.point_3_data_filtered:
        #     if index < dist_r:
        #         new_data.append(index)
        #         new_point.append(point_3_y[i])
        #     i += 1
        #     # self.point_3_data_filtered.remove(index)
        # self.point_3_data_filtered = new_data
        # point_3_y = new_point

        # print(len(self.point_3_data_filtered))
        # print(len(self.point_6_data_filtered))
        #662
        #664

        #Calculate distance from points

        #Distance conversion already happening in point_der_dist
        point_der_dist = []
        for point in simulated_points:
            point_der_dist.append(self.distance(point[0], point[1], robot_pos[0], robot_pos[1])*resolution)

        new_data = []
        point_sim_x_dir = []
        point_sim_y_dir = []
        i = 0
        for dist in point_der_dist:
            if dist < 3 and dist > range_min:
                new_data.append(dist)
                point_sim_x_dir.append(resolution * (simulated_points[i][0] - robot_pos[0]))
                point_sim_y_dir.append(resolution * (simulated_points[i][1] - robot_pos[1]))
            i += 1
        point_der_dist = new_data

        #print(point_der_dist)

        pixel_distances = [pixel_2_m(point_der_dist[index_num], resolution) for index_num in range(0, len(point_der_dist))]
        point_der_dist = pixel_distances
        # if point_toogle == 0:
        #     # if 0 use data from point 3
        #     comparison_hist = self.point_3_np_hist
        # elif point_toogle == 1:
        #     # else if not 0 use from point 6
        #     comparison_hist = self.point_6_np_hist

        # convert the simulated data into a histogram dataset
        # simulated_hist, _ = np.histogram(point_der_dist, "auto")

        # find the norms of each set
        # print(simulated_hist)
        # simulated_norm = np.linalg.norm(simulated_hist)
        # comparison_norm = np.linalg.norm(comparison_hist)

        # Scatter plot
        # print("Here to")
        # print(len(sim_angles))
        # print(len(point_der_dist))
        # plt.scatter(point_der_dist, sim_angles, color="blue")
        # plt.scatter(self.point_3_data_filtered, angle_array, color="red")

        # point_sim_x = [pol2cart(point_der_dist[index], sim_angles[index])[0] for index in range(0, len(point_der_dist))]
        # point_sim_y = [pol2cart(point_der_dist[index], sim_angles[index])[1] for index in range(0, len(point_der_dist))]
        # plt.scatter(point_sim_x, point_sim_y, color="blue")

        #Debug plots

        # return a similarity value between 0 and 1 also attach the coordinate data in the tuple
        # similarity = 0
        # if simulated_norm > comparison_norm:
        #     similarity = (comparison_norm/simulated_norm)
        # else:
        #     similarity = (simulated_norm/comparison_norm)
        # if similarity > 0.0:
        #     print(similarity)
        #     print(robot_pos)
        #     plt.scatter(point_3_x, point_3_y, color="black")
        #     plt.scatter(point_sim_x_dir, point_sim_y_dir, color="red")
        #     plt.show()

        # x, y points of the map in cartesian 
        occupied_points_of_the_map_in_cartesian = simulated_points
        kdt = KDTree(occupied_points_of_the_map_in_cartesian)
        distances= kdt.query(lidar_pointcloud_in_cartesian, k=1)[0][:]
        # weight= np.sum (np.exp(-(distances**2)/(2*lidar_standard_deviation**2)))
        #  OR 
        # around 0.2 from here https://piazza.com/class/l66otfiv2xt5h2/post/156
        # Thanks Ahmad
        lidar_standard_deviation=0.2
        weight= np.prod(np.exp(-(distances**2)/(2*lidar_standard_deviation**2)))
        return weight




def pixel_2_m(pixel_dist: int, resolution: float) -> float:

    if pixel_dist * resolution > 12:
        # 3 was the max distance we saw in the lidar scan values
        return 3
    return pixel_dist * resolution

# set_3 = [point_3_data[index] for index in range(0, len(point_3_data), 2)]
# set_6 = [point_6_data[index] for index in range(0, len(point_3_data), 2)]

# print(set_3[0])

# p3 = (set_3[0], (20, 8)) # 190 arrays
# p6 = (set_6[0], (5, 4)) # 132 arrays

# p3_norm = np.histogram(p3, 10)

# similarity_test = similarity()
# similarity_test_result = similarity_test.similarity_check(p3, 0)

# print(similarity_test_result)

# average data scan to reduce timeset to 1 array of 720 values
# todo average the scan values in the dataset
# def avg_scan(point_data):

#     num_scans = 720
#     num_arrays = len(point_data)
#     for scan in point_data:
#         point_total = 0
#         for point in scan:
#             point_total =




# np.set_printoptions(threshold=sys.maxsize)
# im = np.array(Image.open('map_maze_2.pgm'))

# Data from map_maze_2_yaml file
# resolution of map, meters/pixel
resolution = 0.03 # 0.03 m/px
origin = [-0.843, -3.94, 0]
# min distance observable by the lidar
range_min=0.15000000596046448

angle_increment=0.008714509196579456 # from point 3 scan
angle_increment=0.008714509196579456 # from point 6 scan

# construct angle array
# this array has the angle of each lidar ray
# with ray 0 being at 0 degrees and ray 719 being at 2*PI or 360 degrees

angle_array = [angle_increment*index for index in range(1, 721)]

# print(len(angle_array))

# define infinity/out of range/never returned for a lidar scan
# inf = float('inf')
# set inf as a really large number

def pol2cart(radius, angle):

    if radius == float('inf'):
        return float('inf')
    else:
        x = radius*cos(angle)
        y = radius*sin(angle)

        cart_point = [x, y]

        return cart_point