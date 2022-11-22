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


def pixel_2_m(pixel_dist: int, resolution: float) -> float:

    if pixel_dist * resolution > 12:
        # 3 was the max distance we saw in the lidar scan values
        return 3
    return pixel_dist * resolution


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


# // similarity class


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

    # use the transformation matrix to rotate a point
    def rotation_transform(self, about_point, point_to_rotate, angle):
        apx, apy = about_point
        prx, pry = point_to_rotate

        rotatedx = cos(angle)*(prx) - sin(angle)*(pry)
        rotatedy = sin(angle)*(prx) + cos(angle)*(pry)

        return [rotatedx/0.03, rotatedy/0.03]

    def apply_translation(self, about_point, point_to_rotate, angle):
        apx, apy = about_point
        prx, pry = point_to_rotate

        trans_x = prx + apx
        trans_y = pry + apy

        return [trans_x, trans_y]

    #Robot pos (x,y, rot)
    def similarity_check(self, point_toogle: int, simulated_points, robot_pos, do_plot):

        # x y coordinates for lidar data
        points = []
        if point_toogle == 0:

        # setup coordinates for point 3
            for index in range(0, len(self.point_3_data_filtered)):
                point_to_insert = pol2cart(self.point_3_data_filtered[index], angle_array[index])
                # filter out infinite values
                if point_to_insert != float('inf'):
                    points.append(point_to_insert)

        else:
            for index in range(0, len(self.point_6_data_filtered)):
                point_to_insert = pol2cart(self.point_6_data_filtered[index], angle_array[index])
                # filter out infinite values
                if point_to_insert != float('inf'):
                    points.append(point_to_insert)


        # build the lidar cartian point cloud
        lidar_pointcloud_in_cartesian = []
        for index in range(len(points)):
            lidar_pointcloud_in_cartesian.append(points[index])


        # rotate all points in map abour the robot pose
        rotated_lidar_points = []
        about_point = (robot_pos[0], robot_pos[1])
        for points in lidar_pointcloud_in_cartesian:
            rotated_lidar_points.append(self.rotation_transform(about_point, points, robot_pos[2]))

        translated_lider_points = []
        about_point = (robot_pos[0], robot_pos[1])
        for points in rotated_lidar_points:
            translated_lider_points.append(self.apply_translation(about_point, points, robot_pos[2]))

        rotated_lidar_points = translated_lider_points

        # Scatter plot
        if do_plot:
            plt.scatter([i[0] for i in rotated_lidar_points],[i[1] for i in rotated_lidar_points], color="blue")
            plt.scatter([i[0] for i in simulated_points], [i[1] for i in simulated_points], color="red")
            # plt.xlim(-5, 6)
            # plt.ylim(-10, 15)
            # plt.xlim(-1000, 1000)
            # plt.ylim(-1000, 1000)
            plt.show()

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
        # occupied_points_of_the_map_in_cartesian = rotated_sim_points
        kdt = KDTree(np.array(simulated_points))
        distances = kdt.query(rotated_lidar_points, k=1)[0][:]
        # weight= np.sum (np.exp(-(distances**2)/(2*lidar_standard_deviation**2)))
        #  OR
        # around 0.2 from here https://piazza.com/class/l66otfiv2xt5h2/post/156
        # Thanks Ahmad
        lidar_standard_deviation=0.2
        # weight = np.prod(np.exp(-(distances**2)/(2*lidar_standard_deviation**2)))
        weight= np.sum(np.exp(-(distances**2)/(2*lidar_standard_deviation**2)))
        return weight

set_3 = [point_3_data_all[0][index] for index in range(0, len(point_3_data_all[0]), 2)]
set_6 = [point_6_data_all[0][index] for index in range(0, len(point_6_data_all), 2)]

# print(set_3[0])

# p3 = (set_3[0], (20, 8)) # 190 arrays
# p6 = (set_6[0], (5, 4)) # 132 arrays

# build a test dataset of x y points based on point 3 lidar data
p3 = []
for index in range(0, len(set_3)):
    point_to_insert = pol2cart(set_3[index], angle_array[index])
    # filter out infinite values
    if point_to_insert != float('inf'):
        p3.append(point_to_insert)

p6 = []
for index in range(0, len(set_6)):
    point_to_insert = pol2cart(set_6[index], angle_array[index])
    # filter out infinite values
    if point_to_insert != float('inf'):
        p6.append(point_to_insert)

# p3_norm = np.histogram(p3, 10)

# similarity_test = similarity()
# similarity_test_result = similarity_test.similarity_check([], [], 0, p6, [])

# print(similarity_test_result)