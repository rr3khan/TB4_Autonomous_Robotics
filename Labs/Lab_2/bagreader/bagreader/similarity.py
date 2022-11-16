import numpy as np
from PIL import Image
import sys
# import scan data
# data is currently in the form of alternating distance arrays 
# and intensity arrays
# scan data for point 3
from point_3_datascan_formatted import point_3_data
# scan data for point 6
from point_6_datascan_formatted import point_6_data

# print(len(point_3_data)) # 380 arrays
# print(len(point_6_data)) # 264 arrays

# filter out intesity data from data set
point_3_data_filtered = [point_3_data[index] for index in range(0, len(point_3_data), 2)] # 190 arrays
point_6_data_filtered = [point_6_data[index] for index in range(0, len(point_6_data), 2)] # 132 arrays


# average data scan to reduce timeset to 1 array of 720 values
# todo average the scan values in the dataset
# def avg_scan(point_data):

#     num_scans = 720
#     num_arrays = len(point_data)
#     for scan in point_data:
#         point_total = 0
#         for point in scan:
#             point_total =



# print(len(point_3_data_filtered))
# print(point_6_data_filtered[1][0])

np.set_printoptions(threshold=sys.maxsize)
im = np.array(Image.open('map_maze_2.pgm'))

# Data from map_maze_2_yaml file
# resolution of map, meters/pixel
resolution = 0.03 # 0.03 m/px
origin = [-0.843, -3.94, 0]

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
inf = 9999

def similariy(point_3_data, point_6_data):

    # generate histogram bins for the point 3 data
    point_3_np_hist, _ = np.histogram(point_3_data, 10000)
    point_6_np_hist, _ = np.histogram(point_6_data, 10000)

    similarity = np.linalg.norm(point_3_np_hist)/np.linalg.norm(point_6_np_hist)

    return similarity

    # sum = 0
    # for a1, b1 in zip(array_1,array_2):
    #     #  filter out non-return/ inf values
    #     # max range of lidar is 12 m
    #     # if a1 < 12 and b1 < 12:
    #         # print("a1, b1", a1, b1)
    #         sum = sum + (a1-b1)*(a1-b1)

    # return math.sqrt(sum)/len(array_1)

    # return math.sqrt(sum((a1-b1)*(a1-b1) for a1,b1 in zip(array_1,array_2)))/len(array_1)

# print(similariy(point_3_data_filtered[0], point_6_data_filtered[0]))
# print(similariy(X,Z))



def pixel_2_m(pixel_dist: int, resolution: float) -> float:
    return pixel_dist * resolution


# def similarity():
#     return

# # print("Num intensities: ", len(intentsity_data_example) )
# # print("Num range_example_data: ", len(range_example_data) )
# # print(inf)

# print("Range testing",  similariy(range_example_data, range_example_data_2))

# exp_1 = [1, 2, 3, inf]
# exp_2 = [1, inf, 3, inf]
# print("Range experiment one", similariy(exp_1, exp_2))