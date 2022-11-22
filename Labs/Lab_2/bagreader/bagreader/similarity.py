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

# import for plotting
import matplotlib.pyplot as plt

class similarity():
    def __init__(self):

        # filter out intesity data from data set
        self.point_3_data_filtered = [point_3_data[index] for index in range(0, len(point_3_data), 2)] # 190 arrays
        self.point_6_data_filtered = [point_6_data[index] for index in range(0, len(point_6_data), 2)] # 132 arrays

        # generate histogram bins for the point 3 data
        self.point_3_np_hist, _ = np.histogram(self.point_3_data_filtered[0], 10000)
        self.point_6_np_hist, _ = np.histogram(self.point_6_data_filtered[0], 10000)

    def similarity_check(self, simulated_data, point_toogle: int):

        if point_toogle == 0:
            # if 0 use data from point 3
            comparison_hist = self.point_3_np_hist
        else:
            # else if not 0 use from point 6
            comparison_hist = self.point_6_np_hist

        # convert the simulated data into a histogram dataset
        simulated_hist = np.histogram(simulated_data[0], 10000)

        # find the norms of each set
        simulated_norm = np.linalg.norm(simulated_hist[0])
        comparison_norm = np.linalg.norm(comparison_hist)
        
        print("comparision norm", comparison_norm)
        print("simulated norm", simulated_norm)

        # return a similarity value between 0 and 1 also attach the coordinate data in the tuple
        if simulated_norm > comparison_norm:
            return (comparison_norm/simulated_norm, simulated_data[1])
        else:
            return (simulated_norm/comparison_norm, simulated_data[1])


def pixel_2_m(pixel_dist: int, resolution: float) -> float:
    return pixel_dist * resolution

set_3 = [point_3_data[index] for index in range(0, len(point_3_data), 2)]
set_6 = [point_6_data[index] for index in range(0, len(point_6_data), 2)]

# print(set_3[0])

p3 = (set_3[0], (20, 8)) # 190 arrays
p6 = (set_6[0], (5, 4)) # 132 arrays

# p3_norm = np.histogram(p3, 10)

similarity_test = similarity()
similarity_test_result = similarity_test.similarity_check(p3, 1)

print(similarity_test_result)

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

# # Scatter plot
# plt.scatter(set_3[0], angle_array)
# plt.scatter(set_6[0], angle_array)

def pol2cart(radius, angle):
    x = radius*np.cos(angle)
    y = radius*np.sin(angle)

    return (x, y)

point_3_data_filtered = [point_3_data[index] for index in range(0, len(point_3_data), 2)] # 190 arrays
point_6_data_filtered = [point_6_data[index] for index in range(0, len(point_6_data), 2)] # 132 arrays

point_3_x = [pol2cart(point_3_data_filtered[0][index], angle_array[index])[0] for index in range(0, len(point_3_data_filtered[0]))]
point_3_y = [pol2cart(point_3_data_filtered[0][index], angle_array[index])[1] for index in range(0, len(point_3_data_filtered[0]))]

plt.scatter(point_3_x, point_3_y, color="black")

# Display the plot
plt.show()
