import math
import random
import numpy as np
from itertools import chain
import matplotlib.pyplot as plt
from pathlib import Path
import sys
path_root = Path(__file__).parents[2]
sys.path.append(str(path_root))
print(sys.path)

from mte544_lab2 import similarity

class runner:
    def __init__(self):
        self.map = np.loadtxt('test.txt')
        self.x_range = [225, 320]
        self.y_range = [300, 400]
        self.probability_grid = []
        self.particles = 100 #40
        self.angle_range = [0, 2 * math.pi]
        self.sensor_range = 50
        self.sensor_fov = math.pi / 2
        self.iterations = 100
        self.offset = 300
        self.similarity = similarity.similarity()
        self.run()

    def pad_map(self):
        for i in range(0, len(self.map)):
            for j in range(0, len(self.map[i])):
                if self.map[i][j] > 200:
                    self.map[i][j] = 254

        # make the map bigger
        bigger_map = []
        for i in range(0, self.offset * 2):
            bigger_map.append([])
            for j in range(0, self.offset * 2):
                bigger_map[i].append(254)
        for i in range(0, len(self.map)):
            for j in range(0, len(self.map[i])):
                bigger_map[i + int(self.offset / 4)][j + int(self.offset / 2)] = int(self.map[i][j])
        self.map = bigger_map

    # https://rosettacode.org/wiki/Bitmap/Midpoint_circle_algorithm#Python
    def circle(self, x0, y0, radius):
        points = []
        f = 1 - radius
        ddf_x = 1
        ddf_y = -2 * radius
        x = 0
        y = radius
        points.append((x0, y0 + radius))
        points.append((x0, y0 - radius))
        points.append((x0 + radius, y0))
        points.append((x0 - radius, y0))

        while x < y:
            if f >= 0:
                y -= 1
                ddf_y += 2
                f += ddf_y
            x += 1
            ddf_x += 2
            f += ddf_x
            points.append((x0 + x, y0 + y))
            points.append((x0 - x, y0 + y))
            points.append((x0 + x, y0 - y))
            points.append((x0 - x, y0 - y))
            points.append((x0 + y, y0 + x))
            points.append((x0 - y, y0 + x))
            points.append((x0 + y, y0 - x))
            points.append((x0 - y, y0 - x))
        return points

    # http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
    def bresenham(self, x1, y1, x2, y2):
        # Setup initial conditions
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Max distance = inf
    def brezenham_to_dist(self, points, pose, angle):
        x = 0
        y = 0
        for point in points:
            x = point[0]
            y = point[1]
            if self.map[y][x] == 0:
                return (self.distance(x, y, pose[0], pose[1]), False, (x, y), angle)
        return (9999, True, (x,y), angle)

    def getViewCords(self, pose):
        angle1 = pose[2] - self.sensor_fov / 2
        angle2 = pose[2] + self.sensor_fov / 2
        # Scan a circle
        seach_points = self.circle(pose[0], pose[1], 126)
        # point3 = [sensor_range*math.cos(angle2+pose[0]), sensor_range*math.sin(angle2)+pose[1]]
        # Get raycast to this line
        found_map = []
        i = 0
        for point in seach_points:
            # Line from robot to point
            angle = 2 * math.pi / 716 * i
            val = self.brezenham_to_dist(self.bresenham(pose[0], pose[1], point[0], point[1]), pose, angle)
            # Is infinite for now dont include (While testing)
            # if val[0] == 9999:
            #     pass
            # else:
            found_map.append(val)
            i += 1
        points = []
        distances = []
        angles = []
        for found in found_map:
            points.append(found[2])
            distances.append(found[0])
            #print(found[3])
            angles.append(found[3])
        # print(points)
        x = []
        y = []
        for point in points:
            x.append(point[0])
            y.append(point[1])
        return points, seach_points, distances, angles

    def getPosesUniform(self, n):
        poses = []
        for i in range(0, n):
            x = random.uniform(self.x_range[0], self.x_range[1])
            y = random.uniform(self.y_range[0], self.y_range[1])
            angle = random.uniform(self.angle_range[0], self.angle_range[1])
            pose = (int(x), int(y), 0)
            poses.append(pose)
        return poses

    # Generate new poses
    def getPoses(self, n):
        poses = []
        for i in range(0, n):
            indexes = []
            for i in range(0, len(self.probability_grid)):
                indexes.append(i)
                
            index = random.choices(indexes, weights=self.probability_grid)
            y = int(index[0] / (self.offset * 2))
            x = index[0] % (self.offset * 2)
            assert self.getElement(self.probability_grid, y, x) - self.probability_grid[index[0]] == 0
            pose = (x,y,0)
            #print(pose)
            poses.append(pose)
        return poses
    # https://stackoverflow.com/questions/2151084/map-a-2d-array-onto-a-1d-array
    def setElement(self, arr, row, col, value):
        arr[self.offset * 2 * row + col] = value

    def getElement(self, arr, row, col):
        return arr[self.offset * 2 * row + col]

    def update_pose_distribution(self, pose_dist):
        self.probability_grid = []
        pose_similarities = pose_dist
        for i in range(0, (self.offset * 2) ** 2):
            self.probability_grid.append(0)
        #Generate pose probabilitiy distrobution
        #pose_sim[1] is location pose_sim[0][0] is similarity
        width = self.offset
        for pose_sim in pose_similarities:
            x = pose_sim[1][0]
            y = pose_sim[1][1]
            # index 1d array as 2d
            if self.getElement(self.probability_grid, y, x) < pose_sim[0]:
                self.setElement(self.probability_grid, y, x, pose_sim[0])

        # Apply Gaussian blur so not all poses are in the exact same spot
        # Array out of bounds if used near map edges
        blurred_grid = self.probability_grid.copy()
        for i in range(self.offset * 2):
            for j in range(self.offset * 2):
                val = self.getElement(self.probability_grid, j, i)
                if val > 1:
                    # print(val)
                    self.setElement(blurred_grid, j, i, val * 4 / 16)
                    self.setElement(blurred_grid, j + 1, i + 1, val / 16)
                    self.setElement(blurred_grid, j, i + 1, val * 2 / 16)
                    self.setElement(blurred_grid, j - 1, i + 1, val / 16)
                    self.setElement(blurred_grid, j - 1, i, val * 2 / 16)
                    self.setElement(blurred_grid, j + 1, i - 1, val / 16)
                    self.setElement(blurred_grid, j, i - 1, val * 2 / 16)
                    self.setElement(blurred_grid, j - 1, i - 1, val / 16)
                    self.setElement(blurred_grid, j + 1, i, val * 2 / 16)

        self.probability_grid = blurred_grid

        #For testing
        test_blur = []
        for i in range(0, self.offset * 2):
            test_blur.append([])
            for j in range(0, self.offset * 2):
                val = self.getElement(self.probability_grid, j, i)
                if val > 1:
                    pass
                    #print(val)
                test_blur[i].append(val)
        #print("Testing blur")
        # plt.imshow(test_blur)
        # plt.gray()
        # plt.show()
        # quit()

    def run(self):
        self.pad_map()
        poses = self.getPosesUniform(self.particles)
        for i in range(0, self.iterations):
            x = 287 #x = 308
            y = 329 #
            # x = 308
            # y = 344
            pose_similarities = []

            #remove duplicates
            poses = list(set([i for i in poses]))

            #Measure progress

            poses = []
            poses.append((x,y,0))
            for pose in poses:
                #print(pose)
                plt.plot(pose[0], pose[1], 'ro')
                #plt.plot(0, 0, 'ro')
            plt.imshow(self.map)
            plt.show()

            similarities = []
            j = 0
            #quit()
            #0.9967758030280428
            #(287, 329, 0)
            for pose in poses:
                #point_grid, search_grid = self.getViewCords((x, y, math.pi / 4))
                point_grid, search_grid, distances, angles = self.getViewCords(pose)
                # print(len(distances))
                # print(distances)
                # plt.plot(distances)
                # plt.gray()
                # plt.show()
                #For test renders on map
                # for i in range(0, self.offset * 2):
                #     for j in range(0, self.offset * 2):
                #         self.map[i][j] = 254
                # for point in point_grid:
                #     self.map[point[1]][point[0]] = 100
                # for point in search_grid:
                #     self.map[point[1]][point[0]] = 60
                # self.map[y][x] = 150
                # plt.imshow(self.map)
                # plt.show()
                # quit()

                # print(len(distances))
                # print(distances)
                # print("Here")
                # print(len(angles))
                # print(angles)
                sim = self.similarity.similarity_check(distances, angles, 0, point_grid, pose)
                similarities.append(sim)
                sim_and_pos = (sim, (pose[0], pose[1]))
                pose_similarities.append(sim_and_pos)
                j += 1
                print(j)
                #quit()

            #Plot similarities
            print(similarities)
            plt.plot(similarities)
            plt.gray()
            plt.show()
            quit()


            # pose_similarities.append((20, (10, 6)))
            # pose_similarities.append((80, (10, 6)))
            # pose_similarities.append((10, (7, 7)))
            # pose_similarities.append((99, (5, 5)))
            self.update_pose_distribution(pose_similarities)
            poses = self.getPoses(self.particles)
            #poses.extend(self.getPosesUniform(self.particles))
#
runner = runner()