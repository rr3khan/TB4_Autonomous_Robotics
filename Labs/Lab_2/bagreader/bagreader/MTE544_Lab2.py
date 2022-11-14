import math
import random
import matplotlib.pyplot as plt
import pickle as pkl
import numpy as np

f = open('map_maze_2.pgm', 'rb')

#https://stackoverflow.com/questions/35723865/read-a-pgm-file-in-python
def read_pgm(pgmf):
    """Return a raster of integers from a PGM as a list of lists."""
    assert pgmf.readline() == 'P5\n'
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    return raster

raster_file = read_pgm(f)

#print(raster_file)
i = 0
for row in raster_file:
    j = 0
    for pixle in row:
        #print(pixle)
        if pixle > 0:
            raster_file[i][j] = 254
        j += 1
    i += 1

filename = 'output_array2.txt'
fileObject = open(filename, 'wb')

pkl.dump(raster_file, fileObject)
fileObject.close()


fileObject2 = open('C:\Users\jpunr\PycharmProjects\ECE457A\output_array2.txt', 'r')
raster_file = pkl.load(fileObject2)
fileObject2.close()

# with open('test.txt', 'w') as outfile:
#     np.savetxt(outfile, raster_file)
# print(raster_file)

#https://stackoverflow.com/questions/3886281/display-array-as-raster-image-in-python
plt.imshow(raster_file)
plt.gray()
plt.show()

num = 0
for i in raster_file:
    for j in raster_file:
        num+=1

print(num)
#This might be different from ros global cords (Doesnt matter?)
x_range = [0, 200]
y_range = [0, 300]
angle_range = [0, 2*math.pi]
sensor_fov = 120

#http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
        Produces a list of tuples from start and end

        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """
    # Setup initial conditions
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

# def DrawFilledTriangle (P0, P1, P2, map):
#     points = []
#     #Sort the points so that y0 <= y1 <= y2
#     x0 = P0[0]
#     y0 = P0[1]
#     x1 = P1[0]
#     y1 = P1[1]
#     x2 = P2[0]
#     y2 = P2[1]
#     if y1 < y0:
#         swap(P1, P0)
#     if y2 < y0:
#         swap(P2, P0)
#     if y2 < y1:
#         swap(P2, P1)
#
#     #Compute the x coordinates of the triangle edges
#     #TODO: This is called backwards?
#     x01 = bresenham(y0, x0, y1, x1)
#     x12 = bresenham(y1, x1, y2, x2)
#     x02 = bresenham(y0, x0, y2, x2)
#
#     #Concatenate the short sides
#     remove_last(x01)
#     x012 = x01 + x12
#
#     #Determine which is left and which is right
#     m = math.floor(x012.length / 2)
#     if x02[m] < x012[m]:
#         x_left = x02
#         x_right = x012
#     else:
#         x_left = x012
#         x_right = x02
#
#
#     #Draw the horizontal segments
#     y = y0
#     for y in range(y0, y2):
#         x = x_left[y - y0]
#         for x in range(x,x_right[y - y0]):
#             points[x][y].append()

#https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def line_intersection(line1, line2):
    intersection = True
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        intersection = False
       #raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y, intersection

def getViewCords(pose):
    angle1 = pose[2] - sensor_fov / 2
    angle2 = pose[2] + sensor_fov / 2
    dx = math.cos(angle1)
    dy = math.sin(angle1)
    robot_line_1 = ((pose[0], pose[1]), (0, 10))
    point1 = [x_range[0], y_range[0]]
    point2 = [x_range[1], y_range[0]]
    point3 = [x_range[0], y_range[1]]
    point4 = [x_range[1], y_range[1]]
    lines = [(point1, point2),(point2,point3), (point3, point4), (point4, point1)]
    for line in lines:
        line_intersection(robot_line_1, line)

# def getViewFromPose(pose, map):
#     return DrawFilledTriangle()

def getPoses(n):
    poses = []
    for i in range(0, n):
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        angle = random.uniform(angle_range[0], angle_range[1])
        pose = [x, y, angle]
    return poses

m = 1
M = 10
for m in range(0, M):
    #Generate particles (Random poses)
    poses = getPoses(100)
    # for pose in poses:
    #     getViewFromPose(pose, raster_file)

print line_intersection(((0,0), (0,10)), ((-5,5), (5,5)))
print line_intersection(((0,10), (0,10)), ((-5,5), (5,5)))