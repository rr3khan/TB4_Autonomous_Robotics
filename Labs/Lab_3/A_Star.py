import sys
from Queue import PriorityQueue
from math import sqrt
from cost_map_no_postbox import cost_map_data
import numpy as np
import pygame
from treelib import Tree

#In render y is given x and x is y

WINDOW_HEIGHT = 600
WINDOW_WIDTH = 600
BLACK = (0, 0, 0)
WHITE = (200, 200, 200)
GRAY = (112, 128, 144)
GREEN = (0, 200, 0)
RED = (200, 0, 0)
BLUE = (0, 0, 200)
PURPLE = (200, 0, 200)

class runner:
    def __init__(self, width, height, mapData):
        self.visited = set()
        self.explored = set()
        self.visitedCost = {}
        self.totalExplored = 0
        self.cost = -1 #-1 so as not to account for startinh tile
        self.queue = []
        self.priorityQueue = PriorityQueue()
        self.done = False
        self.path = []
        self.maze = []
        self.tree = Tree()
        self.width = width
        self.height = height
        self.scale = 0.05
        print("Running")
        self.maze = mapData
        self.showPath = True
        self.res = 0.05
        self.x_off = 195
        self.y_off = 74
        # val = self.G_to_P(0, 0)
        # val = self.G_to_P(2.78, -1.04)
        # print(2.78 / self.res)
        # print(-1.04 / self.res)
        # print(val)
        # print(self.P_to_G(val[0], val[1]))
        # y = val[0]
        # x = val[1]
        x = 0
        y = 0
        #self.findPath((-3, -3),(2.78,-1.04))

        self.findPath((-4.274,-2.569),(2.966, 2.365))

    def G_to_P(self, x, y):
        # print("W")
        # print(self.height - int(x/self.res+self.y_off))
        # print("W_e")
        # print(int(y/self.res+self.y_off))
        print(self.width - int(-x/self.res+ self.x_off))
        return (self.width - int(-x/self.res+ self.x_off), int(y/self.res+self.y_off))
    def P_to_G(self, x, y):
        return ((x- self.x_off)*self.res, (y- self.y_off)*self.res)

    def get_map_cords(self, x, y):
        return y,self.width-x

    def drawGrid(self):
        blockSize = 2  # Set the size of the grid block
        for x in range(0, self.width):
            for y in range(0, self.height):
                x_ = self.get_map_cords(x, y)[0]
                y_ = self.get_map_cords(x, y)[1]
                rect = pygame.Rect(x_ * blockSize, y_ * blockSize, blockSize, blockSize)
                Color = (0, 0, self.maze[y][x])
                if self.maze[y][x] == 100:
                    pygame.draw.rect(SCREEN, GRAY, rect, 1)
                else:
                    pygame.draw.rect(SCREEN, Color, rect, 1)
                if self.startPosition[0] == x and self.startPosition[1] == y:
                    pygame.draw.rect(SCREEN, WHITE, rect, 1)
                elif self.endPosition[0] == x and self.endPosition[1] == y:
                    pygame.draw.rect(SCREEN, GREEN, rect, 1)
                elif (x, y) in self.path:
                    pygame.draw.rect(SCREEN, RED, rect, 1)
                elif (x, y) in self.explored:
                    if self.showPath:
                        pygame.draw.rect(SCREEN, PURPLE, rect, 1)


    def findPath(self, startPos, endPos):
        #Reset global vars
        self.visited = set()
        self.explored = set()
        self.visitedCost = {}
        self.totalExplored = 0
        self.cost = -1  # -1 so as not to account for startinh tile
        self.queue = []
        self.priorityQueue = PriorityQueue()
        self.done = False
        self.path = []
        self.tree = Tree()
        self.endPosition = self.G_to_P(endPos[0], endPos[1])
        self.startPosition = self.G_to_P(startPos[0], startPos[1])
        global SCREEN, CLOCK
        pygame.init()
        SCREEN = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        CLOCK = pygame.time.Clock()
        SCREEN.fill(BLACK)

        node = self.tree.create_node(tag='0', data=self.startPosition)
        self.a_star(self.startPosition, node)
        # print(self.path)
        #self.path = [(75, 176), (74, 176), (74, 175), (73, 175), (73, 174), (72, 174), (72, 173), (71, 173), (71, 172), (70, 172), (70, 171), (69, 171), (69, 170), (68, 170), (68, 169), (67, 169), (67, 168), (66, 168), (66, 167), (65, 167), (65, 166), (64, 166), (64, 165), (63, 165), (63, 164), (62, 164), (62, 163), (61, 163), (61, 162), (60, 162), (60, 161), (59, 161), (59, 160), (58, 160), (58, 159), (57, 159), (57, 158), (56, 158), (56, 157), (55, 157), (55, 156), (54, 156), (54, 155), (53, 155), (53, 154), (52, 154), (52, 153), (51, 153), (51, 152), (50, 152), (50, 151), (49, 151), (49, 150), (49, 149), (49, 148), (49, 147), (49, 146), (49, 145), (49, 144), (49, 143), (49, 142), (49, 141), (49, 140), (49, 139), (49, 138), (49, 137), (49, 136), (49, 135), (49, 134), (49, 133), (49, 132), (49, 131), (49, 130), (49, 129), (49, 128), (49, 127), (49, 126), (49, 125), (49, 124)]
        #self.path =  [(45, 124), (44, 124), (43, 124), (42, 124), (41, 124), (41, 125), (41, 126), (40, 126), (39, 126), (39, 127), (39, 128), (39, 129), (39, 130), (40, 130), (40, 131), (40, 132), (41, 132), (41, 133), (42, 133), (42, 134), (43, 134), (43, 135), (44, 135), (45, 135), (45, 136), (46, 136), (47, 136), (48, 136), (49, 136), (49, 137), (50, 137), (51, 137), (52, 137), (53, 137), (54, 137), (54, 136), (55, 136), (56, 136), (57, 136), (58, 136), (58, 135), (59, 135), (60, 135), (60, 134), (61, 134), (61, 133), (62, 133), (62, 132), (63, 132), (63, 131), (63, 130), (64, 130), (64, 129), (64, 128), (64, 127), (64, 126), (65, 126), (65, 125), (65, 124), (65, 123), (65, 122), (65, 121), (65, 120), (65, 119), (65, 118), (65, 117), (65, 116), (65, 115), (65, 114), (65, 113), (65, 112), (66, 112), (67, 112), (68, 112), (69, 112), (70, 112), (71, 112), (72, 112), (73, 112), (74, 112), (75, 112), (76, 112), (77, 112), (78, 112), (79, 112), (80, 112), (81, 112), (82, 112), (83, 112), (84, 112), (85, 112), (86, 112), (87, 112), (88, 112), (89, 112), (90, 112), (91, 112), (92, 112), (93, 112), (93, 113), (94, 113), (95, 113), (95, 114), (96, 114), (96, 115), (97, 115), (97, 116), (98, 116), (98, 117), (98, 118), (99, 118), (99, 119), (99, 120), (99, 121), (99, 122), (100, 122), (100, 123), (100, 124), (100, 125), (99, 125), (99, 126), (99, 127), (99, 128), (99, 129), (98, 129), (98, 130), (98, 131), (98, 132), (98, 133), (98, 134), (99, 134), (99, 135), (99, 136), (99, 137), (99, 138), (100, 138), (100, 139), (100, 140), (100, 141), (100, 142), (101, 142), (102, 142), (103, 142), (104, 142), (105, 142), (106, 142), (107, 142), (108, 142)]
        #self.path = [(164, 49), (164, 50), (165, 50), (165, 51), (166, 51), (166, 52), (167, 52), (167, 53), (168, 53), (168, 54), (169, 54), (169, 55), (170, 55), (170, 56), (171, 56), (171, 57), (172, 57), (172, 58), (173, 58), (173, 59), (174, 59), (174, 60), (175, 60), (175, 61), (176, 61), (176, 62), (177, 62), (177, 63), (178, 63), (178, 64), (179, 64), (179, 65), (180, 65), (180, 66), (181, 66), (182, 66), (183, 66), (184, 66), (185, 66), (186, 66), (187, 66), (187, 67), (188, 67), (189, 67), (190, 67), (191, 67), (191, 68), (192, 68), (193, 68), (193, 69), (194, 69), (195, 69), (196, 69), (197, 69), (198, 69), (199, 69), (200, 69), (201, 69), (202, 69), (203, 69), (204, 69), (205, 69), (206, 69), (207, 69), (208, 69), (209, 69), (210, 69), (211, 69), (212, 69), (213, 69), (214, 69), (215, 69), (216, 69), (217, 69), (218, 69), (219, 69), (220, 69), (220, 70)]
        #self.path = [(294, 117), (293, 117), (292, 117), (291, 117), (291, 116), (291, 115), (291, 114), (290, 114), (290, 113), (289, 113), (289, 112), (288, 112), (288, 111), (287, 111), (287, 110), (286, 110), (286, 109), (285, 109), (285, 108), (284, 108), (284, 107), (283, 107), (283, 106), (282, 106), (282, 105), (281, 105), (281, 104), (280, 104), (280, 103), (279, 103), (279, 102), (279, 101), (279, 100), (279, 99), (279, 98), (279, 97), (279, 96), (279, 95), (279, 94), (279, 93), (279, 92), (279, 91), (279, 90), (279, 89), (279, 88), (279, 87), (279, 86), (279, 85), (279, 84), (279, 83), (279, 82), (279, 81), (279, 80), (279, 79), (278, 79), (278, 78), (278, 77), (278, 76), (278, 75), (278, 74), (278, 73), (278, 72), (277, 72), (277, 71), (276, 71), (275, 71), (275, 70), (274, 70), (273, 70), (272, 70), (271, 70), (271, 69), (270, 69), (269, 69), (268, 69), (267, 69), (266, 69), (265, 69), (264, 69), (263, 69), (262, 69), (261, 69), (260, 69), (259, 69), (258, 69), (257, 69), (256, 69), (255, 69), (254, 69), (253, 69), (252, 69), (251, 69), (250, 69), (249, 69), (248, 69), (248, 70), (248, 71)]

        while True:
            self.drawGrid()
            #print(pygame.mouse.get_pos())
            # pos = pygame.mouse.get_pos()
            # print((pos[1], pos[0]/2))
            # print(self.P_to_G((pos[1])/2, pos[0]/2))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()

    def get_tree(self, node):
        try:
            print("Path")
            while True:
                self.path.append(node.data)
                node = self.tree.parent(node.identifier)
                self.cost += 1
        except:
            return

    def a_star(self, position, node):
        self.visited.add(position)
        self.visitedCost[position] = int(node.tag)
        #Priority queue is order based on first object
        self.priorityQueue.put((self.manhattan_distance(position, self.endPosition), position, node))
        previous_node = node
        while self.priorityQueue.qsize() > 0 and not self.done:
            distance, position, node = self.priorityQueue.get()
            self.totalExplored += 1
            self.explored.add(position)
            if (position == self.endPosition):
                print("Found")
                self.get_tree(node)
                print(position)
                print(self.cost)
                print(self.totalExplored)
                self.done = True
                print("Done")
                break
            for new_position in self.getMoves_order1(position):
                #
                x = new_position[0]
                y = new_position[1]
                #print("X: " + str(x) + " Y:" + str(y))
                cost = int(node.tag) + int(self.distance(new_position, (x,y))) + int(self.maze[y][x]*self.scale)
                revisit = False
                if new_position in self.visited:
                    if self.visitedCost[new_position] > cost:
                        revisit = True
                if new_position not in self.visited or revisit:
                    new_node = self.tree.create_node(tag='' + str(cost), data=new_position, parent=node)
                    self.visited.add(new_position)
                    self.visitedCost[new_position] = int(node.tag)
                    self.priorityQueue.put((self.manhattan_distance(new_position, self.endPosition) + cost, new_position, new_node))

    def distance(self, point1, point2):
        return sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

    def manhattan_distance(self, position, position2):
        return min(self.distance(position, position2), self.distance(position, position2))

    # In render y is given x and x is y
    def getMoves_order1(self, position):
        moves = []
        #Left
        if self.canMove((position[0], position[1] - 1)):
            moves.append((position[0], position[1] - 1))
        #Up
        if self.canMove((position[0] + 1, position[1])):
            moves.append((position[0] + 1, position[1]))
        #Right
        if self.canMove((position[0], position[1] + 1)):
            moves.append((position[0], position[1] + 1))
        #Down
        if self.canMove((position[0] - 1, position[1])):
            moves.append((position[0] - 1, position[1]))
        return moves

    def canMove(self, position):
        x = position[0]
        y = position[1]
        #print("X: " + str(x) + " Y:" + str(y) + " H: " + str(self.height))
        # self.maze x and y axises are flipped
        inBounds = not (x < 0 or x >= self.width or y < 0 or y >= self.height)
        if inBounds:
            return self.maze[y][x] < 100
        return inBounds

runner = runner(310, 195, np.array(cost_map_data).reshape(-1, 310).tolist())