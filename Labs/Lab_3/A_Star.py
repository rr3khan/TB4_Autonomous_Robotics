import sys
from Queue import PriorityQueue
#import PriorityQueue
from math import sqrt

import treelib
import pygame
from treelib import Tree

maze = [[0,   0,   0,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.8,   1,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.9,   1,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 0, 0, 0, 0],
            [0,   0,   0, 0.9,   0,   1, 0, 0, 0, 0],
            [0,   0, 0.9,   1,   1, 0.7, 0, 0, 0, 0],
            [0,   0,   0,   1,   0,   0, 0, 0, 0, 0],
            [0,   0,   0,   0, 0.9,   0, 0, 0, 0, 0],
            [0,   0,   0,   0,   0,   0, 0, 0, 0, 0]]

moves = 1000
cost = 0
width = len(maze[0]) - 1
height = len(maze) - 1

#In render y is given x and x is y

exit1 = (8, 4)
start = (5, 2)

WINDOW_HEIGHT = 600
WINDOW_WIDTH = 600
BLACK = (0, 0, 0)
WHITE = (200, 200, 200)
GREEN = (0, 200, 0)
RED = (200, 0, 0)
BLUE = (0, 0, 200)
PURPLE = (200, 0, 200)

class runner:
    def __init__(self):
        self.visited = set()
        self.explored = set()
        self.visitedCost = {}
        self.totalExplored = 0
        self.cost = -1 #-1 so as not to account for startinh tile
        self.queue = []
        self.scale = 1
        self.priorityQueue = PriorityQueue()
        self.done = False
        self.path = []
        self.tree = Tree()
        print("Running")
        self.map_to_int()
        self.projectLoop()

    def map_to_int(self):
        for i in range(0, len(maze)):
            for j in range(0, len(maze[i])):
                # print maze[i][j]
                # print maze[i]
                maze[i][j] = int(255*maze[i][j])

    def get_map_cords(self, x, y):
        return y,width-x

    def drawGrid(self):
        blockSize = 20  # Set the size of the grid block
        for x in range(0, width):
            for y in range(0, height):
                x_ = self.get_map_cords(x, y)[0]
                y_ = self.get_map_cords(x, y)[1]
                rect = pygame.Rect(x_ * blockSize, y_ * blockSize, blockSize, blockSize)
                Color = (0, 0, maze[x][y])
                pygame.draw.rect(SCREEN, Color, rect, 1)
                if start[0] == x and start[1] == y:
                    pygame.draw.rect(SCREEN, WHITE, rect, 1)
                elif exit1[0] == x and exit1[1] == y:
                    pygame.draw.rect(SCREEN, GREEN, rect, 1)
                elif (x, y) in self.path:
                    pygame.draw.rect(SCREEN, RED, rect, 1)
                elif (x, y) in self.explored:
                    pygame.draw.rect(SCREEN, PURPLE, rect, 1)

    def projectLoop(self):
        global SCREEN, CLOCK
        pygame.init()
        SCREEN = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        CLOCK = pygame.time.Clock()
        SCREEN.fill(BLACK)
        while True:
            if not self.done:
                #A*
                node = self.tree.create_node(tag='0', data=start)
                self.a_star(start, node)
                print(self.path)
            self.drawGrid()
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
        self.priorityQueue.put((self.manhattan_distance(position, exit1), position, node))
        previous_node = node
        while self.priorityQueue.qsize() > 0 and not self.done:
            distance, position, node = self.priorityQueue.get()
            self.totalExplored += 1
            self.explored.add(position)
            if (position == exit1):
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
                cost = int(node.tag) + int(self.distance(new_position, (x,y))) + int(maze[y][x]*self.scale)
                revisit = False
                if new_position in self.visited:
                    if self.visitedCost[new_position] > cost:
                        revisit = True
                if new_position not in self.visited or revisit:
                    new_node = self.tree.create_node(tag='' + str(cost), data=new_position, parent=node)
                    self.visited.add(new_position)
                    self.visitedCost[new_position] = int(node.tag)
                    self.priorityQueue.put((self.manhattan_distance(new_position, exit1) + cost, new_position, new_node))

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
        # Maze x and y axises are flipped
        return not (position[0] < 0 or position[0] >= width or position[1] < 0 or position[1] >= height
                    or maze[position[0]][position[1]] >= 254)


runner = runner()