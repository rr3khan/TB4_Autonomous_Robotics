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
        self.findPath((0, 0),(160, 120))

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
        self.endPosition = endPos
        self.startPosition = startPos
        global SCREEN, CLOCK
        pygame.init()
        SCREEN = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        CLOCK = pygame.time.Clock()
        SCREEN.fill(BLACK)

        node = self.tree.create_node(tag='0', data=self.startPosition)
        self.a_star(self.startPosition, node)
        print(self.path)

        while True:
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