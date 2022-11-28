import random

class Node:
    def __init__(self, x, y):
        self.pos = [x,y]
        self.connections = []

class MapNode:
    def __init__(self):
        self.map = []
        self.mapSize = 100
        self.nodeAmount = 1000
        self.nodes = []
        self.testGenMap()

    def testGenMap(self):
        for y in range(0, self.mapSize):
            self.map.append([])
            for x in range(0, self.mapSize):
                self.map[y].append(0)

    def generateNodes(self):
        for i in range(0, self.nodeAmount):
            node = Node(random.randint(0, self.mapSize),random.randint(0, self.mapSize))
            self.nodes.append(node)

    def connectNodes(self):
        for baseNode in self.nodes:
            for connectNode in self.nodes:

