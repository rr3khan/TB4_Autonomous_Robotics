import sys
from Queue import PriorityQueue
from math import sqrt
from treelib import Tree

#runner = runner(310, 195, np.array(cost_map_data).reshape(-1, 310).tolist())

#In render y is given x and x is y
class Pathfinder:
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
        self.findPath((30, 20),(70, 70))

    def get_map_cords(self, x, y):
        return y,self.width-x

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

        #A*
        node = self.tree.create_node(tag='0', data=self.startPosition)
        self.a_star(self.startPosition, node)

        #assert(self.path == [(180, 120), (180, 121), (181, 121), (181, 122), (182, 122), (182, 123), (183, 123), (183, 124), (184, 124), (184, 125), (185, 125), (185, 126), (186, 126), (186, 127), (187, 127), (187, 128), (187, 129), (187, 130), (187, 131), (187, 132), (187, 133), (187, 134), (187, 135), (187, 136), (187, 137), (187, 138), (187, 139), (187, 140), (187, 141), (187, 142), (187, 143), (187, 144), (187, 145), (187, 146), (187, 147), (187, 148), (187, 149), (187, 150), (187, 151), (187, 152), (187, 153), (187, 154), (187, 155), (187, 156), (187, 157), (187, 158), (187, 159), (187, 160), (187, 161), (187, 162), (187, 163), (187, 164), (187, 165), (187, 166), (187, 167), (187, 168), (188, 168), (188, 169), (188, 170), (188, 171), (188, 172), (189, 172), (189, 173), (189, 174), (190, 174), (190, 175), (191, 175), (192, 175), (193, 175), (194, 175), (195, 175), (196, 175), (197, 175), (198, 175), (199, 175), (200, 175), (201, 175), (202, 175), (203, 175), (204, 175), (205, 175), (206, 175), (207, 175), (208, 175), (209, 175), (210, 175), (210, 174), (211, 174), (211, 173), (211, 172), (212, 172), (212, 171), (212, 170), (212, 169), (212, 168), (213, 168), (213, 167), (213, 166), (213, 165), (213, 164), (213, 163), (213, 162), (213, 161), (213, 160), (213, 159), (213, 158), (213, 157), (213, 156), (213, 155), (213, 154), (213, 153), (213, 152), (213, 151), (213, 150), (213, 149), (213, 148), (213, 147), (213, 146), (213, 145), (213, 144), (213, 143), (213, 142), (213, 141), (213, 140), (213, 139), (213, 138), (213, 137), (213, 136), (213, 135), (213, 134), (213, 133), (213, 132), (213, 131), (213, 130), (213, 129), (213, 128), (213, 127), (213, 126), (213, 125), (213, 124), (213, 123), (213, 122), (213, 121), (213, 120), (213, 119), (213, 118), (213, 117), (213, 116), (213, 115), (213, 114), (213, 113), (213, 112), (213, 111), (213, 110), (213, 109), (213, 108), (213, 107), (213, 106), (213, 105), (213, 104), (213, 103), (213, 102), (213, 101), (214, 101), (214, 100), (214, 99), (213, 99), (213, 98), (213, 97), (213, 96), (213, 95), (212, 95), (212, 94), (212, 93), (211, 93), (210, 93), (209, 93), (208, 93), (207, 93), (207, 92), (206, 92), (205, 92), (204, 92), (204, 91), (203, 91), (202, 91), (201, 91), (200, 91), (199, 91), (198, 91), (197, 91), (196, 91), (195, 91), (194, 91), (193, 91), (192, 91), (191, 91), (190, 91), (189, 91), (188, 91), (187, 91), (186, 91), (185, 91), (184, 91), (183, 91), (182, 91), (181, 91), (180, 91), (179, 91), (178, 91), (177, 91), (176, 91), (175, 91), (174, 91), (173, 91), (172, 91), (171, 91), (170, 91), (169, 91), (168, 91), (167, 91), (166, 91), (165, 91), (164, 91), (163, 91), (162, 91), (161, 91), (160, 91), (159, 91), (158, 91), (157, 91), (156, 91), (155, 91), (154, 91), (153, 91), (152, 91), (151, 91), (150, 91), (150, 92), (149, 92), (148, 92), (147, 92), (147, 93), (146, 93), (145, 93), (144, 93), (143, 93), (142, 93), (142, 94), (142, 95), (141, 95), (141, 96), (141, 97), (141, 98), (141, 99), (140, 99), (140, 100), (140, 101), (140, 102), (140, 103), (140, 104), (140, 105), (140, 106), (140, 107), (140, 108), (140, 109), (140, 110), (140, 111), (139, 111), (138, 111), (137, 111), (136, 111), (135, 111), (134, 111), (133, 111), (132, 111), (131, 111), (130, 111), (129, 111), (128, 111), (127, 111), (126, 111), (125, 111), (124, 111), (123, 111), (122, 111), (121, 111), (120, 111), (119, 111), (118, 111), (117, 111), (116, 111), (115, 111), (114, 111), (113, 111), (112, 111), (111, 111), (110, 111), (109, 111), (108, 111), (107, 111), (106, 111), (105, 111), (104, 111), (103, 111), (102, 111), (101, 111), (100, 111), (99, 111), (98, 111), (97, 111), (96, 111), (95, 111), (94, 111), (93, 111), (92, 111), (91, 111), (90, 111), (89, 111), (88, 111), (87, 111), (87, 112), (86, 112), (85, 112), (84, 112), (83, 112), (83, 113), (82, 113), (81, 113), (81, 114), (80, 114), (80, 115), (79, 115), (79, 116), (78, 116), (78, 117), (77, 117), (77, 118), (77, 119), (76, 119), (76, 120), (76, 121), (76, 122), (76, 123), (75, 123), (75, 124), (75, 125), (76, 125), (76, 126), (76, 127), (76, 128), (76, 129), (77, 129), (77, 130), (77, 131), (77, 132), (77, 133), (77, 134), (76, 134), (76, 135), (76, 136), (76, 137), (75, 137), (74, 137), (73, 137), (72, 137), (71, 137), (70, 137), (69, 137), (68, 137), (67, 137), (66, 137), (65, 137), (64, 137), (63, 137), (62, 137), (61, 137), (60, 137), (59, 137), (58, 137), (57, 137), (56, 137), (55, 137), (54, 137), (53, 137), (52, 137), (51, 137), (50, 137), (49, 137), (49, 136), (48, 136), (47, 136), (46, 136), (45, 136), (45, 135), (44, 135), (43, 135), (43, 134), (42, 134), (42, 133), (41, 133), (41, 132), (40, 132), (40, 131), (40, 130), (39, 130), (39, 129), (39, 128), (39, 127), (39, 126), (38, 126), (38, 125), (38, 124), (38, 123), (38, 122), (38, 121), (38, 120), (38, 119), (38, 118), (38, 117), (38, 116), (38, 115), (37, 115), (37, 114), (36, 114), (35, 114), (34, 114), (33, 114), (33, 113), (32, 113), (31, 113), (31, 112), (30, 112), (30, 111), (29, 111), (29, 110), (29, 109), (29, 108), (29, 107), (29, 106), (29, 105), (29, 104), (29, 103), (29, 102), (29, 101), (29, 100), (29, 99), (29, 98), (29, 97), (29, 96), (29, 95), (29, 94), (29, 93), (30, 93), (30, 92), (31, 92), (31, 91), (32, 91), (33, 91), (33, 90), (34, 90), (35, 90), (36, 90), (37, 90), (37, 89), (38, 89), (38, 88), (38, 87), (38, 86), (38, 85), (38, 84), (38, 83), (38, 82), (38, 81), (38, 80), (38, 79), (38, 78), (38, 77), (38, 76), (38, 75), (38, 74), (38, 73), (38, 72), (38, 71), (38, 70), (38, 69), (38, 68), (38, 67), (38, 66), (38, 65), (38, 64), (38, 63), (37, 63), (37, 62), (37, 61), (36, 61), (36, 60), (36, 59), (36, 58), (36, 57), (35, 57), (34, 57), (34, 56), (33, 56), (33, 55), (32, 55), (32, 54), (31, 54), (31, 53), (31, 52), (30, 52), (30, 51), (30, 50), (30, 49), (30, 48), (30, 47), (30, 46), (30, 45), (30, 44), (30, 43), (30, 42), (31, 42), (31, 41), (31, 40), (31, 39), (31, 38), (31, 37), (31, 36), (31, 35), (31, 34), (31, 33), (31, 32), (32, 32), (32, 31), (32, 30), (33, 30), (33, 29), (34, 29), (34, 28), (35, 28), (35, 27), (36, 27), (37, 27), (37, 26), (38, 26), (38, 25), (38, 24), (38, 23), (38, 22), (38, 21), (38, 20), (37, 20), (36, 20), (35, 20), (34, 20), (33, 20), (32, 20), (31, 20), (30, 20)])
        return self.path

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