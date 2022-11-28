import numpy as np
import matplotlib.pyplot as plt
class node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end node
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node == end_node:
            # Complete here code to return the shortest path found
            path = []
            return path

        # Complete here code to generate children, which are the neighboring nodes. You should use 4 or 8 points connectivity for a grid.
        children = []

        # Check each child node of the coordinate, if its not blocked or out of bounds add it to 
        # list of children nodes
        # currently checks 4 nieghbors, TODO update to check 8
        for node in [cal_right(current_node), cal_down(current_node), cal_left(current_node), cal_up(current_node)]:
            if  not check_blocked(current_node) and not check_backrooms(current_node):
                children.append(current_node)

        # Loop through children to update the costs
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values, replace the 0s with appropriate formulations of the costs
                child.g = g_distanced_travelled(path)
                child.h = h_dist_to_goal(child, end_node.position)
                child.f = child.g + child.h

                # Complete here code to check whether to add a child to the open list
                open_list.append(child)

# A Star helper functions

        # To move to the right increment x coordinate only
        
        def cal_right(coord: list) -> list:
            return [coord[0] + 1, coord[1]]

        # To move to the left decrement x coordinate only

        def cal_left(coord) -> list:
            return [coord[0] - 1, coord[1]]

        # To move up increment y coordinate only

        def cal_up(coord) -> list:
            return [coord[0], coord[1] + 1]

        # To move down decrement y coordinate only

        def cal_down(coord) -> list:
            return [coord[0], coord[1] - 1]

    def check_blocked(self, potential_next_move) -> bool:
        if self.map[potential_next_move[1]][potential_next_move[0]] == 1:
            # code map is flipped i.e in terms of y then x ccordinates
            # print("Coordinate readings", potential_next_move[0], potential_next_move[1]  )
            # print("Map reading: ", self.map[potential_next_move[0]][potential_next_move[1]] )
            # print(potential_next_move , " is blocked!")
            return True
        else:
            return False

     # check to make sure we do not go out of bounds of map
    # out of bounds would be if either x or y coordinate of next move
    # is negative or > map length or map width
    def check_backrooms(self, potential_next_move, map) -> bool:
        ## try will only catch > len(map) and not negative numbers
        try:
            self.map[potential_next_move[1]][potential_next_move[0]]
        except:
            # print("Out of Bounds!")
            return True

        if potential_next_move[0] < 0 or potential_next_move[1] < 0  or potential_next_move[0] > len(map) or potential_next_move[1] > len(map):
            # print("Out of Bounds!")
            return True
        else:
            return False

   # define the evaluation functions here used for A * Search
   # calculated distance to goal
   # This currently uses manhattan distance to do replace with something else that
   # is better for 8 neighbours     

    def h_dist_to_goal(node, goal) -> int:
        # Using manhattan distance
        # Find manhattan distance estimate as difference between x and y
        # coordinates
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def g_distanced_travelled(self, path):
        return len(path)

def main():

    # Load your maze here
    maze = []
    
    # This is an example maze you can use for testing, replace it with loading the actual map
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

    
    
    # Define here your start and end points
    start = (0, 0)
    end = (6, 6)
    
    # Compute the path with your implementation of Astar
    path = np.asarray( astar(maze, start, end), dtype=np.float)
    maze_plot=np.transpose(np.nonzero(maze))

    plt.plot(maze_plot[:,0], maze_plot[:,1], 'o')
    
    if not np.any(path): # If path is empty, will be NaN, check if path is NaN
        print("No path found")
    else:
        plt.plot(path[:,0], path[:,1])
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()