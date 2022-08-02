import math

class Node:
    def __init__(self,x,z,res,prev,goal,grid) -> None:

        ROWS, COLS = len(grid),len(grid[0])

        mid_row_id = (ROWS-1)//2
        mid_col_id = (COLS-1)//2

        self.x = x
        self.z = z

        self.row = round(-z*res + mid_row_id)
        self.col = round(x*res + mid_col_id)

        self.res = res
        self.prev = prev
        self.goal = goal

        self.set_g_score(prev)
        self.set_h_score(goal)
        self.set_f_score()

        self.grid = grid
        self.neighbours = []

    def set_g_score(self,prev_node):
        if prev_node:
            self.g = prev_node.g + self.distance(prev_node)
        else:
            self.g = 0

    def set_h_score(self,goal):
        if goal:
            self.h = self.distance(goal)
        else:
            self.h = 0

    def set_f_score(self):
        self.f = self.g + self.h

    def distance(self,b):
        return math.sqrt(((self.x - b.x) ** 2) + ((self.z - b.z) ** 2))

    def determine_neighbours(self,covered_grid):
        # Above, Right, Below, Left
        if self.row > 0 and self.grid[self.row-1][self.col].all() == 0 and covered_grid[self.row-1][self.col].all() == 0: # Above available
            above_neigbour = Node(self.x,self.z+1/self.res,self.res,self,self.goal,self.grid)
            self.neighbours.append(above_neigbour)
            covered_grid[self.row-1][self.col] = 1

        if self.row < len(self.grid)-1 and self.grid[self.row+1][self.col].all() == 0 and covered_grid[self.row+1][self.col].all() == 0: # Below available
            below_neigbour = Node(self.x,self.z-1/self.res,self.res,self,self.goal,self.grid)
            self.neighbours.append(below_neigbour)
            covered_grid[self.row+1][self.col] = 1

        if self.col < len(self.grid[0])-1 and self.grid[self.row][self.col+1].all() == 0 and covered_grid[self.row][self.col+1].all() == 0:  # Right available
            right_neigbour = Node(self.x+1/self.res,self.z,self.res,self,self.goal,self.grid)
            self.neighbours.append(right_neigbour)
            covered_grid[self.row][self.col+1] = 1

        if self.col > 0 and self.grid[self.row][self.col-1].all() == 0 and covered_grid[self.row][self.col-1].all() == 0:  # Left available
            left_neigbour = Node(self.x-1/self.res,self.z,self.res,self,self.goal,self.grid)
            self.neighbours.append(left_neigbour)
            covered_grid[self.row][self.col-1] = 1

        return covered_grid

    def __lt__(self, other): # Function required for using the heappush function
        return self.f < other.f