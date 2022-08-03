import math
import numpy as np

class Node:
    def __init__(self,x,z,res,goal,grid) -> None:
        ROWS, COLS = len(grid),len(grid[0])

        mid_row_id = (ROWS-1)//2
        mid_col_id = (COLS-1)//2

        self.x = x
        self.z = z

        self.row = round(-z*res + mid_row_id)
        self.col = round(x*res + mid_col_id)

        self.res = res
        self.goal = goal

        self.set_h_score(goal)

        self.g = math.inf
        self.f = math.inf

        self.grid = grid
        self.neighbours = []

        self.came_from = None

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

    def determine_neighbours(self,covered_grid: np.ndarray) -> np.ndarray:
        '''
        Determines the neighbours of the Node and returns the covered grid of Nodes.
        '''
        if self.row > 0: # A row above available 
            if self.grid[self.row-1][self.col].all() == 0 : # Node Straight Above available
                if covered_grid[self.row-1][self.col] == 0: 
                    x = self.x
                    z = self.z + 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row-1][self.col] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-1][self.col])

            if self.col < len(self.grid[0])-1 and self.grid[self.row-1][self.col+1].all() == 0 : # Node Right Above available
                if covered_grid[self.row-1][self.col+1] == 0:  # No previous Node exists at this location
                    x = self.x + 1/self.res
                    z = self.z + 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row-1][self.col+1] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-1][self.col+1])

            if self.col > 0 and self.grid[self.row-1][self.col-1].all() == 0 : # Node Left Above available
                if covered_grid[self.row-1][self.col-1] == 0:  
                    x = self.x - 1/self.res
                    z = self.z + 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row-1][self.col-1] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-1][self.col-1])

        if self.row < len(self.grid)-1: # A row below available
            if self.grid[self.row+1][self.col].all() == 0 : # Node Straight Below available
                if covered_grid[self.row+1][self.col] == 0: 
                    x = self.x
                    z = self.z - 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row+1][self.col] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+1][self.col])

            if self.col < len(self.grid[0])-1 and self.grid[self.row+1][self.col+1].all() == 0 : # Node Right Below available
                if covered_grid[self.row+1][self.col+1] == 0: 
                    x = self.x + 1/self.res
                    z = self.z - 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row+1][self.col+1] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+1][self.col+1])
       
            if self.col > 0 and self.grid[self.row+1][self.col-1].all() == 0 : # Node Left Below available
                if covered_grid[self.row+1][self.col-1] == 0: 
                    x = self.x - 1/self.res
                    z = self.z - 1/self.res
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row+1][self.col-1] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+1][self.col-1])


        if self.col < len(self.grid[0])-1 and self.grid[self.row][self.col+1].all() == 0 : # Node Straight Right available
            if covered_grid[self.row][self.col+1] == 0:  
                x = self.x + 1/self.res
                z = self.z 
                self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                covered_grid[self.row][self.col+1] = self.neighbours[-1]
            else:
                self.neighbours.append(covered_grid[self.row][self.col+1])

        if self.col > 0 and self.grid[self.row][self.col-1].all() == 0 : # Node Straight Left available
            if covered_grid[self.row][self.col-1] == 0:  
                x = self.x - 1/self.res
                z = self.z 
                self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                covered_grid[self.row][self.col-1] = self.neighbours[-1]
            else:
                self.neighbours.append(covered_grid[self.row][self.col-1])

        return covered_grid     # Grid includes the Node objects

    def __lt__(self, other):    # Function required for defining the priority definition when using the heappush function
        return self.f < other.f