import math
import numpy as np

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def check_is_obstacle_in_boundary(center: tuple, width: float, length: float, heading: float, grid: np.ndarray,res: int) -> bool:
    '''
    Checks if an obstacle is in the boundary of the vehicle. Returns True if obstacle is found inside the boundary. Returns False otherwise.
    '''
    heading = np.deg2rad(heading)
    dist = math.sqrt(length/2 **2 + width/2 **2)

    o = np.deg2rad(180)
    a = np.arctan(width/length)

    angle_offsets = [a,-a,o+a,o-a]
    angles = []
    x = []
    z = []

    for i in range(len(angle_offsets)):
        angles.append(heading+angle_offsets[i])

        x.append(round((dist * np.sin(angles[i]) + center[0]),2)) # Rounded x-coordinate with 2 decimals
        z.append(round((dist * np.cos(angles[i]) + center[1]),2)) # Rounded z-coordinate with 2 decimals

    polygon = Polygon([(x[i],z[i]) for i in range(len(x))])

    ROWS, COLS = len(grid),len(grid[0])

    mid_row_id = (ROWS-1)//2
    mid_col_id = (COLS-1)//2

    min_row = round(-max(z) * res + mid_row_id)
    max_row = round(-min(z) * res + mid_row_id)

    min_col = round(min(x) * res + mid_col_id)
    max_col = round(max(x) * res + mid_col_id)


    for r in range(min_row,max_row+1):
        for c in range(min_col,max_col+1):
            if grid[r][c].any() != 0: # Obstacle
                point_x = (c - mid_col_id)/res
                point_z = -(r - mid_row_id)/res

                point = Point(point_x,point_z)
                
                if polygon.contains(point): return True

    return False


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

        self.width = 1
        self.length = 1

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

    def determine_neighbours(self,covered_grid: np.ndarray,step: int) -> np.ndarray:
        '''
        Determines the neighbours of the Node and returns the covered grid of Nodes.
        '''
        if self.row > 0: # A row above available 
            if self.grid[self.row-step][self.col].all() == 0 : # Node Straight Above available
                if covered_grid[self.row-step][self.col] == 0: 
                    x = self.x
                    z = self.z + 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=0,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row-step][self.col] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-step][self.col])

            if self.col < len(self.grid[0])-step and self.grid[self.row-step][self.col+step].all() == 0 : # Node Right Above available
                if covered_grid[self.row-step][self.col+step] == 0:  # No previous Node exists at this location
                    x = self.x + 1/self.res
                    z = self.z + 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=45,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row-step][self.col+step] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-step][self.col+step])

            if self.col > 0 and self.grid[self.row-step][self.col-step].all() == 0 : # Node Left Above available
                if covered_grid[self.row-step][self.col-step] == 0:  
                    x = self.x - 1/self.res
                    z = self.z + 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=-45,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row-step][self.col-step] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row-step][self.col-step])

        if self.row < len(self.grid)-step: # A row below available
            if self.grid[self.row+step][self.col].all() == 0 : # Node Straight Below available
                if covered_grid[self.row+step][self.col] == 0: 
                    x = self.x
                    z = self.z - 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=180,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row+step][self.col] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+step][self.col])

            if self.col < len(self.grid[0])-step and self.grid[self.row+step][self.col+step].all() == 0 : # Node Right Below available
                if covered_grid[self.row+step][self.col+step] == 0: 
                    x = self.x + 1/self.res
                    z = self.z - 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=135,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row+step][self.col+step] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+step][self.col+step])
       
            if self.col > 0 and self.grid[self.row+step][self.col-step].all() == 0 : # Node Left Below available
                if covered_grid[self.row+step][self.col-step] == 0: 
                    x = self.x - 1/self.res
                    z = self.z - 1/self.res
                    if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=225,grid=self.grid,res=self.res):
                        self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                        covered_grid[self.row+step][self.col-step] = self.neighbours[-1]
                else:
                    self.neighbours.append(covered_grid[self.row+step][self.col-step])


        if self.col < len(self.grid[0])-step and self.grid[self.row][self.col+step].all() == 0 : # Node Straight Right available
            if covered_grid[self.row][self.col+step] == 0:  
                x = self.x + 1/self.res
                z = self.z 
                if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=90,grid=self.grid,res=self.res):
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row][self.col+step] = self.neighbours[-1]
            else:
                self.neighbours.append(covered_grid[self.row][self.col+step])

        if self.col > 0 and self.grid[self.row][self.col-step].all() == 0 : # Node Straight Left available
            if covered_grid[self.row][self.col-step] == 0:  
                x = self.x - 1/self.res
                z = self.z 
                if not check_is_obstacle_in_boundary(center=(x,z),width=self.width,length=self.length,heading=270,grid=self.grid,res=self.res):
                    self.neighbours.append(Node(x,z,self.res,self.goal,self.grid))
                    covered_grid[self.row][self.col-step] = self.neighbours[-1]
            else:
                self.neighbours.append(covered_grid[self.row][self.col-step])

        return covered_grid     # Grid includes the Node objects

    def __lt__(self, other):    # Function required for defining the priority definition when using the heappush function
        return self.f < other.f