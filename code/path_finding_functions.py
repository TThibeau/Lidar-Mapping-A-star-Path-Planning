from scripts.node import Node
import numpy as np
from heapq import heappush, heappop
from PIL import Image
import math
from mapping_functions import grid_to_img

def reconstruct_path(current: Node) -> list[Node]:
    '''
    Returns the path (as a list of nodes) leading up to the input Node. 
    '''
    path = [current]
    while current.came_from:
        current = current.came_from
        path.insert(0,current)

    return path

def check_node_in_heap(node: Node, heap: list[tuple]) -> bool:
    '''
    Returns True if the node is in the heap, otherwise returns False.
    '''
    for i in heap:
        if i[1]==node:
            return True
    return False

def store_path_as_png(path: list[Node], grid: np.ndarray, output_name: str) -> None:
    '''
    Stores the path of Nodes as a png image.
    '''
    map = np.zeros((len(grid),len(grid[0]),3),dtype='uint8')

    for node in path:
        map[node.row][node.col] = (255,0,0)
    
    pil_image = Image.fromarray((map).astype('uint8'))
    pil_image.save(f"{output_name}.png")
    print(f"{output_name}.png has been saved.")

def store_combined_map_path_as_png(main_img: str,path_img: str) -> None:
    '''
    Combines the 2 grids: main (Lidar-measurement-map) and path (A*-shortest-path) and stores them as a combined png
    '''
    main_grid_array = np.asarray(Image.open(f"{main_img}.png"))
    path_grid_array = np.asarray(Image.open(f"{path_img}.png").convert('RGB'))
    new = path_grid_array*(1,0,1)+main_grid_array*(0.4,0.4,0.4)
    grid_to_img(new,"combined",rgb_tuple=(1,1,1))

# def determine_valid_stepsize(start_node: Node, goal_node: Node, desired_stepsize: int) -> int:
#     '''
#     !UNUSED! 
#     Could be used to calculate stepsizes to reduce nodes in path
#     '''
#     r_diff = abs(start_node.row - goal_node.row)
#     c_diff = abs(start_node.col - goal_node.col)
#     if r_diff % desired_stepsize == 0 and c_diff % desired_stepsize == 0:
#         valid_stepsize = desired_stepsize
#     else:
#         valid_stepsize = math.gcd(r_diff,c_diff)

#     return valid_stepsize

def a_star(start: tuple , goal: tuple, grid: np.ndarray, res) -> list:
    '''
    Computes the shortest path from the start location to the goal.
    Input: 
        Start   (x,z) coordinates,
        Goal    (x,z) coordinates,
        Grid    numpy.ndarray with values 0 for "free" and values > 0 for "occupied"
        Res     int value for the resolution of the Grid. This resolution equals the amount of 
                points are included in the grid per meter distance

    Output:
        Path as a list of Nodes
    '''
    goal_node = Node(x=goal[0],z=goal[1],res=res,goal=None,grid=grid)
    start_node = Node(x=start[0],z=start[1],res=res,goal=goal_node,grid=grid)
    start_node.g = 0
    start_node.f = start_node.h

    print(f"start: {start_node.row},{start_node.col},{start_node.x},{start_node.z}")
    print(f"goal: {goal_node.row},{goal_node.col},{goal_node.x},{goal_node.z}")


    open_set = []   # The set of discovered nodes that may need to be (re-)expanded.
    heappush(open_set,(start_node.f,start_node))    # Push the start node onto the heap (open_set)

    explored = np.zeros((len(grid),len(grid[0])),dtype=Node)
    explored[start_node.row][start_node.col] = start_node   # Add the start node to the explored map/grid

    while open_set:
        current = open_set[0][1]    # Gets the highest priority object in the priority queue 
                                    # (highest priority = lowest f score)
    
        if current.row == goal_node.row and current.col == goal_node.col:   # If the location of the current Node equals the location of the goal node,
                                                                            # return the path leading up to this Node
            print("The goal node is reached!")
            print(f"Total cost: {current.f}")
            
            final_path = reconstruct_path(current)

            print(f"Length of the final path: {len(final_path)} nodes")
            return final_path

        heappop(open_set)  # Remove the current node from the heap

        # stepsize = determine_valid_stepsize(start_node=start_node,goal_node=goal_node,desired_stepsize=1) # Function can be used once the addition check for wall jumps is implemented
        
        current.determine_neighbours(explored,1) # Determine the neighbours of the current node, this populates current.neighbours

        for neighbour in current.neighbours:
            provisional_g = current.g + current.distance(neighbour)
            if provisional_g < neighbour.g:
                neighbour.came_from = current
                neighbour.g = provisional_g
                neighbour.f = provisional_g + neighbour.h

                if not check_node_in_heap(node=neighbour,heap=open_set):
                    heappush(open_set,(neighbour.f,neighbour))

    raise Exception("No valid path was found to the goal Node and all possible Nodes were explored.")