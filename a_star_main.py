from numpy import zeros,asarray,uint8
from node import Node
from heapq import heappush,heappop
from PIL import Image
from mapping_functions import grid_to_img

testgrid = asarray(Image.open('Out/main_grid.png'))

goalnode = Node(
    x=12,
    z=16,
    res=5,
    prev=None,
    goal=None,
    grid=testgrid
)
startnode = Node(
    x=0,
    z=0,
    res=5,
    prev=None,
    goal=goalnode,
    grid=testgrid
)

print(f"start: {startnode.row},{startnode.col},{startnode.x},{startnode.z}")
print(f"goal: {goalnode.row},{goalnode.col},{goalnode.x},{goalnode.z}")

current_node = startnode
heap = []
heappush(heap,(current_node.f,current_node)) # Push the startnode onto the heap
found = False

final_nodes = []
explored = zeros((len(testgrid),len(testgrid[0])),dtype='int8')
explored[current_node.row][current_node.col] = 1

while heap:
    current_node = heap[0] # Gets the highest priority object in the priority queue (highes priority == lowest f score)
    
    if current_node[1].row == goalnode.row and current_node[1].col == goalnode.col:
        found = True
        print("Goal node reached!")
        print(f"Total cost: {current_node[1].f}")

        # Reconstruct the path based on the previous nodes leading up to the goal node
        final_nodes.append(current_node[1])
        previous = current_node[1].prev

        while previous:
            final_nodes.append(previous)
            previous = previous.prev 

        path = zeros((len(testgrid),len(testgrid[0])),dtype='int8') # Initialize grid for visualizing the final path
            
        for node in set(final_nodes):
            path[node.row][node.col] = 1

        break
    
    heappop(heap) # Remove the current node from the heap

    explored = current_node[1].determine_neighbours(explored) # Grid for visualizing all the explored nodes

    for neighbour in current_node[1].neighbours:
        heappush(heap,(neighbour.f,neighbour))

if not found:
    print("Goal node not found")

else:
    pil_image = Image.fromarray((path*255).astype(uint8))
    pil_image.save(f'Out/final_path.png')

    main_grid_array = asarray(Image.open("Out/main_grid.png"))
    path_grid_array = asarray(Image.open("Out/final_path.png").convert('RGB'))
    new = path_grid_array*(1,0,0)+main_grid_array

    grid_to_img(new,"combined",rgb_tuple=(1,1,1))