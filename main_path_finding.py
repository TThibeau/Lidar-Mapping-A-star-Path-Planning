from path_finding_functions import a_star,store_path_as_png
import numpy as np
from PIL import Image

start = (0,0)
goal = (-5,21)
grid = np.asarray(Image.open("Out/main_grid.png"))
RES = 10

nodes_path = a_star(start=start,goal=goal,grid=grid,res=RES)

store_path_as_png(path=nodes_path,grid=grid,output_name="Out/nodes_path")