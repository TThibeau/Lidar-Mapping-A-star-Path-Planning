from code.path_finding_functions import a_star,store_path_as_png,store_combined_map_path_as_png
import numpy as np
from PIL import Image

start = (0,0)
goal = (-5,21)
grid = np.asarray(Image.open("Out/main_grid_5res.png"))
RES = 5

nodes_path = a_star(start=start,goal=goal,grid=grid,res=RES)

store_path_as_png(path=nodes_path,grid=grid,output_name="Out/final_path")
store_combined_map_path_as_png(main_img="Out/main_grid_5res",path_img="Out/final_path")