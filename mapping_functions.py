import numpy as np
from PIL import Image

def save_grid(grid,name):
    with open(f"Out\{name}.txt", "w") as file:
        np.savetxt(file, grid,fmt="%u")

def grid_to_img(img_grid,name):
    pil_image = Image.fromarray((img_grid * 255).astype(np.uint8))
    pil_image.save(f'Out\{name}.png')
    # pil_image.show()
    
def measurement_to_point_cloud(angle_dist,x_veh,z_veh,veh_heading)->list[(float,float)]:
    '''
    This function converts the measurements of a 2D (horizontal) Lidar in which each measurement consists of a list of tuples (angle,distance)
    and returns a point cloud (list of tuples with x,z coordinates)

    Input:  angle_dist = [(a_1,d_1),(a_2,d_2),...,(a_n,d_n)]
            x_veh = x-coordinate of vehicle
            z_veh = z-coordinate of vehicle
            veh_heading = heading of the vehicle in degrees (0 deg=N, 90 deg=E, 180 deg=S, 270 deg=W)

    Output: [(x1,z1),(x2,z2),...,(xn,zn)]
    '''
    point_cloud = []

    for i in range(len(angle_dist)):
        angle = np.deg2rad(angle_dist[i][0])+np.deg2rad(veh_heading)

        dist = angle_dist[i][1]

        x = round((dist * np.sin(angle) + x_veh),2) # Rounded x-coordinate with 2 decimals
        z = round((dist * np.cos(angle) + z_veh),2) # Rounded z-coordinate with 2 decimals

        point_cloud.append((x,z))
    
    return point_cloud

def create_main_grid(resolution,width,height):
    '''
    This function creates the initial main grid based on the required;
    resolution (grid points per meter)
    width (in meters)
    height (in meters)
    '''
    ROWS,COLS = height*resolution,width*resolution

    if ROWS % 2 == 0: 
        ROWS += 1 # The grid should have odd sizes (to have a mid row)
    if COLS % 2 == 0: 
        COLS += 1 # The grid should have odd sizes (to have a mid col)
    
    main_grid = np.zeros((ROWS,COLS))

    return main_grid

def point_cloud_to_grid(pc,grid,res):
    '''
    This function converts a point cloud (list of (x,y) tuples) and adds it
    to an existing grid with a certain res(olution) (grid points/meter)
    '''

    ROWS, COLS = len(grid),len(grid[0])

    mid_row_id = (ROWS-1)//2
    mid_col_id = (COLS-1)//2

    for p in pc:
        x = p[0]
        z = p[1]

        row = round(-z*res + mid_row_id)
        col = round(x*res + mid_col_id)

        try:
            grid[row][col] += 1
        except:
            print("this point is outside the global grid")

    return grid