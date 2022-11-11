from code.mapping_functions import measurement_to_point_cloud,create_main_grid,point_cloud_to_grid, save_grid,grid_to_img
from code.lidar import LidarConnection
from code.udp_connection import UDPConnection
from code.color_tuples import WHITE,BLUE
from time import sleep
from os import sys

ip = "127.0.0.1"
port_lidar = 54320
port_pos = 54321

lidar_1 = LidarConnection(ip,port_lidar)
udp_comms = UDPConnection(ip,port_pos) 

RES = 10 
WIDTH = 60
HEIGHT = 60

main_grid = create_main_grid(resolution=RES,width=WIDTH,height=HEIGHT)
print(f"Size of grid: {len(main_grid)} x {len(main_grid[0])}")
try:

    while True:
        ## Method:
        ## Take the measurement (angle,distance)
        ## Turn this into a x,z coordinates point cloud
        ## Add the point cloud to a global grid
        ## Additionally:
            ## Save the grid as txt
            ## Save the grid as png
        

        measurement = lidar_1.recieve_lidar_dist()
        x_veh,z_veh,heading = udp_comms.get_vehicle_info() 

        point_cloud = measurement_to_point_cloud(angle_dist=measurement
                                                ,x_veh=x_veh
                                                ,z_veh=z_veh,
                                                veh_heading=heading)


        main_grid = point_cloud_to_grid(pc=point_cloud,grid=main_grid,res=RES)

        # save_grid(main_grid,name="main_grid")
        grid_to_img(main_grid,name="main_grid",rgb_tuple=255)

except KeyboardInterrupt:
    print('Interrupted')
    grid_to_img(main_grid,name="main_grid",rgb_tuple=255)
    sys.exit(0)
# TODO: 
# - Add some sort of decay or something that removes the false positives