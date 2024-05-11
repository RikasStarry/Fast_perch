import open3d as o3d
import numpy as np

point = o3d.io.read_point_cloud("/home/gnij/uneven_planner-main/src/uneven_planner/uneven_map/maps/newhill.pcd")

o3d.visualization.draw_geometries([point])
