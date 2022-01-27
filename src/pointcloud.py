import numpy as np
import open3d as o3d
import os
from datetime import datetime

"""This function converts the real space coordinates into the point cloud data format and visualizes the point cloud"""
def point_cloud_visualization(space_coordinates):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(space_coordinates)
    o3d.visualization.draw_geometries([pcd])

"""Write images to a file_path"""
def write_image(pcd):
    if isinstance(pcd, o3d.geometry.PointCloud):
        date_time = datetime.now()
        case_id =  date_time.strftime("%Y-%m-%d_%H-%M-%S")
        grasping_state_machine_path = os.path.abspath(os.path.join(__file__, *[os.pardir] * 2))
        pcd_data_path = os.path.join(grasping_state_machine_path, 'pcd_data')
        file_name = '%s_grasp_image.pcd' % case_id
        file_path = os.path.join(pcd_data_path, file_name)
        o3d.io.write_point_cloud(file_path, pcd)
    else:
        raise RuntimeError('Not Open3d Pointcloud Format')


