
import numpy as np
from datetime import datetime
from pointcloud import write_image
from scipy.spatial.transform import Rotation as R
import open3d as o3d

def rawdata_to_pcd_file():
    img = np.loadtxt('/home/rosmatch/grasp_task_gpd/src/grasping_state_machine/rawdata/rawdata.txt')
    z_coordinates = []
    x_coordinates = []
    y_coordinates = []
    for l in range(len(img)):
        for e in range(len(img[l])):
            if img[l,e] != 0.0:
                z_coordinates.append((img[l,e] - 32768.0) * 0.0399 * 0.001)
                x_coordinates.append((e+1-1024) * 0.6159 * 0.001 * -1)
                y_coordinates.append((l+1-1024) * 0.6159 * 0.001)
    space_coordinates = [list(a) for a in zip(x_coordinates, y_coordinates, z_coordinates) if a[2] > 0.007]
    space_coordinates = np.array(space_coordinates)
    pcd = o3d.geometry.PointCloud()
    #coord = o3d.create_mesh_coordinate_frame(0.1, [0, 0, 0])
    #T = np.eye(4)
    #T[0, 0] = 0.34202014
    #T[0, 1] = 0.93969262
    #T[0, 2] = 0
    #T[0, 3] = -0.0876
    #T[1, 0] = -0.93969262
    #T[1, 1] = 0.34202014
    #T[1, 2] = 0
    #T[1, 3] = 0.3218
    #T[2, 0] = 0
    #T[2, 1] = 0
    #T[2, 2] = 1
    #T[2, 3] = 0
    #robo_link = copy.deepcopy(coord).transform(T)
    #r = R.from_euler('Z', -70, degrees=True)
    #print(r.as_matrix())
    #pcd = o3d.io.read_point_cloud("table_mug.pcd")
    #mesh_box = o3d.create_mesh_box(width=0.1,height=0.1,depth=0.1)
    pcd.points = o3d.utility.Vector3dVector(space_coordinates)
    #o3d.visualization.draw_geometries([pcd])
    write_image(pcd)
    o3d.visualization.draw_geometries([pcd])
