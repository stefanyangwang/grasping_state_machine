#from pointcloud import depth_to_space_coordinate, point_cloud_visualization
from harvesters.core import Harvester
import numpy as np
# import sys
# np.set_printoptions(threshold=sys.maxsize)
def policy():
    h = Harvester()
    h.add_file('/opt/mvIMPACT_Acquire/lib/x86_64/mvGenTLProducer.cti')
    h.update()
    ia = h.create_image_acquirer(0)
    #print(dir(ia.remote_device.node_map))
    #print(ia.remote_device.node_map.IntermediateDataValue.value)
    ia.remote_device.node_map.Width.value = 2048
    ia.remote_device.node_map.Height.value = 2048
    ia.remote_device.node_map.MultiCaptureImageType.value = 'FilteredImage1'
    ia.start_acquisition()
    with ia.fetch_buffer() as buffer:
        image = buffer.payload.components[0]
        _2d = image.data.reshape(image.height, image.width)
        # print(np.max(image.data))
        # print(_2d)
        mat = np.matrix(_2d)
        with open('/home/rosmatch/grasp_task_gpd/src/grasping_state_machine/rawdata/rawdata.txt','wb') as f:
            for line in mat:
                np.savetxt(f, line, fmt='%.2f')   
                                                                                                                                                                                                                                                                                                                                    
    ia.stop_acquisition()
    ia.destroy()
    h.reset()
    #space_coordinates = depth_to_space_coordinate('rawdata.txt')
    #print(space_coordinates)
    #point_cloud_visualization(space_coordinates)