import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import ctypes
import struct
import numpy as np
from datetime import datetime

filePath1 = '/home/richa/notebook/jupyterenv/notebook/point_cloud/data/pc2_'
filePath2 = '/home/richa/catkin_ws/src/melon/pcs_data/0'

exp = True

class PointCloudSubscriber(object):
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/rtabmap/cloud_map",
                                     PointCloud2,
                                     self.PointCloud, queue_size=20)

    def PointCloud(self, PointCloud2):
        pointCloudData = np.array([])
        gen = point_cloud2.read_points(PointCloud2, skip_nans=True)
        int_data = list(gen)
        print(len(int_data))
        for p in int_data:
            test = p[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            pointCloudData = np.append(pointCloudData, 
                                       np.array([p[0],p[1],p[2],r,g,b]))
            # print (p[0],p[1],p[2],r,g,b)
            # prints r,g,b values in the 0-255 range
            # x,y,z can be retrieved from the x[0],x[1],x[2]
        now = datetime.now()
        current_time = now.strftime("%y%m%d_%H%M%S")

        if exp:
            fileName = filePath2
            np.save(fileName, pointCloudData)

        fileName = filePath1 + current_time
        np.save(fileName, pointCloudData)
        print('saved')

if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    PointCloudSubscriber()
    rospy.spin()