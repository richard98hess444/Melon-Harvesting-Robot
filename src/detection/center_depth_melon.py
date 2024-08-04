import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import cv2
import numpy as np
bridge = CvBridge()

hand_melon_topic = '/hand_position'
realsense_melon_topic = '/realsense_detect'
depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
rgb_image_topic = '/camera/color/image_raw' 

ddddepth = np.array([])
X = 640
Y = 480
camera_intrinsic = np.array([[383.8562493,    0.       ,  316.5216793 ],
                             [  0.       ,  384.466263 ,  253.65872331],
                             [  0.       ,    0.       ,    1.        ]])

class BoundingBoxSubscriber():
    def __init__(self):  
        self.sub = rospy.Subscriber(realsense_melon_topic, Twist, self.bBoxCallBack)
        self.startPoint = (int(X/2), int(Y/2))
        self.endPoint = (int(X/2+50), int(Y/2+50))
        self.confidence = 0.0
        self.triggered = False

    def bBoxCallBack(self, data):
        self.startPoint = (int(data.linear.x), int(data.linear.y))
        self.endPoint = (int(data.angular.x), int(data.angular.y))
        self.confidence = data.angular.z
        self.triggered = True


class ImageColorSubscriber():
    def __init__(self):  
        self.sub = rospy.Subscriber(rgb_image_topic, Image, self.imgColorCallback)
        self.img_color = None
        self.triggered = False

    def imgColorCallback(self, data):
        self.img_color = data
        self.triggered = True


class ImageDepthSubscriber():
    def __init__(self):  
        self.sub = rospy.Subscriber(depth_image_topic, Image, self.imgDepthCallback)
        self.img_depth = None
        self.triggered = False

    def imgDepthCallback(self, data):
        self.img_depth = data
        self.triggered = True


class HandPositionSubscriber():
    def __init__(self):  
        self.sub = rospy.Subscriber(hand_melon_topic, Point, self.handCallback)
        self.x = X/2
        self.y = Y/2
        self.triggered = False

    def handCallback(self, data):
        self.x = data.x
        self.y = data.y
        self.triggered = True


def imageColorCallback(data, start_point, end_point, confidence):
    try:
        x0_center = int((start_point[0] + end_point[0])*0.5)
        y0_center = int((start_point[1] + end_point[1])*0.5)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.circle(cv_image, (x0_center, y0_center), radius=5, color=(0, 0, 255), thickness=-1)
        cv_image = cv2.rectangle(cv_image, start_point, end_point, color=(255, 0, 0), thickness=5)
        cv_image = cv2.putText(cv_image, str(confidence), start_point, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
        # cv_image = cv2.line(cv_image, (0, int(249.7012481689453)), (640, int(249.7012481689453)), (0, 255, 0), thickness=2)
        # cv_image = cv2.line(cv_image, (int(321.3174133300781), 0), (int(321.3174133300781), 480), (0, 255, 0), thickness=2)
        cv_image = cv2.circle(cv_image, (int(321.3174133300781), int(249.7012481689453)), radius=40, color=(0, 255, 0), thickness=1)
        cv_image = cv2.circle(cv_image, (283, 246), radius=20, color=(0, 255, 255), thickness=2)
        ros_img = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub = rospy.Publisher("/realsense_img", Image, queue_size=20)
        pub.publish(ros_img)
    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def imageDepthCallback(data, start_point, end_point):
    try:
        # Average of the box size
        box_u = int((end_point[0] - start_point[0]))
        box_v = int((end_point[1] - start_point[1]))
        box_avg = int((box_u + box_v)*0.5)
        # s0
        x0_center = int((start_point[0] + end_point[0])*0.5)
        y0_center = int((start_point[1] + end_point[1])*0.5)
        # s1
        x1_center = int(x0_center + (np.sqrt(3)/4)*box_avg)
        y1_center = y0_center
        # s2
        x2_center = x0_center
        y2_center = int(y0_center - (np.sqrt(3)/4)*box_avg)
        # s3
        x3_center = int(x0_center - (np.sqrt(3)/4)*box_avg)
        y3_center = y0_center
        # s4
        x4_center = x0_center
        y4_center = int(y0_center + (np.sqrt(3)/4)*box_avg)

        x1_center, y1_center = checkImgSpace(x1_center, y1_center)
        x2_center, y2_center = checkImgSpace(x2_center, y2_center)
        x3_center, y3_center = checkImgSpace(x3_center, y3_center)
        x4_center, y4_center = checkImgSpace(x4_center, y4_center)

        # Get the depth of each features (s0 to s4)
        cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
        center_0 = [x0_center, y0_center, cv_image[y0_center, x0_center]]
        center_1 = [x1_center, y1_center, cv_image[y1_center, x1_center]]
        center_2 = [x2_center, y2_center, cv_image[y2_center, x2_center]]
        center_3 = [x3_center, y3_center, cv_image[y3_center, x3_center]]
        center_4 = [x4_center, y4_center, cv_image[y4_center, x4_center]]

        print(f'============================================================')
        print(f"s0 = np.array([{center_0[0]}, {center_0[1]}, {center_0[2]}])")
        print(f"s1 = np.array([{center_1[0]}, {center_1[1]}, {center_1[2]}])")
        print(f"s2 = np.array([{center_2[0]}, {center_2[1]}, {center_2[2]}])")
        print(f"s3 = np.array([{center_3[0]}, {center_3[1]}, {center_3[2]}])")
        print(f"s4 = np.array([{center_4[0]}, {center_4[1]}, {center_4[2]}])")

        melon_center, radius = centerCalculator(center_0, center_1, center_2, center_3, center_4)
        melon_data = np.append(melon_center, radius).tolist()
        melon_data.extend([x0_center, y0_center])

        pub = rospy.Publisher("rgbd_coordinate", Float32MultiArray, queue_size=20)
        center_msg = Float32MultiArray(data = melon_data)
        pub.publish(center_msg)
        rate = rospy.Rate(10)

        # global ddddepth
        # ddddepth = np.append(ddddepth, center_0[2])
        # np.save('/home/richa/notebook/jupyterenv/notebook/melon/depth.npy', ddddepth)

        rate.sleep()
    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def centerCalculator(s0, s1, s2, s3, s4):
    s = [s0, s1, s2, s3, s4]
    p = []
    p_norm = []
    radius = []

    for features in s:
        p.append(np.linalg.inv(camera_intrinsic) @ np.vstack((np.array([features[:2]]).reshape(2, -1), 1)) * features[2]/1000)

    for index in range(1,5):
        p_norm.append(p[0] - p[index])

    for r in p_norm:
        radius.append(np.linalg.norm(r))

    r_avg = sum(radius)/4
    pc = p[0] + r_avg * np.array([[0,0,1]]).T

    return pc.T[0], r_avg

def checkImgSpace(x, y):
    if any(np.array([x, y])>(639, 479)) or any(np.array([x, y])<(0, 0)):
        x, y = 480, 240
    return x, y

def main():
    bbox = BoundingBoxSubscriber()
    hand = HandPositionSubscriber()
    color = ImageColorSubscriber()
    depth = ImageDepthSubscriber()

    while (not bbox.triggered or
           not color.triggered or
           not depth.triggered):
        pass
    
    while not rospy.is_shutdown():
        point_start = bbox.startPoint
        point_end = bbox.endPoint
        confidence = bbox.confidence
        data_color = color.img_color
        data_depth = depth.img_depth

        if hand.triggered:
            if hand.x == -1:
                hand.triggered = False
            point_start = (int(hand.x-50), int(hand.y-50))
            point_end = (int(hand.x+50), int(hand.y+50))

        imageColorCallback(data_color, point_start, point_end, confidence)
        imageDepthCallback(data_depth, point_start, point_end)
    

if __name__ == '__main__':
    rospy.init_node("center_depth_node")
    main()

# region [old code]
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# import cv2
# import numpy as np
# bridge = CvBridge()

# X = 640
# Y = 480
# start_point = (0,0)
# end_point = (0,0)
# confidence = 0

# def boundingBoxCallBack(data):
#     global start_point
#     global end_point
#     global confidence
#     start_point = (int(data.linear.x), int(data.linear.y))
#     end_point = (int(data.angular.x), int(data.angular.y))
#     confidence = data.angular.z


# def imageColorCallback(data):
#     try:
#         x0_center = int((start_point[0] + end_point[0])*0.5)
#         y0_center = int((start_point[1] + end_point[1])*0.5)
#         cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#         cv_image = cv2.circle(cv_image, (x0_center, y0_center), radius=10, color=(0, 0, 255), thickness=-1)
#         cv_image = cv2.rectangle(cv_image, start_point, end_point, color=(255, 0, 0), thickness=5)
#         cv_image = cv2.putText(cv_image, str(confidence), start_point, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
#         ros_img = bridge.cv2_to_imgmsg(cv_image, "bgr8")
#         pub = rospy.Publisher("/realsense_img", Image, queue_size=20)
#         pub.publish(ros_img)
#     except CvBridgeError as e:
#         print(e)
#         return
#     except ValueError as e:
#         return
    

# def imageDepthCallback(data):
#     try:
#         x0_center = int((start_point[0] + end_point[0])*0.5)
#         y0_center = int((start_point[1] + end_point[1])*0.5)
#         cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
#         center = [x0_center, y0_center, cv_image[int(y0_center), int(x0_center)]]
#         print("Coordinate:(%d, %d, %d)" %(center[0], center[1], center[2]))
#         pub = rospy.Publisher("rgbd_coordinate", Float32MultiArray, queue_size=20)
#         center_msg = Float32MultiArray(data = center)
#         pub.publish(center_msg)
#         rate = rospy.Rate(10)
#         rate.sleep()

#     except CvBridgeError as e:
#         print(e)
#         return
#     except ValueError as e:
#         return

# def main():
#     realsense_melon_topic = '/realsense_detect'
#     depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
#     rgb_image_topic = '/camera/color/image_raw' 

#     rospy.Subscriber(realsense_melon_topic, Twist, boundingBoxCallBack)
#     rospy.Subscriber(depth_image_topic, Image, imageDepthCallback)
#     rospy.Subscriber(rgb_image_topic, Image, imageColorCallback)
    
#     rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node("center_depth_node")
#     main()
# endregion [old code]