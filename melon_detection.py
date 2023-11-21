import torch
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from models.common import DetectMultiBackend
from utils.general import (Profile, check_img_size, non_max_suppression, scale_boxes)

bridge = CvBridge()

# Load model
weights='/home/richa/ntu-melon/yolov5/yolo_melon.pt'
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = DetectMultiBackend(weights, device=device, dnn=False, fp16=False)
stride, names, pt = model.stride, model.names, model.pt
imgsz = check_img_size((640, 640), s=stride)  # check image size
model.warmup(imgsz=(1 if pt else 1, 3, *imgsz))  # warmup
dt = (Profile(), Profile(), Profile())


def camera_detection(img0, model, names):
    im = img0.transpose((2, 0, 1))[::-1]
    conf_thres=0.25  # confidence threshold
    iou_thres=0.45  # NMS IOU threshold
    hide_labels=False  # hide labels
    hide_conf=False  # hide confidences
    mydata = None, None
    with dt[0]:
        im = torch.from_numpy(im.copy()).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim    
    # Inference
    with dt[1]:
        pred, proto = model(im, augment=False, visualize=False)[:2]
    # NMS
    with dt[2]:
        pred = non_max_suppression(pred, conf_thres, iou_thres, None, False, max_det=1000, nm=32)
    # Process predictions
    for i, det in enumerate(pred):  # per image
        im0 = img0.copy()
        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size
            for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                c = int(cls)  # integer class
                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                mydata = (xyxy, label)
    
    return mydata


def camera_talker(cv2_img):
    pub = rospy.Publisher('realsense_detect', Twist, queue_size=10)
    rate = rospy.Rate(10)
    melon = Twist()
    xyxy, label = camera_detection(cv2_img, model, names)
    if xyxy != None and label != None:
        melon.linear.x = int(xyxy[0].item())
        melon.linear.y = int(xyxy[1].item())
        melon.angular.x = int(xyxy[2].item())
        melon.angular.y = int(xyxy[3].item())
        melon.angular.z = float(label.split(' ')[-1])
        pub.publish(melon)
        print('yes melon')
        rate.sleep()
    else:
        print('no object detected')

def image_callback(data):
    try:
        cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        camera_talker(cv2_img)
    except CvBridgeError:
        print('error')

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rospy.init_node('realsense_detector', anonymous=False)
            rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

