# Yolo V5 with Melon Recognition & ROS

#### 1. Put the code along with the ```yolov5``` folder. The code should be at the same level with ```models```, ```utils```.

#### 2. The input is ```'/camera/color/image_raw'``` topic. The output is the start and end points of the bounding box, and the confidence of the target.

* data.linear.x = start point x
* data.linear.y = start point y
* data.angular.x = end point x
* data.angular.y = end point y
* data.angular.z = confidence
