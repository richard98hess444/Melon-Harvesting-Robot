import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import ctypes
import struct
import numpy as np
import tf.transformations as tf
import copy
import time
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from datetime import datetime
from math import pi, tau, dist, fabs, cos, sin

np.set_printoptions(suppress=True)

filePath1 = '/home/richa/notebook/jupyterenv/notebook/point_cloud/data/pc2_'
filePath2 = '/home/richa/catkin_ws/src/melon/pcs_data/0'

def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z) 
    pose.append(pose_msg.orientation.x)
    pose.append(pose_msg.orientation.y)
    pose.append(pose_msg.orientation.z)
    pose.append(pose_msg.orientation.w)
    return pose

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Emulidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("greenhouse_demo", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "tmr_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def set_pose_goal(self, p, s=1):   
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = p[0]
        pose_goal.position.y = p[1]
        pose_goal.position.z = p[2]
        pose_goal.orientation.x = p[3]
        pose_goal.orientation.y = p[4]
        pose_goal.orientation.z = p[5]
        pose_goal.orientation.w = p[6]

        move_group.set_pose_target(pose_goal)
        move_group.set_max_velocity_scaling_factor(s)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def set_joint_state(self, a):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = a[0]/(180/pi)
        joint_goal[1] = a[1]/(180/pi)
        joint_goal[2] = a[2]/(180/pi)
        joint_goal[3] = a[3]/(180/pi)
        joint_goal[4] = a[4]/(180/pi)
        joint_goal[5] = a[5]/(180/pi)

        move_group.set_max_velocity_scaling_factor(1)
        plan = move_group.go(joint_goal, wait=True)
        move_group.stop()
        return plan

    def path_planning(self, scale=1):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= scale * 0.2
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z -= scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 10.0
        )

        self.move_group.execute(plan, wait=True)
        return plan, fraction

    def check_position(self):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        x = wpose.position.x
        y = wpose.position.y
        z = wpose.position.z
        a = wpose.orientation.x
        b = wpose.orientation.y
        c = wpose.orientation.z
        d = wpose.orientation.w
        
        position = np.array([x,y,z,a,b,c,d])
        return position
    
    def check_joint_state(self):
        move_group = self.move_group
        current_joints = move_group.get_current_joint_values()
        a0 = current_joints[0]*180/pi
        a1 = current_joints[1]*180/pi
        a2 = current_joints[2]*180/pi
        a3 = current_joints[3]*180/pi
        a4 = current_joints[4]*180/pi
        a5 = current_joints[5]*180/pi
        a = np.array([a0,a1,a2,a3,a4,a5])
        return a
    
    def add_obj(self, name, p, obj, r, box_size=(0.8, 0.001, 0.8)):
        self.box_name = name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.position.x = p[0]
        box_pose.pose.position.y = p[1]
        box_pose.pose.position.z = p[2]
        try:
            box_pose.pose.orientation.x = p[3]
            box_pose.pose.orientation.y = p[4]
            box_pose.pose.orientation.z = p[5]
            box_pose.pose.orientation.w = p[6]
        except:
            box_pose.pose.orientation.x = 0
            box_pose.pose.orientation.y = 0
            box_pose.pose.orientation.z = 0
            box_pose.pose.orientation.w = 1
        if obj == 'plane':
            self.scene.add_plane(self.box_name, box_pose, normal = (1, 0, 0), offset = 0.01)
        elif obj == 'sphere':
            self.scene.add_sphere(self.box_name, box_pose, radius = r)
        elif obj == 'box':
            self.scene.add_box(self.box_name, box_pose, size = box_size)
    
    def remove_box(self, name):
        self.scene.remove_world_object(name)

class PointCloudSubscriber(object):
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/rtabmap/cloud_map",
                                     PointCloud2,
                                     self.PointCloud, queue_size=20)
        self.data = np.array([])
        self.triggered = False

    def PointCloud(self, PointCloud2):
        pointCloudData = np.array([])
        gen = point_cloud2.read_points(PointCloud2, skip_nans=True)
        int_data = list(gen)
        for p in int_data:
            test = p[3] 
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            pointCloudData = np.append(pointCloudData, 
                                       np.array([p[0],p[1],p[2],r,g,b]))
        now = datetime.now()
        current_time = now.strftime("%y%m%d_%H%M%S")

        self.data = pointCloudData
        self.triggered = True


def heightCalculator(data):
    data = data.reshape(-1,6)

    # Get rid of oversize xyz
    position_threshold = [0.6, 0.2, 0.8]
    position_filter = np.array([])
    for i in data:
        if abs(i[0]-0.5) < position_threshold[0]:
            if abs(i[1]) < position_threshold[1]:
                if abs(i[2]) < position_threshold[2]:
                    position_filter = np.append(position_filter, i)
    position_filter = position_filter.reshape(-1,6)
    print('Position Filter', position_filter.shape)

    # Leave green color only
    color_threshold = [65, 60, 70]
    color_filter = np.array([])
    for i in position_filter:
        if i[3] < color_threshold[0]:
            if i[4] > color_threshold[1]:
                if i[5] < color_threshold[2]:
                    color_filter = np.append(color_filter, i)
    color_filter = color_filter.reshape(-1,6)
    print('Color Filter', color_filter.shape)

    z = color_filter.T[2]
    z_max_index = z.argmax()
    z_min_index = z.argmin()
    return z[z_max_index]-z[z_min_index] + 0.1, color_filter

def checkSteadyJoint(goal_state, thr=0.01):
    t_start = time.time()
    while(1):
        goal = np.sqrt(np.sum((tutorial.check_joint_state()-goal_state)**2))
        if abs(goal)<thr:
            break
        elif time.time()-t_start > 5:
            break

def heightPublisher(data):
    pub = rospy.Publisher('height_Publisher', Float32, queue_size=5)
    pub.publish(float(data))
    rospy.sleep(1)

# Classes
tutorial = MoveGroupPythonInterfaceTutorial()
pc2Sub = PointCloudSubscriber()

# Parameters of Camera
camera_intrinsic = np.array([[383.8562493,    0.       ,  316.5216793 ],
                             [  0.       ,  384.466263 ,  253.65872331],
                             [  0.       ,    0.       ,    1.        ]])

# Manipulator Joint States
home = np.array([[0, 0, 0, 0, 90, 0]]).T
ready1 = np.array([[0,0,90,0,90,0]]).T
ready2 = np.array([[0,0,90,-90,90,0]]).T
ready3 = np.array([[0,0,90,90,-90,0]]).T
ready4 = np.array([[0,0,90,0,-90,0]]).T
ready5 = np.array([[0,0,90,-40,90,0]]).T
ready6 = np.array([[0,45,95,-140,90,0]]).T
ready7 = np.array([[0,32,93,-125,90,0]]).T

exp = True

def main():
    # while not pc2Sub.triggered: pass
    tutorial.set_joint_state(ready5.T[0])
    checkSteadyJoint(ready5.T[0])
    try:
        # Initial motion
        for i in np.arange(-40,-81,-10):
            tutorial.set_joint_state([0,0,90,i,90,0])
            checkSteadyJoint([0,0,90,i,90,0])

        ## Stretch motion
        tutorial.set_joint_state(ready6.T[0])
        checkSteadyJoint(ready6.T[0])
        tutorial.set_joint_state(ready2.T[0])
        checkSteadyJoint(ready2.T[0])
        for i in np.arange(0,41,10):
            tutorial.set_joint_state([0,0,0,i,90,0])
            checkSteadyJoint([0,0,0,i,90,0])
        

        # Zig-zag motion 1
        # tutorial.set_joint_state(ready2.T[0])
        # checkSteadyJoint(ready2.T[0])
        # tutorial.set_joint_state([27.67571826, 4.04177816, 111.7662043, -115.83585725, 42.29832825, -0.03530475])
        # checkSteadyJoint([27.67571826, 4.04177816, 111.7662043, -115.83585725, 42.29832825, -0.03530475])
        # p1 = tutorial.check_position()
        # tutorial.set_joint_state([-28.04156183, 17.42127676, 69.82839884, -87.24132768, 138.38572672, 0.00095804])
        # checkSteadyJoint([-28.04156183, 17.42127676, 69.82839884, -87.24132768, 138.38572672, 0.00095804])
        # p2 = tutorial.check_position()
        # tutorial.set_joint_state([27.66427214, -1.35197706, 88.58671611, -87.33430909, 42.281375, -0.13319441])
        # checkSteadyJoint([27.66427214, -1.35197706, 88.58671611, -87.33430909, 42.281375, -0.13319441])
        # tutorial.set_joint_state([-28.03526034, 31.18878826, 19.27584214, -50.50850067, 138.31464917, 0.02919903])
        # checkSteadyJoint([-28.03526034, 31.18878826, 19.27584214, -50.50850067, 138.31464917, 0.02919903])
        
        # Zig-zag motion 2
        tutorial.set_joint_state(ready2.T[0])
        checkSteadyJoint(ready2.T[0])
        tutorial.set_joint_state([33.11364178, 23.40699288, 117.69877223, -140.10088422,  26.04733865, 1.64330036])
        checkSteadyJoint([33.11364178, 23.40699288, 117.69877223, -140.10088422,  26.04733865, 1.64330036])
        tutorial.set_joint_state([-13.3531655, 21.08556186, 64.90969442, -85.99295818, 138.03888169, -0.00453929])
        checkSteadyJoint([-13.3531655, 21.08556186, 64.90969442, -85.99295818, 138.03888169, -0.00453929])
        tutorial.set_joint_state([30.43399983, 0.52617244, 89.47296827, -89.99751439, 37.04393944, 0.00573058])
        checkSteadyJoint([30.43399983, 0.52617244, 89.47296827, -89.99751439, 37.04393944, 0.00573058])
        tutorial.set_joint_state([-14.01678752, 35.59521027, 4.37593632, -39.50252632, 123.47768215, -0.25935903])
        checkSteadyJoint([-14.01678752, 35.59521027, 4.37593632, -39.50252632, 123.47768215, -0.25935903])
        
        tutorial.set_joint_state(home.T[0])
        checkSteadyJoint(home.T[0])
        pc2Data = pc2Sub.data
        np.save(filePath1, pc2Data)
        h, color_filter = heightCalculator(pc2Data)
        if abs(h-1.5) > 0.2:
            h = 1.5438246
        print('hight:', h)
        
        for i in range(5):
            heightPublisher(h)
            rospy.sleep(0.5)
        
        x = color_filter.T[0]
        y = color_filter.T[1]
        z = color_filter.T[2]
        color = color_filter[:,3:]
        ax = plt.figure(figsize=(6,6)).add_subplot(projection='3d')
        ax.view_init(30, 225, 0)
        ax.set_title('Filtered Point Cloud:' + str(h), fontsize=15)
        ax.scatter(x, y, z, c=color/255.0, marker='o', s=10, label=' ')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
            
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    rospy.sleep(10)
    main()



# tutorial.set_pose_goal([0.5247, -0.12226+0.2, 0.5,  0.5,  0.5,  0.5,  0.5])
# q1 = tutorial.check_joint_state() + np.array([0,0,0,0,-20,0])
# tutorial.set_joint_state(q1)
# tutorial.set_pose_goal([0.5247, -0.12226-0.2, 0.68019, 0.4039, 0.58046, 0.58043, 0.40378])
# tutorial.set_pose_goal(p2 + np.array([0,0,0.2,0,0,0,0]))
# tutorial.set_pose_goal(p1 + np.array([0,0,0.2,0,0,0,0]))
# print(tutorial.check_joint_state())