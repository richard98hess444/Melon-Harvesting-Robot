from __future__ import print_function
from tkinter.tix import X_REGION
from six.moves import input
from geometry_msgs.msg import Vector3
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import MotionPlanRequest
import numpy as np
from numpy import sin, cos
np.set_printoptions(suppress=True)
from scipy.spatial.transform import Rotation
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import math


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

x0 = 0.3
y0 = 0
z0 = 0.8
alpha0_rad = 45/(180/np.pi)

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
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
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
        rot = Rotation.from_euler('zyx', [p[5], p[4], p[3]], degrees=True)
        rot_quat = rot.as_quat().tolist()
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = p[3]
        pose_goal.orientation.y = p[4]
        pose_goal.orientation.z = p[5]
        pose_goal.orientation.w = p[6]
        pose_goal.position.x = p[0]
        pose_goal.position.y = p[1]
        pose_goal.position.z = p[2]

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
    
    def path_planning(self, scale=2):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        # wpose.position.x += scale * 0.1
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        # wpose.position.z -= scale * 0.2
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 10.0
        )

        self.move_group.execute(plan, wait=True)
        return plan, fraction

    def check_position(self):
        decimal = 3
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        x = wpose.position.x
        y = wpose.position.y
        z = wpose.position.z
        a = wpose.orientation.x
        b = wpose.orientation.y
        c = wpose.orientation.z
        d = wpose.orientation.w
        rot = Rotation.from_quat([round(a,decimal),round(b,decimal),
                                  round(c,decimal),round(d,decimal)])
        # eul_out = rot.as_euler('zyx', degrees=True)
        
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
    
    def add_box(self, name, p):
        self.box_name = name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.position.x = p[0]
        box_pose.pose.position.y = p[1]
        box_pose.pose.position.z = p[2] 
        box_pose.pose.orientation.x = p[3]
        box_pose.pose.orientation.y = p[4]
        box_pose.pose.orientation.z = p[5]
        box_pose.pose.orientation.w = p[6]
        self.scene.add_box(self.box_name, box_pose, size=(0.05, 0.05, 0.5))
        # scene.add_sphere(box_name, box_pose, radius = 0.1)
        # scene.add_plane(box_name, box_pose, normal = (0, 0, 1), offset = 0.5)

    
    def remove_box(self, name):
        self.scene.remove_world_object(name)


class keyboardSubscriber():
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_tmr", Vector3, self.Callback)
        self.triggered = False

    def Callback(self, position):
        self.triggered = True
        self.x = position.x
        self.y = position.y
        self.z = position.z


def tansformation(rot, tsl, is_qua = False, is_degree = True):
    
    if is_qua:
        agl = Rotation.from_quat(rot)
        euler = agl.as_euler('zyx', degrees=False)
    elif is_degree:
        euler = np.asarray(rot)
        euler = euler/180*np.pi
        
    def Rz(t):
        return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
    def Ry(t):
        return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
    def Rx(t):
        return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
    
    rotation_mtx = Rx(euler[2]) @ Ry(euler[1]) @ Rz(euler[0])
    translation_mtx = np.asarray(tsl).reshape((3, 1))
    tansformation_mtx = np.hstack((rotation_mtx, translation_mtx))
    tansformation_mtx = np.vstack((tansformation_mtx, [0, 0, 0, 1]))

    return tansformation_mtx

def tfmOprt(tfm, col_vec):
    return np.delete(tfm @ np.vstack((col_vec, [1])), -1, 0)

def tfmTotal(q):
    q = q.flatten()

    rot01 = [q[0], 0, 0]
    tsl01 = [0, 0, 0.1452]
    l0_T_l1 = tansformation(rot01, tsl01)
    # print(l0_T_l1)

    rot12 = [q[1]-90, 0, -90]
    tsl12 = [0, 0, 0]
    l1_T_l2 = tansformation(rot12, tsl12)
    # print(l1_T_l2)

    rot23 = [q[2], 0, 0]
    tsl23 = [0.429, 0, 0]
    l2_T_l3 = tansformation(rot23, tsl23)
    # print(l2_T_l3)

    rot34 = [q[3]+90, 0, 0]
    tsl34 = [0.4115, 0, -0.1223]
    l3_T_l4 = tansformation(rot34, tsl34)
    # print(l3_T_l4)

    rot45 = [q[4], 0, 90]
    tsl45 = [0, -0.106, 0]
    l4_T_l5 = tansformation(rot45, tsl45)
    # print(l4_T_l5)

    rot56 = [q[5], 0, 90]
    tsl56 = [0, -0.11315, 0]
    l5_T_l6 = tansformation(rot56, tsl56)
    # print(l5_T_l6)

    tfm = l0_T_l1 @ l1_T_l2 @ l2_T_l3 @ l3_T_l4 @ l4_T_l5 @ l5_T_l6
    return tfm

def forwardKinematics(tfm):
    x = tfm[0][3]
    y = tfm[1][3]
    z = tfm[2][3]
    alpha = np.arctan2(-tfm[1][2], tfm[2][2])
    beta = np.arcsin(tfm[0][2])
    gamma = np.arctan2(-tfm[0][1], tfm[0][0])
    return np.array([[x, y, z, alpha, beta, gamma]]).T

def robotJacobian(q):
    q = q.flatten()
    
    rot01 = [q[0], 0, 0]
    tsl01 = [0, 0, 0.1452]
    l0_T_l1 = tansformation(rot01, tsl01)

    rot12 = [q[1]-90, 0, -90]
    tsl12 = [0, 0, 0]
    l1_T_l2 = tansformation(rot12, tsl12)

    rot23 = [q[2], 0, 0]
    tsl23 = [0.429, 0, 0]
    l2_T_l3 = tansformation(rot23, tsl23)

    rot34 = [q[3]+90, 0, 0]
    tsl34 = [0.4115, 0, -0.1223]
    l3_T_l4 = tansformation(rot34, tsl34)

    rot45 = [q[4], 0, 90]
    tsl45 = [0, -0.106, 0]
    l4_T_l5 = tansformation(rot45, tsl45)

    rot56 = [q[5], 0, 90]
    tsl56 = [0, -0.11315, 0]
    l5_T_l6 = tansformation(rot56, tsl56)

    a01 = l0_T_l1
    a02 = a01 @ l1_T_l2
    a03 = a02 @ l2_T_l3
    a04 = a03 @ l3_T_l4
    a05 = a04 @ l4_T_l5
    a06 = a05 @ l5_T_l6

    def get_z_axis(arr):
        return np.array([[arr[0][2], arr[1][2], arr[2][2]]]).T
    def get_t_axis(arr):
        return np.array([[arr[0][3], arr[1][3], arr[2][3]]]).T
    
    z0 = np.array([[0,0,1]]).T
    z1 = get_z_axis(a01)
    z2 = get_z_axis(a02)
    z3 = get_z_axis(a03)

    t1 = get_t_axis(a01)
    t2 = get_t_axis(a02)
    t3 = get_t_axis(a03)
    t4 = get_t_axis(a04)
    t5 = get_t_axis(a05)
    t6 = get_t_axis(a06)

    j11 = np.cross(z0.flatten(), (t6-t0).flatten()).reshape(3,-1)
    j12 = np.cross(z1.flatten(), (t6-t1).flatten()).reshape(3,-1)
    j13 = np.cross(z2.flatten(), (t6-t2).flatten()).reshape(3,-1)
    j14 = np.cross(z3.flatten(), (t6-t3).flatten()).reshape(3,-1)
    j15 = np.cross(z4.flatten(), (t6-t4).flatten()).reshape(3,-1)
    j16 = np.cross(z5.flatten(), (t6-t5).flatten()).reshape(3,-1)

    j1 = np.hstack((j11, j12, j13, j14, j15, j16))
    j2 = np.hstack((z1, z2, z3, z4, z5, z6))
    j = np.vstack((j1, j2))

    return j

def checkSteadyJoint(tutorial, goal_state, thr=0.01):
    t_start = time.time()
    while(1):
        goal = np.sqrt(np.sum((tutorial.check_joint_state()-goal_state)**2))
        if abs(goal)<thr:
            break
        elif time.time()-t_start > 5:
            break

home = np.array([[0, 0, 0, 0, 0, 0]]).T
ready1 = np.array([[0,0,90,0,90,0]]).T
ready2 = np.array([[0,0,90,-90,90,0]]).T
ready3 = np.array([[0,0,90,90,-90,0]]).T
ready4 = np.array([[0,0,90,0,-90,0]]).T
joint_state_list = np.array([home, ready1, ready2, ready3, ready4])
ts = 1
p_dot = np.array([[0,0.1,0,0,0,0]]).T

def main():
    arm_record = np.array([])
    tutorial = MoveGroupPythonInterfaceTutorial()
    # tutorial.set_joint_state(ready2.T[0])
    # checkSteadyJoint(tutorial, ready2.T[0])
    tutorial.set_joint_state([53.62004089, 18.48831749, 67.52174377, -86.00888824, 36.38777924, -0.00166667])
    checkSteadyJoint(tutorial, [53.62004089, 18.48831749, 67.52174377, -86.00888824, 36.38777924, -0.00166667])
    try:
        # while(not rospy.is_shutdown()):
        #     p_now = tutorial.check_position()
        #     arm_record = np.append(arm_record, p_now)
        #     q_now = tutorial.check_joint_state()
        #     j = robotJacobian(q_now)
        #     try:
        #         q_dot = np.linalg.inv(j) @ p_dot
        #     except:
        #         q_dot = np.zeros(6).reshape(6,-1)
        #     tutorial.set_joint_state(q_now + q_dot.T[0]*ts)
        #     rospy.sleep(0.1)
        # np.save('/home/richa/notebook/jupyterenv/notebook/melon/jacobian.npy', arm_record)

            # print('add box')
            # x = float(input())
            # for i in np.linspace(-1,1,11):
            #     tutorial.add_box(str(i), p=[-0.3,i,0.5,0,0,0,1])
            
            # print('remove box')
            # x = float(input())
            # for i in np.linspace(-1,1,11):
            #     tutorial.remove_box(str(i))

        tutorial.path_planning(scale = -10)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

