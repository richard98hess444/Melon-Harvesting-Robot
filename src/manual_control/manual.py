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
        # print('position:', x, y, z)
        # print('rotation:', a, b, c, d)
        rot = Rotation.from_quat([round(a,decimal),round(b,decimal),
                                  round(c,decimal),round(d,decimal)])
        eul_out = rot.as_euler('zyx', degrees=True)

        # position = np.array([round(x,decimal), 
        #                      round(y,decimal), 
        #                      round(z,decimal), 
        #                      round(eul_out[0],decimal), 
        #                      round(eul_out[1],decimal), 
        #                      round(eul_out[2],decimal)])
        
        position = np.array([x,y,z,a,b,c,d])
        return position


class keyboardSubscriber():
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_tmr", Vector3, self.Callback)
        self.triggered = False

    def Callback(self, position):
        self.triggered = True
        self.x = position.x
        self.y = position.y
        self.z = position.z


def trans(rz, ry, rx, tx, ty, tz, degree = False):
    if(degree == True):
            rz = rz/180*np.pi
            ry = ry/180*np.pi
            rx = rx/180*np.pi

    def Rz(t):
        return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
    def Ry(t):
        return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
    def Rx(t):
        return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
    
    rotation_mtx = Rx(rx) @ Ry(ry) @ Rz(rz)

    translation_mtx = np.array([[tx, ty, tz]]).T

    tansformation_mtx = np.hstack((rotation_mtx, translation_mtx))
    tansformation_mtx = np.vstack((tansformation_mtx, [0, 0, 0, 1]))

    return tansformation_mtx


ready2 = np.array([0, 0, 90, -80, 90, 0])
scaling_factor = 0.02


def main():
    arm_record = np.array([])
    tutorial = MoveGroupPythonInterfaceTutorial()
    keyboard = keyboardSubscriber()
    tutorial.set_joint_state(ready2)
    rospy.sleep(0.1)
    try:
        while(not rospy.is_shutdown()):
            if (not keyboard.triggered):
                pass
            else:
                keyboard.triggered = False
                if (keyboard.x and keyboard.y and keyboard.z == 1010):
                    tutorial.set_joint_state(ready2)
                else:
                    x = keyboard.x * scaling_factor
                    y = keyboard.y * scaling_factor
                    z = keyboard.z * scaling_factor
                    postn_state = tutorial.check_position()
                    arm_record = np.append(arm_record, postn_state[:3])
                    postn_state += np.array([x, y, z, 0, 0, 0, 0])
                    print(postn_state[:3])
                    tutorial.set_pose_goal(postn_state)
            np.save('/home/richa/notebook/jupyterenv/notebook/melon/arm_record.npy', arm_record)
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

