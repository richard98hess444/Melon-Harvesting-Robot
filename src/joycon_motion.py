from __future__ import print_function
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from moveit_commander.conversions import pose_to_list
from numpy import sin, cos, pi
import tf.transformations as tf
import geometry_msgs.msg
import numpy as np
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
np.set_printoptions(suppress=True)

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

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

    def set_joint_state(self, a, degree=True):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        if(degree == True):
            a = a/(180/pi)

        joint_goal[0] = a[0]
        joint_goal[1] = a[1]
        joint_goal[2] = a[2]
        joint_goal[3] = a[3]
        joint_goal[4] = a[4]
        joint_goal[5] = a[5]

        plan = move_group.go(joint_goal, wait=True)
        move_group.stop()
        return plan

    def set_pose_goal(self, p, s):   
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

        rot = Rotation.from_quat([a, b, c, d])
        rot_euler = rot.as_euler('zyx', degrees=True).tolist()

        position = np.array([x, y, z, a, b, c, d])
        # print(position)
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
        print("angle:", a)
        return a


def trans(rz, ry, rx, tx, ty, tz, degree = True):
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
    
    rotation_mtx = Rz(rz) @ Ry(ry) @ Rx(rx)
    translation_mtx = np.array([[tx, ty, tz]]).T

    tansformation_mtx = np.hstack((rotation_mtx, translation_mtx))
    tansformation_mtx = np.vstack((tansformation_mtx, [0, 0, 0, 1]))

    return tansformation_mtx

def rotation(y, p, r):    
    def Rz(t):
        return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
    def Ry(t):
        return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
    def Rx(t):
        return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
    return Rz(y) @ Ry(p) @ Rx(r)

def big_rot_mtx(y, p, r):    
    def Rz(t):
        return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
    def Ry(t):
        return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
    def Rx(t):
        return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
    
    rot_mtx = Rz(y) @ Ry(p) @ Rx(r)
    big_rot_mtx = np.hstack((rot_mtx, np.zeros(3).reshape(3,1)))
    big_rot_mtx = np.vstack((big_rot_mtx, [0, 0, 0, 1]))
    return big_rot_mtx
    
def rad(degree):
    return degree/180*pi

def Rz(t):
    return np.array([[cos(t), -sin(t)], [sin(t), cos(t)]])

def heavisideFilter(a):
    return np.heaviside(np.abs(a)-500, 0) * a

def joyconStatus(joycon):
    rh = joycon.rh
    rv = joycon.rv
    index_R = int(joycon.ri)
    lh = joycon.lh
    lv = joycon.lv
    index_L = int(joycon.li)

    drh = heavisideFilter(rh-rh0)
    drv = heavisideFilter(rv-rv0)
    rb = name_R[index_R]

    dlh = heavisideFilter(lh-lh0)
    dlv = heavisideFilter(lv-lv0)
    lb = name_L[index_L]

    if drh or drv != 0:
        theta = np.arctan2(drv, drh)
        j = np.array([[k * np.cos(theta)], 
                      [k * np.sin(theta)]])
        pR = Rz(-np.pi/2*0) @ j
    else:
        pR = np.array([[0],[0]])
    
    if dlh or dlv != 0:
        theta = np.arctan2(dlv, dlh)
        j = np.array([[k * np.cos(theta)], 
                      [k * np.sin(theta)]])
        pL = Rz(-np.pi/2) @ j
    else:
        pL = np.array([[0],[0]])
    
    return pR.T[0], pL.T[0], rb, lb

class joycon_subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/joycon_msg", Twist, self.CofCallBack)
        self.rh = rh0
        self.rv = rv0
        self.ri = -1
        self.lh = lh0
        self.lv = lv0
        self.li = -1
        self.triggered = False

    def CofCallBack(self, data):
        self.rh = data.linear.x
        self.rv = data.linear.y
        self.ri = data.linear.z
        self.lh = data.angular.x
        self.lv = data.angular.y
        self.li = data.angular.z
        self.triggered = True



# Fixed Transformation Matrix
home = np.array([0,0,0,0,0,0])
ready2 = np.array([0, 0, 90, -90, 90, 0])
name_R = ['A', 'B', 'X', 'Y', 'R', 'ZR', 'PLUS', 'NONE_R']
name_L = ['UP', 'RIGHT', 'DOWN', 'LEFT', 'L', 'ZL', 'MINUS', 'NONE_L']
rh0 = 2133
rv0 = 1872
lh0 = 2080
lv0 = 2315
k = 0.02
sleepTime = 0.05

def main():
    # class
    tutorial = MoveGroupPythonInterfaceTutorial()
    joycon = joycon_subscriber()

    print('========================')
    print('|                      |')
    print('|    PROGRAM STARTS    |')
    print('|                      |')
    print('========================')
    rospy.sleep(sleepTime)

    try:
        tutorial.set_joint_state(ready2)
        while(not rospy.is_shutdown()):
            jsr, jsl, br, bl = joyconStatus(joycon)
            z = 0
            roll = 0
            pitch = 0
            yaw = 0

            if (np.sum(jsr) == 0 and np.sum(jsl) == 0 and br == 'NONE_R' and bl == 'NONE_L'):
                pass
            else:
                if br == 'A':
                    z = 0.02
                elif br == 'B':
                    z = -0.02
                elif br == 'R':
                    yaw = 10

                if bl == 'MINUS':
                    tutorial.set_joint_state(ready2)
                elif bl == 'L':
                    yaw = -10
                elif bl == 'UP':
                    roll = 10
                elif bl == 'DOWN':
                    roll = -10
                elif bl == 'RIGHT':
                    pitch = -10
                elif bl == 'LEFT':
                    pitch = 10

                s_now = tutorial.check_position()
                p_now = s_now[:3] + np.array([jsl[0] ,jsl[1], z])
                q_now = s_now[3:]
                R_now = tf.quaternion_matrix(q_now)
                R_d = big_rot_mtx(rad(yaw), rad(pitch), rad(roll))
                R_new = R_now @ R_d
                q_new = tf.quaternion_from_matrix(R_new)
                s_new = np.append(p_now, q_new)

                print('s_new', s_new)
                tutorial.set_pose_goal(s_new, 1)
            rospy.sleep(0.1)   
            
                    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

