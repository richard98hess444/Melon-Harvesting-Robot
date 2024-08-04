import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import tf.transformations as tf
import copy

from std_msgs.msg import Float32MultiArray
from math import pi, tau, dist, fabs, cos, sin

np.set_printoptions(suppress=True)


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
        rospy.init_node("move_group_python_interface", anonymous=True)

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



class MelonSubscriber():
    def __init__(self):
        self.sub = rospy.Subscriber("/rgbd_coordinate", Float32MultiArray, self.CofCallBack)
        self.mx = -0.1 
        self.my = 0.1 
        self.mz = 0.508 
        self.mr = 0.06
        self.mu = 242
        self.mv = 264
        self.triggered = False

    def CofCallBack(self, data):
        self.mx = data.data[0]
        self.my = data.data[1]
        self.mz = data.data[2]
        self.mr = data.data[3]
        self.mu = data.data[4]
        self.mv = data.data[5]
        self.triggered = True

class TechmanTransformation():
    def __init__(self) -> None:
        pass

    def tfmMtx(self, rot, tsl, degree = True):
        if not isinstance(rot, np.ndarray):
            rot = np.array(rot)
        if(degree == True):
            rot = rot/180*np.pi

        def Rz(t):
            return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
        def Ry(t):
            return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
        def Rx(t):
            return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
        
        rotation_mtx = Rz(rot[0]) @ Ry(rot[1]) @ Rx(rot[2])
        translation_mtx = np.array([tsl]).T

        tansformation_mtx = np.hstack((rotation_mtx, translation_mtx))
        tansformation_mtx = np.vstack((tansformation_mtx, [0, 0, 0, 1]))

        return tansformation_mtx

    def rotMtx(self, y, p, r):    
        def Rz(t):
            return np.array([[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]])
        def Ry(t):
            return np.array([[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]])
        def Rx(t):
            return np.array([[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]])
        return Rz(y) @ Ry(p) @ Rx(r)

    def rotMtx_big(self, y, p, r):    
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

    def tfmOprt(self, tfm, col_vec):
        return np.delete(tfm @ np.vstack((col_vec, [1])), -1, 0)

    def tfmTotal(self, q):
        q = q.flatten()

        rot01 = [q[0], 0, 0]
        tsl01 = [0, 0, 0.1452]
        l0_T_l1 = self.tfmMtx(rot01, tsl01)
        # print(l0_T_l1)

        rot12 = [q[1]-90, 0, -90]
        tsl12 = [0, 0, 0]
        l1_T_l2 = self.tfmMtx(rot12, tsl12)
        # print(l1_T_l2)

        rot23 = [q[2], 0, 0]
        tsl23 = [0.429, 0, 0]
        l2_T_l3 = self.tfmMtx(rot23, tsl23)
        # print(l2_T_l3)

        rot34 = [q[3]+90, 0, 0]
        tsl34 = [0.4115, 0, -0.1223]
        l3_T_l4 = self.tfmMtx(rot34, tsl34)
        # print(l3_T_l4)

        rot45 = [q[4], 0, 90]
        tsl45 = [0, -0.106, 0]
        l4_T_l5 = self.tfmMtx(rot45, tsl45)
        # print(l4_T_l5)

        rot56 = [q[5], 0, 90]
        tsl56 = [0, -0.11315, 0]
        l5_T_l6 = self.tfmMtx(rot56, tsl56)
        # print(l5_T_l6)

        tfm = l0_T_l1 @ l1_T_l2 @ l2_T_l3 @ l3_T_l4 @ l4_T_l5 @ l5_T_l6
        return tfm

    def forwardKinematics(self, tfm):
        x = tfm[0][3]
        y = tfm[1][3]
        z = tfm[2][3]
        alpha = np.arctan2(-tfm[1][2], tfm[2][2])
        beta = np.arcsin(tfm[0][2])
        gamma = np.arctan2(-tfm[0][1], tfm[0][0])
        return np.array([[x, y, z, alpha, beta, gamma]]).T

    def robotJacobian(self, q):
        q = q.flatten()
        
        rot01 = [q[0], 0, 0]
        tsl01 = [0, 0, 0.1452]
        l0_T_l1 = self.tfmMtx(rot01, tsl01)

        rot12 = [q[1]-90, 0, -90]
        tsl12 = [0, 0, 0]
        l1_T_l2 = self.tfmMtx(rot12, tsl12)

        rot23 = [q[2], 0, 0]
        tsl23 = [0.429, 0, 0]
        l2_T_l3 = self.tfmMtx(rot23, tsl23)

        rot34 = [q[3]+90, 0, 0]
        tsl34 = [0.4115, 0, -0.1223]
        l3_T_l4 = self.tfmMtx(rot34, tsl34)

        rot45 = [q[4], 0, 90]
        tsl45 = [0, -0.106, 0]
        l4_T_l5 = self.tfmMtx(rot45, tsl45)

        rot56 = [q[5], 0, 90]
        tsl56 = [0, -0.11315, 0]
        l5_T_l6 = self.tfmMtx(rot56, tsl56)

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
        z4 = get_z_axis(a04)
        z5 = get_z_axis(a05)
        z6 = get_z_axis(a06)

        t0 = np.zeros(3).reshape(3,-1)
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

    def poseMtx(self, pose):
        if not isinstance(pose, np.ndarray):
            pose = np.array(pose)
        pose = pose.flatten()
        rot = tf.quaternion_matrix(pose[3:])[:-1,:-1]
        trl = np.array([pose[:3]]).T
        return self.stack_tfm(rot, trl)

    def stack_tfm(self, rot, tsl):
        if not isinstance(rot, np.ndarray) or not isinstance(tsl, np.ndarray):
            print('invalid type')
            return None
        elif rot.shape != (3,3) or tsl.shape != (3,1):
            print('invalid shape')
            return None
        else:
            return np.vstack((np.hstack((rot, tsl)), [0, 0, 0, 1]))

def rad(degree):
    return degree/180*pi

def homoColumn(p):
    if not isinstance(p, np.ndarray):
        print('invalid type')
        return None
    elif p.shape != (3,1):
        print('invalid shape')
        return None
    else:
        return np.vstack((p, 1))

def homoColumnInv(p):
    if not isinstance(p, np.ndarray):
        print('invalid type')
        return None
    elif p.shape != (4,1):
        print('invalid shape')
        return None
    else:
        return p[:-1]

def checkMelonSteady(melonData):
    data = np.array([])
    radius = np.array([])
    steady = False
    while(not steady):
        while(len(data)<30):
            if melon.mz != 0 and melon.mz<1.2 and melon.mr<0.1:
                u = melonData.mu
                v = melonData.mv
                z = melonData.mz
                for i in [u,v,z]: data = np.append(data, i)
                radius  = np.append(radius, melon.mr)
                rospy.sleep(0.05)
        a = data.reshape(-1, 3).T[0]
        b = data.reshape(-1, 3).T[1]
        c = data.reshape(-1, 3).T[2]
        if (np.abs(np.mean(a)-a[-1])<1 and 
            np.abs(np.mean(b)-b[-1])<1 and
            np.abs(np.mean(c)-c[-1])<0.001):
            # print('ja steady')
            steady = True
            return steady, np.mean(radius)
        else:
            # print('no steady')
            data = np.delete(data, [0,1,2,3,4,5])

def createObject(tutorial, p_eef, p_melon):
    tutorial.add_obj('target', p_melon, 'sphere', melon.mr)
    tutorial.add_obj('plane_left', p_eef[:3] + np.array([0,0.2,0]), 'box', melon.mr)
    tutorial.add_obj('plane_back', 
                     [p_eef[0] -0.4, p_eef[1], p_eef[2], 0, 0, 0.7071, 0.7071], 
                     'box', melon.mr)
    tutorial.add_obj('plane_right', p_eef[:3] + np.array([0,-0.2,0]), 'box', melon.mr)

def firstPosition(pMelonB, radius, degree=70):
    dist = 0.5 + radius
    x_new = pMelonB[0] - dist*sin(degree/180*pi)
    y_new = pMelonB[1]
    z_new = pMelonB[2] + dist*cos(degree/180*pi)
    return np.array([[x_new, y_new, z_new]]).T

# Classes
tutorial = MoveGroupPythonInterfaceTutorial()
melon = MelonSubscriber()
tmtf = TechmanTransformation()

# Parameters of Camera
camera_intrinsic = np.array([[383.8562493,    0.       ,  316.5216793 ],
                             [  0.       ,  384.466263 ,  253.65872331],
                             [  0.       ,    0.       ,    1.        ]])

eefTcof = tmtf.poseMtx([-0.00818888, 0.00183305, 0.0401319, 
                        -0.00561637, -0.00193742, 0.999965, 0.00585165])

# Manipulator Joint States
home = np.array([[0, 0, 0, 0, 0, 0]]).T
ready1 = np.array([[0,0,90,0,90,0]]).T
ready2 = np.array([[0,0,90,-90,90,0]]).T
ready3 = np.array([[0,0,90,90,-90,0]]).T
ready4 = np.array([[0,0,90,0,-90,0]]).T
ready5 = np.array([[0,-30,60,-30,90,0]]).T
ready6 = np.array([[0,-20,20,20,90,0]]).T


def main():
    tutorial.set_joint_state(ready6.T[0])
    while (not melon.triggered): pass
    rospy.sleep(0.1)
    try:
        # while(not rospy.is_shutdown()):
        # tutorial.add_obj('plane_back', 
        #                 [-0.6, 0, 0.7, 0, 0, 0, 1], 
        #                 'box', melon.mr, box_size=(0.001, 0.8, 1.4))
        # Check if the melon is steady
        cameraSteady = False
        while(not cameraSteady): cameraSteady, melon_radius1 = checkMelonSteady(melon)
        print('steady camera')

        # Get the first Euclidean coordinate of the melon under camera frame
        p_melon_C1 = np.array([[melon.mx, melon.my, melon.mz]]).T
        print('Radius:', p_melon_C1)

        # Calculate the first world coordinate of the melon
        x_now = tutorial.check_position()
        baseTeef = tmtf.poseMtx(x_now)
        p_melon_B1 = baseTeef @ eefTcof @ homoColumn(p_melon_C1)
        p_melon_B1 = np.append(homoColumnInv(p_melon_B1), [0.5, 0.5, 0.5, 0.5])
        print('====Melon Info1:', p_melon_B1, melon_radius1)

        # Calculate the first position of the camera
        p_EIH_B1 = firstPosition(p_melon_B1, melon_radius1)

        # Calculate the first position of the EEF
        p_EEF_B1 = -homoColumnInv(tmtf.poseMtx(np.append([0,0,0], x_now[3:])) @ eefTcof @ np.array([[0,0,0,1]]).T) + p_EIH_B1
        p_EEF_B1 = np.append(p_EEF_B1, x_now[3:])
        print('====EndEF Info1:', p_EEF_B1)

        # Create a virtual object and move the maniulator
        tutorial.set_pose_goal(p_EEF_B1)
        print('end of the first part')
        print('press enter to continue')

        # Calculate the second world coordinate of the melon
        x = input()
        cameraSteady = False
        while(not cameraSteady): cameraSteady, melon_radius2 = checkMelonSteady(melon)
        p_melon_C2 = np.array([[melon.mx, melon.my, melon.mz]]).T
        print('Radius:', p_melon_C2)
        x_now = tutorial.check_position()
        baseTeef = tmtf.poseMtx(x_now)
        p_melon_B2 = baseTeef @ eefTcof @ homoColumn(p_melon_C2)
        p_melon_B2 = np.append(homoColumnInv(p_melon_B2), [0.5, 0.5, 0.5, 0.5])
        tutorial.add_obj('melon2', p_melon_B2, 'sphere', melon_radius2)
        print('====Melon Info2:', p_melon_B2, melon_radius2)

        # Calculate the second position of the EEF
        p_EEF_B2 = p_melon_B2 - np.array([melon_radius2*4,0,0,0,0,0,0])
        tutorial.set_pose_goal(p_EEF_B2)
        print('====EndEF Info2:', p_EEF_B2)

        print('end of the first part')
        print('press enter to continue')
        tutorial.remove_box('melon2')
        x = input()

        x_last = tutorial.check_position()
        tutorial.set_pose_goal(x_last + np.array([melon_radius2, 0, 1*melon_radius2, 0, 0, 0, 0]))
        print('====EndEF Info3:', x_last + np.array([melon_radius2, 0, 1*melon_radius2, 0, 0, 0, 0]))
        # tutorial.remove_box('plane_back')
        # tutorial.set_joint_state(ready6.T[0])

            

        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()


# Create walls that constraint the manipulator
# tutorial.add_obj('plane_left', 
#                  (x_now[:3] + p_melon_B[:3])*0.5 + np.array([0,0.25,0]), 
#                  'box', melon.mr, box_size=(0.8, 0.001, 1.6))
# tutorial.add_obj('plane_back', 
#                  [x_now[0] -0.4, x_now[1], x_now[2], 0, 0, 0.7071, 0.7071], 
#                  'box', melon.mr, box_size=(0.8, 0.001, 1.6))
# tutorial.add_obj('plane_right', 
#                  (x_now[:3] + p_melon_B[:3])*0.5 - np.array([0,0.25,0]), 
#                  'box', melon.mr, box_size=(0.8, 0.001, 1.6))

# tutorial.remove_box('plane_left')
# tutorial.remove_box('plane_back')
# tutorial.remove_box('plane_right')