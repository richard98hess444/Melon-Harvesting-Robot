#!/usr/bin/python
# -*- coding: utf-8 -*-

# 参考  https://blog.csdn.net/fengyu19930920/article/details/81144042

from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
import time
 
JOINT_NAMES = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

angle = np.linspace(0, np.pi/2, 100)
way_point = np.array([])
for a in angle:
    way_point = np.append(way_point,[0,0,0,0,a,0])
for a in angle[-2:0:-1]:
    way_point = np.append(way_point,[0,0,0,0,a,0])
way_point = way_point.reshape(-1,6)

# way_point = np.array([[0,0,0,0,0.0,0],
#                       [0,0,0,0,0.1,0],
#                       [0,0,0,0,0.2,0],
#                       [0,0,0,0,0.3,0],
#                       [0,0,0,0,0.4,0],
#                       [0,0,0,0,0.5,0]])

def trajectory():
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = JOINT_NAMES
    goal.trajectory.header.stamp = rospy.Time.now()

    goal.trajectory.points=[0]*4
    goal.trajectory.points[0]=JointTrajectoryPoint(positions=[0,0,0,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(0.0))
    goal.trajectory.points[1]=JointTrajectoryPoint(positions=[0,0,0,0,0,0.1], velocities=[0]*6,time_from_start=rospy.Duration(1.0))
    goal.trajectory.points[2]=JointTrajectoryPoint(positions=[0,0,0,0,0,0.2], velocities=[0]*6,time_from_start=rospy.Duration(2.0))
    goal.trajectory.points[3]=JointTrajectoryPoint(positions=[0,0,0,0,0,0.3], velocities=[0]*6,time_from_start=rospy.Duration(3.0))

    pub = rospy.Publisher('joint_states', FollowJointTrajectoryGoal, queue_size=10)
    pub.publish(goal)
    rospy.sleep(0.02)

def jointState():
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = JOINT_NAMES
    
    for w in way_point:
        print(w)
        js.position = w
        js.velocity = [0,0,0,0,0.1,0]
        pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        pub_joint_state.publish(js)
        rospy.sleep(0.1)
 
def joint_state_publisher():
    rospy.init_node("joint_cmd")
    while not rospy.is_shutdown():
        # trajectory()
        jointState()
 
if __name__ == "__main__":
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
    
