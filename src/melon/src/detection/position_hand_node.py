import rospy
from geometry_msgs.msg import Point

def position_publisher():
    print('enter coordinate:')
    print('x:0~640, y:0:~480')
    x = float(input())
    y = float(input())
    position = Point()
    position.x, position.y = x, y
    pub = rospy.Publisher('hand_position', Point, queue_size=10)
    pub.publish(position)

if __name__ == '__main__':
    try:
        rospy.init_node('position_hand_node')
        while not rospy.is_shutdown():
            position_publisher()
    except rospy.ROSinterruptException:
        pass