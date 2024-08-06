import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
from std_msgs.msg import Int16
np.set_printoptions(suppress=True)


def motorJoyconCmd(pwmR, pwmL):
    pub = rospy.Publisher('cmd_motor', Point, queue_size=10)
    motor = Point()
    motorR = pwmR
    motorL = pwmL
    motor.x, motor.y = motorR, motorL
    pub.publish(motor)

def stepMotorCmd(step):
    pub = rospy.Publisher('cmd_stepper', Int16, queue_size=10)
    pub.publish(step)

def heavisideFilter(a):
    # Eliminate the noise near origin
    return np.heaviside(np.abs(a)-joy_range, 0) * a


def smoothPWM(a, b, t):
    '''
    Example: 
        Input:
            a = 10, b = 100, t = 40
        Output:
            data = np.array([50, 90])
    '''
    data = np.array([])
    while np.abs(a-b)>t:
        a1 = a - np.abs(a-b)/(a-b)*t
        data = np.append(data, a1)
        a = a1
    return data

        
class MotorJoyconCmd(object):
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.xbox_joy_callback)
        self.lh = lh0
        self.lv = lv0
        self.rt = rt0
        self.buttonA = 0
        self.buttonB = 0
        self.forback = 0
        self.leftright = 0
        self.triggered = False

    def xbox_joy_callback(self, joy_msg):
        # Xbox axes
        self.lh = joy_msg.axes[1]
        self.lv = joy_msg.axes[0] * -1
        self.rt = joy_msg.axes[5]
        self.buttonA = joy_msg.buttons[0]
        self.buttonB = joy_msg.buttons[1]
        self.forback = joy_msg.axes[7]
        self.leftright = joy_msg.axes[6]
        self.triggered = True
        
joy_range = 0.09
lh0 = 0 #2100
lv0 = 0 #2300
rt0 = 1
gain = 40 # 30 <= gain <= 130

def main1():
    pwmBuff = np.array([])
    joycon = MotorJoyconCmd()
    rospy.init_node('motor_cmder', anonymous=True)
    try:
        while (not joycon.triggered): pass
        print('Start to Remote Control') 

        while (not rospy.is_shutdown()):
            hor = heavisideFilter(joycon.lh)
            ver = heavisideFilter(joycon.lv)

            if joycon.rt == 1:
                gain = 40
            elif 0 <= joycon.rt < 1:
                gain = 40 + (1-joycon.rt)*140
            elif -1 <= joycon.rt < 0:
                gain = 180

            # if joycon.buttonA == 1:
            #     gain -= 10
            #     rospy.sleep(0.3)
            # elif joycon.buttonB == 1:
            #     gain += 10
            #     rospy.sleep(0.3)
            # if gain <= 30:
            #     gain = 30
            # elif gain >= 130:
            #     gain = 130
            
            joycmd = np.array([[ver, hor]]).T
            matrixA = np.array([[-1, 1],
                                [1, 1]])
            pwm = gain * matrixA @ joycmd# + np.array([[20, 20]]).T

            if 0 <= pwm[0][0] < 20:
                pwm[0][0] = 20
            # elif 0 > pwm[0][0] >= -20:
                # pwm[0][0] = -20

            if 0 <= pwm[1][0] < 20:
                pwm[1][0] = 20
            # elif 0 > pwm[0][0] >= -20:
            #     pwm[1][0] = -20 

            motorJoyconCmd(pwm[0][0], pwm[1][0])
            #motorJoyconCmd(-10,0)
            # print(hor, ver)
            print(pwm.T)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass

def main2():
    joycon = MotorJoyconCmd()
    rospy.init_node('motor_cmder', anonymous=True)
    pwm_trn = np.array([20, 20])
    pwm_rot = np.array([20,-20])
    count_trn = 0
    count_rot = 0
    try:
        while (not joycon.triggered): pass
        print('Start to Remote Control') 

        while (not rospy.is_shutdown()):
            count_trn += joycon.forback
            count_rot += joycon.leftright
            if abs(count_trn) > 5: count_trn = 5*joycon.forback
            if abs(count_rot) > 5: count_rot = 5*joycon.leftright
            
            if joycon.buttonB == 1:
                count_rot = 0
                for i in range(abs(int(count_trn))):
                    count_trn -= 1*abs(count_trn)/count_trn
                    pwm = pwm_trn*(2*count_trn) + pwm_rot*2*count_rot
                    print(pwm)
                    motorJoyconCmd(pwm[0], pwm[1])
                    rospy.sleep(0.1)

            pwm = pwm_trn*(2*count_trn) + pwm_rot*2*count_rot
            if abs(pwm[0]) > 200: pwm = np.array([200*abs(pwm[0])/pwm[0], pwm[1]])
            if abs(pwm[1]) > 200: pwm = np.array([pwm[0], 200*abs(pwm[1])/pwm[1]])
            
            print(pwm)
            motorJoyconCmd(pwm[0], pwm[1])
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main2()
