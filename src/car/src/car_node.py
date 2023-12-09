import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np

def motorHandCmd():
    pub = rospy.Publisher('cmd_motor', Point, queue_size=10)
    motor = Point()
    while not rospy.is_shutdown():
        print('Enter PWM for R and L Motor:')
        motorR = float(input())
        motorL = float(input())
        motor.x, motor.y = motorR, motorL
        pub.publish(motor)
        rospy.sleep(0.1)

def motorJoyconCmd(pwmR, pwmL):
    pub = rospy.Publisher('cmd_motor', Point, queue_size=10)
    motor = Point()
    motorR = pwmR
    motorL = pwmL
    motor.x, motor.y = motorR, motorL
    pub.publish(motor)

def heavisideFilter(a):
    # Eliminate the noise near origin
    return np.heaviside(np.abs(a)-300, 0) * a

def joycon2PWM(joycon, interval, ratio):
    # Convert joycon value to PWM value
    # Use interval to set the step of the PWM
    # A larger ration means a slower velocity
    return int(255*joycon/ratio/interval)*interval + 20

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
        self.sub = rospy.Subscriber("/joycon_msg", Twist, self.joyconCallback)
        self.right_joycon = 0
        self.left_joycon = 0
        self.triggered = False

    def joyconCallback(self, data):
        self.right_joycon = data.linear.y
        self.left_joycon = data.angular.y
        self.triggered = True

cmdR0 = 1900
cmdL0 = 2200
pwm_threshold = 40
pwm_ratio = 2500

if __name__ == '__main__':
    pwmBuff = np.array([])
    joycon = MotorJoyconCmd()
    rospy.init_node('motor_cmder', anonymous=True)
    try:
        while (not joycon.triggered): pass
        print('Start to Remote Control')
        
        while (not rospy.is_shutdown()):
            # -1176 < cmdR < 1074
            # -1087 < cmdR < 1059
            cmdR = heavisideFilter(heavisideFilter(joycon.right_joycon - cmdR0) + 100)
            cmdL = heavisideFilter(joycon.left_joycon - cmdL0)

            pwmR = joycon2PWM(cmdR, 1, pwm_ratio)
            pwmL = joycon2PWM(cmdL, 1, pwm_ratio)

            # region[not done]
            # This part is under construction
            if (len(pwmBuff) < 5): 
                pwmBuff = np.append(pwmBuff, [pwmR, pwmL])
            else:
                pwmBuffR = pwmBuff.reshape(2,-1).T[0]
                pwmBuffL = pwmBuff.reshape(2,-1).T[1]
                if (np.abs(pwmBuffR[-1] - pwmBuffR[0]) > pwm_threshold or 
                    np.abs(pwmBuffL[-1] - pwmBuffL[0]) > pwm_threshold):
                    pwmSmoothR = smoothPWM(pwmBuffR[-1], pwmBuffR[0], pwm_threshold)
                    pwmSmoothL = smoothPWM(pwmBuffL[-1], pwmBuffL[0], pwm_threshold)
                    for i, j in zip(pwmSmoothR, pwmSmoothL):
                        motorJoyconCmd(i, j)
                        print(i, j)
                        rospy.sleep(0.1)
            # endregion[not done]

            motorJoyconCmd(pwmR, pwmL)
            # print(pwmL, pwmR)
            # print(cmdL, cmdR)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass