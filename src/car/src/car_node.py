import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np
np.set_printoptions(suppress=True)

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

def joyconStatus(joycon):
    k = 1
    rh = joycon.rh
    rv = joycon.rv
    index_R = int(joycon.ri)
    lh = joycon.lh
    lv = joycon.lv
    index_L = int(joycon.li)

    dlh = heavisideFilter(lh-lh0)
    dlv = heavisideFilter(lv-lv0)
    lb = name_L[index_L]
    rb = name_R[index_R]

    if dlh or dlv != 0:
        theta = np.arctan2(dlv, dlh)
        j = np.array([[k * np.cos(theta)], 
                      [k * np.sin(theta)]])
        pL = j
    else:
        pL = np.array([[0],[0]])
    
    return pL, rb
        
class MotorJoyconCmd(object):
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

cmdR0 = 1900
cmdL0 = 2200
pwm_threshold = 40
pwm_ratio = 2500

name_R = ['A', 'B', 'X', 'Y', 'R', 'ZR', 'PLUS', 'NONE_R']
name_L = ['UP', 'RIGHT', 'DOWN', 'LEFT', 'L', 'ZL', 'MINUS', 'NONE_L']
rh0 = 2200
rv0 = 1900
lh0 = 2100
lv0 = 2300
gain = 80

if __name__ == '__main__':
    pwmBuff = np.array([])
    joycon = MotorJoyconCmd()
    rospy.init_node('motor_cmder', anonymous=True)
    try:
        while (not joycon.triggered): pass
        print('Start to Remote Control')
        
        # while (not rospy.is_shutdown()):
        #     # -1176 < cmdR < 1074
        #     # -1087 < cmdR < 1059
        #     cmdR = heavisideFilter(heavisideFilter(joycon.right_joycon - cmdR0) + 100)
        #     cmdL = heavisideFilter(joycon.left_joycon - cmdL0)

        #     pwmR = joycon2PWM(cmdR, 1, pwm_ratio)
        #     pwmL = joycon2PWM(cmdL, 1, pwm_ratio)

        #     # region[not done]
        #     # This part is under construction
        #     if (len(pwmBuff) < 5): 
        #         pwmBuff = np.append(pwmBuff, [pwmR, pwmL])
        #     else:
        #         pwmBuffR = pwmBuff.reshape(2,-1).T[0]
        #         pwmBuffL = pwmBuff.reshape(2,-1).T[1]
        #         if (np.abs(pwmBuffR[-1] - pwmBuffR[0]) > pwm_threshold or 
        #             np.abs(pwmBuffL[-1] - pwmBuffL[0]) > pwm_threshold):
        #             pwmSmoothR = smoothPWM(pwmBuffR[-1], pwmBuffR[0], pwm_threshold)
        #             pwmSmoothL = smoothPWM(pwmBuffL[-1], pwmBuffL[0], pwm_threshold)
        #             for i, j in zip(pwmSmoothR, pwmSmoothL):
        #                 motorJoyconCmd(i, j)
        #                 print(i, j)
        #                 rospy.sleep(0.1)
        #     # endregion[not done]

        #     motorJoyconCmd(pwmR, pwmL)
        #     # print(pwmL, pwmR)
        #     # print(cmdL, cmdR)
        #     rospy.sleep(0.1)

        while (not rospy.is_shutdown()):
            pl, rb = joyconStatus(joycon)
            matrixA = np.array([[-1, 1],
                                [1, 1]])
            pwm = gain * matrixA @ pl + np.array([[20],[20]])
            motorJoyconCmd(pwm[0][0], pwm[1][0])

            if rb == 'X':
                motorJoyconCmd(140, 140)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
