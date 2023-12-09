from pyjoycon import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def joyconPublisher(joycon_R, joycon_L, loop):
    pub = rospy.Publisher('joycon_msg', Twist, queue_size=10)
    rospy.init_node('joycon_node', anonymous=True)
    while loop:
        ra = joycon_R.get_button_a()
        rb = joycon_R.get_button_b()
        rx = joycon_R.get_button_x()
        ry = joycon_R.get_button_y()
        rr = joycon_R.get_button_r()
        rz = joycon_R.get_button_zr()
        rplus = joycon_R.get_button_plus()
        joySticks_R = joycon_R.get_status().get('analog-sticks')['right']
        rh = joySticks_R['horizontal']
        rv = joySticks_R['vertical']

        lup = joycon_L.get_button_up()
        lright = joycon_L.get_button_right()
        ldown = joycon_L.get_button_down()
        lleft = joycon_L.get_button_left()
        ll = joycon_L.get_button_l()
        lz = joycon_L.get_button_zl()
        lminus = joycon_L.get_button_minus()
        joySticks_L = joycon_L.get_status().get('analog-sticks')['left']
        lh = joySticks_L['horizontal']
        lv = joySticks_L['vertical']

        name_R = ['A', 'B', 'X', 'Y', 'R', 'ZR', 'PLUS', 'NONE_R']
        name_L = ['UP', 'RIGHT', 'DOWN', 'LEFT', 'L', 'ZL', 'MINUS', 'NONE_L']
        state_R = np.array([ra,rb,rx,ry,rr,rz,rplus])
        state_L = np.array([lup,lright,ldown,lleft,ll,lz,lminus])
        condition_R = np.sum(state_R)
        condition_L = np.sum(state_L)
        index_R = -1
        index_L = -1
        
        data = Twist()
        data.linear.x = rh
        data.linear.y = rv
        data.linear.z = -1
        data.angular.x = lh
        data.angular.y = lv
        data.angular.z = -1

        if condition_R:
            index_R = np.where(state_R==1)[0][0]
            data.linear.z = index_R
            if name_R[index_R]=='PLUS':
                loop = False
        
        if condition_L:
            index_L = np.where(state_L==1)[0][0]
            data.angular.z = index_L
        
        pub.publish(data)
        rospy.sleep(0.005)
        
connected = False

if __name__ == '__main__':
    while (not connected):
        try:
            loop = True
            joycon_id_R = get_R_id()
            joycon_id_L = get_L_id()
            joycon_R = JoyCon(*joycon_id_R)
            joycon_L = JoyCon(*joycon_id_L)
            print('Joycon connected')
            joyconPublisher(joycon_R, joycon_L, loop)
            connected = True

        except:
            print('No Joycon Connection')
            print('Reconnecting in 3 seconds...')
            rospy.sleep(3)
        