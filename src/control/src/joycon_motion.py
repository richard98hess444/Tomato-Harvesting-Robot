from __future__ import print_function
from tkinter.tix import X_REGION
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist

import numpy as np
from numpy import sin, cos, pi
np.set_printoptions(suppress=True)
from scipy.spatial.transform import Rotation
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "sinica_arm"
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

        plan = move_group.go(joint_goal, wait=True)
        move_group.stop()
        return plan

    def set_pose_goal(self, t):   
        E_ref = np.array([[t[0]], [t[1]]])
        theta1, theta2 ,theta3 = tmtInvKin(E_ref)
        self.set_joint_state(np.array([theta1, theta2 ,theta3,0]))

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
        a = np.array([a0,a1,a2,a3])
        print("angle:", a)
        return a


def rotation2D(t, degree = True):
    if(degree == True):
        t = t/180*np.pi

    return np.array([[cos(t), -sin(t)], 
                     [sin(t),  cos(t)]])

def check_theta(t1, t2, t3, c1, c2, c3):
    if(t3<c3[0] or t3>c3[1]):
        return 0
    elif(t1<c1[0] or t1>c1[1]):
        return 0
    elif(t2<c2[0] or t2>c2[1]):
        return 0
    return 1

def tmtInvKin(e_ref):
    # print(e_ref)
    e2 = e_ref - r0 @ l3
    x = e2[0][0]
    y = e2[1][0]

    l1x = l1[0][0]
    l2x = l2[0][0]
    l = np.sqrt(x**2+y**2)

    try:
        phi = np.arctan2(y,-x)*180/np.pi
        psi_1 = np.arccos((l1x**2+l**2-l2x**2)/(2*l1x*l))*180/np.pi
        psi_2 = np.arccos((l2x**2+l**2-l1x**2)/(2*l2x*l))*180/np.pi
        # warnings.warn(Warning())
    except:
        print('out of range, back to home position')
        return 90, -90, 0

    a1 = phi + psi_1
    a2 = phi - psi_2
    theta1 = 90 - a1
    theta2 = a1 - a2
    theta3 = -theta1-theta2
    check = check_theta(theta1, theta2, theta3, 
                        [-45,90], [-135,120], [-80,11])
    
    if(check == 0):
        print('first invalid to reach the position')
        print(theta1,theta2,theta3)
        theta1 = 90 - a2
        theta2 = a2 - a1
        theta3 = -theta1-theta2
        check = check_theta(theta1, theta2, theta3, 
                        [-45,90], [-135,120], [-80,11])
    else:
        return theta1, theta2 ,theta3
        
    if(check == 0):
        print('second invalid to reach the position')
        print(theta1,theta2,theta3)
        return 90, -90, 0
    else:
        return theta1, theta2 ,theta3

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
    
    rotation_mtx = Rx(rx) @ Ry(ry) @ Rz(rz)
    translation_mtx = np.array([[tx, ty, tz]]).T

    tansformation_mtx = np.hstack((rotation_mtx, translation_mtx))
    tansformation_mtx = np.vstack((tansformation_mtx, [0, 0, 0, 1]))

    return tansformation_mtx

def Rz(t):
    return np.array([[cos(t), -sin(t)], [sin(t), cos(t)]])

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

def stepper_Publisher2(pub, s):
    stepper_msg = Int16(data = s)
    pub.publish(stepper_msg)
    # print('published')

def gripper_Publisher2(pub, s):
    gripper_msg = Int16(data = s)
    pub.publish(gripper_msg)

def heavisideFilter(a):
    return np.heaviside(np.abs(a)-300, 0) * a

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
        pL = Rz(-np.pi/2*0) @ j
    else:
        pL = np.array([[0],[0]])
    
    return pR.T[0], pL.T[0], rb, lb

# Fixed Parameters 1
l1 = np.array([[0.159],[0]])
l2 = np.array([[0.204],[0]])
l3 = np.array([[0.048],[0.043]])
r0 = rotation2D(90)
home = np.array([90, -90, 0, 0])

# Fixed Parameters 2
fx = 380.02
fy = 379.79
a = 22.5
b = 10.2
initial_height = 0.520
offset = 0.0396 + 0.03
height = initial_height + offset

# Fixed Parameters 3
sleepTime = 0.3
dislocation_controller = 1

## Tunable Parameters
u_test = 274
v_test = 258
z_test = 620
usbTomato = np.array([[320, 240]]).T
gain = 0.2
g = 1

## Calculated Transformation Data
def ceiling_para(t):
    x = (a*cos(t) + b*sin(t))*0.001
    z = (a*sin(t) - b*cos(t) + 11)*0.001
    return x,z

def cof_para(input_space):
    img_space = input_space - np.array([[320, 240, 0, 0]]).T
    xw = img_space[0][0]*img_space[2][0]/fx*0.001
    yw = img_space[1][0]*img_space[2][0]/fy*0.001
    return xw, yw

def armBase_para(steps):
    global height
    deltaH = (0.0192*steps)*0.001
    deltaZ = height + deltaH
    height = deltaZ
    return deltaZ


# Fixed Transformation Matrix
cblTcof = trans(-90,90,0,0.0115,0.0475,0.0145)
baseTceiling = trans(0,0,0,0,0,0.788)
name_R = ['A', 'B', 'X', 'Y', 'R', 'ZR', 'PLUS', 'NONE_R']
name_L = ['UP', 'RIGHT', 'DOWN', 'LEFT', 'L', 'ZL', 'MINUS', 'NONE_L']
rh0 = 2133
rv0 = 1872
lh0 = 2080
lv0 = 2315
k = 0.02

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
        delta_stepperStep = 0
        tutorial.set_joint_state(home)
        position = tutorial.check_position()
        rospy.sleep(sleepTime)

        while(not rospy.is_shutdown()):
            jsr, jsl, rb, lb = joyconStatus(joycon)
            print(jsr, jsl, rb, lb)
            rospy.sleep(0.1)
            

            
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

