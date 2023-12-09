from __future__ import print_function
from tkinter.tix import X_REGION
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from scipy.spatial.transform import Rotation

import numpy as np
from numpy import sin, cos, pi
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
# import warnings
# warnings.filterwarnings('error')


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

def check_stable(u,v,ue,ve):
    if np.abs(u-ue)>2 and np.abs(v-ve)>2:
        return ue, ve
    else:
        return u,v

class imu_subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/imuInfo", Float32, self.ImuCallBack)
        self.myangle = 0
        self.triggered = False # thanks to https://github.com/sciyen

    def ImuCallBack(self, data):
        angle = data.data # we add 2 degree here
        angle = angle*np.pi/180
        self.triggered = True
        self.myangle = angle
        self.unsubscribe()
        # print("exit: ", angle) 
        # print("exit2: ", t_ceiling)
        
    def unsubscribe(self):
        self.sub.unregister()

class cof_subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/rgbd_coordinate", Float32MultiArray, self.CofCallBack)
        self.u = u_test
        self.v = v_test
        self.z = z_test
        self.triggered = False

    def CofCallBack(self, data):
        coordinate = data.data
        self.u = coordinate[0]
        self.v = coordinate[1]
        self.z = coordinate[2]
        if coordinate[2] != 0:
            self.triggered = True

class usbTomato_subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/usb_cdn", Point, self.CofCallBack)
        self.tomato = np.array([[0,0]]).T
        self.triggered = False

    def CofCallBack(self, data):
        x = round(data.x)
        y = round(data.y)
        self.tomato = np.array([[x, y]]).T
        self.triggered = True
        # self.unsubscribe()

    def unsubscribe(self):
        self.sub.unregister()


def stepper_Publisher2(pub, s):
    stepper_msg = Int16(data = s)
    pub.publish(stepper_msg)
    # print('published')

def gripper_Publisher2(pub, s):
    gripper_msg = Int16(data = s)
    pub.publish(gripper_msg)

# Fixed Parameters 1
l1 = np.array([[0.159],[0]])
l2 = np.array([[0.204],[0]])
l3 = np.array([[0.048],[0.043]])
r0 = rotation2D(90)
home = np.array([90, -45, 0, 0])

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

def main():

    # data recording
    arm_x = np.array([])
    arm_y = np.array([])
    arm_z = np.array([])
    e_x = np.array([])
    e_y = np.array([])
    u_z = np.array([])

    # class
    tutorial = MoveGroupPythonInterfaceTutorial()
    imu = imu_subscriber()
    cof = cof_subscriber()
    tmt = usbTomato_subscriber()
    pub_stepper = rospy.Publisher("/cmd_stepper", Int16, queue_size=10)
    pub_gripper = rospy.Publisher("/cmd_gripper", Int16, queue_size=10)
    while (not imu.triggered):
        pass
    while (not cof.triggered):
        pass


    print('========================')
    print('|                      |')
    print('|    PROGRAM STARTS    |')
    print('|                      |')
    print('========================')
    rospy.sleep(sleepTime)

    try:
        delta_stepperStep = 0
        # compensate = np.array([[0.,0.,0.,0.]]).T
        img_compensate = np.array([[0.,0.,0.,0.]]).T
        print('go to home position')
        tutorial.set_joint_state(home)
        position = tutorial.check_position()
        arm_x = np.append(arm_x, position[0])
        arm_y = np.append(arm_y, position[1])
        arm_z = np.append(arm_z, height)
        rospy.sleep(sleepTime)

        while(not rospy.is_shutdown()):

            # region [first step]
            gripper_Publisher2(pub_gripper, 1)
            # print('img compensate\n', img_compensate)
            if (dislocation_controller == 1):
                img_space = np.array([[cof.u, cof.v, cof.z, 1]]).T + img_compensate
            else:
                img_space = np.array([[cof.u, cof.v, cof.z, 1]]).T
            img_first = img_space
            img_last = np.array([[0.,0.,0.,0.]]).T
            # print('img space\n', img_space)
            # img_space = np.array([[u_test, v_test, z_test, 1]]).T
            xw, yw = cof_para(img_space)
            p_cof = np.array([[xw, yw, img_space[2][0]*0.001, 1]]).T
            ceilingX, ceilingZ = ceiling_para(imu.myangle)
            dz = armBase_para(delta_stepperStep)

            ceilingTcbl = trans(0,90-imu.myangle/np.pi*180,0,ceilingX,0,ceilingZ)
            baseTarmbase = trans(-90,0,0,0.13,0,dz)
            armbaseTbase = np.linalg.inv(baseTarmbase)
            p_armbase = armbaseTbase @ baseTceiling @ ceilingTcbl @ cblTcof @ p_cof
            p_armbase = p_armbase - np.array([[0,0.13+0.02,0,0]]).T

            tr = [p_armbase[0][0], p_armbase[1][0]]
            tz = p_armbase[2][0]
            delta_stepperStep = int((tz*1000+0.4)/0.0192)
            print('first position')
            try:
                # print('setting pose goal')
                tutorial.set_pose_goal(tr)
            except:
                print('set_pose_goal error')
            rospy.sleep(sleepTime)
            if (np.abs(delta_stepperStep) > 100):
                stepperStep = 5000 + delta_stepperStep
                # print("FF delta step", delta_stepperStep)
                u_z = np.append(u_z, delta_stepperStep)
                stepper_Publisher2(pub_stepper, stepperStep)
                rospy.sleep(sleepTime)
            else:
                delta_stepperStep = 0
            
            arm_x = np.append(arm_x, p_armbase[0][0])
            arm_y = np.append(arm_y, p_armbase[1][0])
            first_p = np.array([p_armbase[0][0], p_armbase[1][0], tz + height])
            # endregion [first step]

            while(1):
                data = np.array([])
                while (not tmt.triggered):
                    pass
                while(1):
                    while(len(data)<20):
                        u_err = tmt.tomato[0][0]-320
                        v_err = tmt.tomato[1][0]-240
                        rospy.sleep(sleepTime)
                        data = np.append(data, u_err)
                        data = np.append(data, v_err)
                        # np.save('/home/richa/notebook/jupyterenv/notebook/yolo_detect/error_data', data)
                    u = data.reshape(-1,2).T[0]
                    v = data.reshape(-1,2).T[1]
                    if (np.abs(np.mean(u)-u[-1]) < 2):
                        # print('steady error')
                        break
                    else:
                        # print('not yet steady')
                        data = np.delete(data, [0,1,2,3])

                print("========Steady Error========", u_err, v_err)
                e_x = np.append(e_x, u_err)
                e_y = np.append(e_y, v_err)

                if(np.abs(u_err)<20 and np.abs(v_err)<20):
                    dz = armBase_para(delta_stepperStep)
                    arm_z = np.append(arm_z, dz)
                    # last_p = np.array([arm_x[-1], arm_y[-1], arm_z[-1]])
                    img_last = img_space
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/arm_x.npy', arm_x)
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/arm_y.npy', arm_y)
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/arm_z.npy', arm_z)
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/e_x.npy', e_x)
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/e_y.npy', e_y)
                    np.save('/home/richa/notebook/jupyterenv/notebook/transformation/u_z.npy', u_z)
                    break
                
                if(np.abs(u_err)<6):
                    u_err = 0
                u_err = u_err*gain
                v_err = v_err*gain

                img_space = img_space + np.array([[u_err, v_err, 0, 0]]).T
                xwp, ywp = cof_para(img_space)
                pp = np.array([[xwp, ywp, img_space[2][0]*0.001, 1]]).T
                ceilingX, ceilingZ = ceiling_para(imu.myangle)
                dz = armBase_para(delta_stepperStep)

                ceilingTcbl = trans(0,90-imu.myangle/np.pi*180,0,ceilingX,0,ceilingZ)
                baseTarmbase = trans(-90,0,0,0.13,0,dz)
                armbaseTbase = np.linalg.inv(baseTarmbase)
                p_armbase_p = armbaseTbase @ baseTceiling @ ceilingTcbl @ cblTcof @ pp
                p_armbase_p = p_armbase_p - np.array([[0,0.13+0.02,0,0]]).T

                arm_x = np.append(arm_x, p_armbase_p[0][0])
                arm_y = np.append(arm_y, p_armbase_p[1][0])
                arm_z = np.append(arm_z, baseTarmbase[2][3])

                trp = [p_armbase_p[0][0], p_armbase_p[1][0]]
                # print(trp)
                tzp = p_armbase_p[2][0]
                delta_stepperStep = int((tzp*1000+0.4)/0.0192)
                u_z = np.append(u_z, delta_stepperStep)

                if (np.abs(delta_stepperStep) > 100):
                    stepperStep = 5000 + delta_stepperStep
                    stepper_Publisher2(pub_stepper, stepperStep)
                    rospy.sleep((200)/2000)
                else:
                    delta_stepperStep = 0
                try:
                    tutorial.set_pose_goal(trp)
                except:
                    print('set_pose_goal error')        
                print('end of iter')
                rospy.sleep(sleepTime)
            
            print('Congratulations!')
            p_armbase_new = tutorial.check_position()
            tr_new = [p_armbase_new[0]-0.02, p_armbase_new[1]+0.065]
            try:
                tutorial.set_pose_goal(tr_new)
                rospy.sleep(sleepTime)
            except:
                print('set_pose_goal error')
            gripper_Publisher2(pub_gripper, 0)
            tutorial.set_joint_state(home)
            rospy.sleep(sleepTime*2)
            gripper_Publisher2(pub_gripper, 1)
            position = tutorial.check_position()
            arm_x = np.append(arm_x, position[0])
            arm_y = np.append(arm_y, position[1])
            arm_z = np.append(arm_z, arm_z[-1])
            e_x = np.append(e_x, 0)
            e_y = np.append(e_y, 0)
            u_z = np.append(u_z, 0)

            img_err = img_last - img_first
            img_compensate += img_err
            print('img error:\n', img_err)
            gripper_Publisher2(pub_gripper, 1)
            # err_d = np.hstack((last_p-first_p, 0))
            # err_d = err_d.reshape(4,1)
            # compensate += err_d
            # print('the dislocation error:[', err_d[0][0], ',', err_d[1][0], ',', err_d[2][0], ']')
            x = float(input())
            
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

