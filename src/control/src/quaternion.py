from __future__ import print_function
from tkinter.tix import X_REGION
from six.moves import input
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation

import numpy as np
from numpy import sin, cos
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
from tm_msgs.msg import *
from tm_msgs.srv import *

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        # print('1')
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        # print('2')
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        # print('3')
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

    def set_joint_state(self, a):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = a[0]/(180/pi)
        joint_goal[1] = a[1]/(180/pi)
        joint_goal[2] = a[2]/(180/pi)
        joint_goal[3] = a[3]/(180/pi)

        plan = move_group.go(joint_goal, wait=True)
        move_group.stop()
        return plan

    def set_pose_goal(self, t):   
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = rot_quat[0]
        # pose_goal.orientation.y = rot_quat[1]
        # pose_goal.orientation.z = rot_quat[2]
        # pose_goal.orientation.w = rot_quat[3]
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z = t[2]

        move_group.set_pose_target(pose_goal)
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

        position = np.array([x, y, z, rot_euler[0], rot_euler[1], rot_euler[2]])
        return position

    def check_joint_state(self):
        move_group = self.move_group
        current_joints = move_group.get_current_joint_values()
        a0 = current_joints[0]*180/pi
        a1 = current_joints[1]*180/pi
        a2 = current_joints[2]*180/pi
        a3 = current_joints[3]*180/pi
        a = np.array([a0,a1,a2,a3])
        # print("angle:", a)
        return a

    # def plan_cartesian_path(self, t, scale=1):
    #     move_group = self.move_group
    #     waypoints = []

    #     wpose = move_group.get_current_pose().pose
    #     wpose.position.z -= scale * t[2]  # First move up (z)
    #     wpose.position.y += scale * t[1]  # and sideways (y)
    #     waypoints.append(copy.deepcopy(wpose))

    #     wpose.position.x += scale * t[0]  # Second move forward/backwards in (x)
    #     waypoints.append(copy.deepcopy(wpose))

    #     wpose.position.y -= scale * t[1]  # Third move sideways (y)
    #     waypoints.append(copy.deepcopy(wpose))

    #     (plan, fraction) = move_group.compute_cartesian_path(
    #         waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    #     )  # jump_threshold

    #     return plan 
    
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)




ready1 = np.array([0, 0, 0, 0])

def main():

    tutorial = MoveGroupPythonInterfaceTutorial()
    print('========================')
    print('|                      |')
    print('|    PROGRAM STARTS    |')
    print('|                      |')
    print('========================')

    try:
        # set initial pose
        tutorial.set_joint_state(ready1)

        while(1):
            p = tutorial.check_position()
            print(p)
            rospy.sleep(1)
            
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

