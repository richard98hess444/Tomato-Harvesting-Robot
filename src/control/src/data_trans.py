import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

def listener():
    rospy.Subscriber("/joint_states", JointState, jointState)
    rospy.spin()

def jointState(data):
    s = [data.position[0], data.position[1], data.position[2], data.position[3]]
    pub = rospy.Publisher('cmd_servo', Quaternion, queue_size=10)
    joint_state = Quaternion()
    joint_state.x = s[0]
    joint_state.y = s[1]
    joint_state.z = s[2]
    joint_state.w = s[3]
    pub.publish(joint_state)
        

if __name__ == '__main__':
    try:
        rospy.init_node('data_trans')
        listener()
    except rospy.ROSinterruptException:
        pass