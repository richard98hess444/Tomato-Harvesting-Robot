import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
from numpy import sin, cos
np.set_printoptions(suppress=True)

def vec_2_eulerAngle(vx, vy, vz):
    v = np.array([[vx], [vy], [vz]])
    v = v/np.sqrt(np.sum(v**2))

    j = np.arcsin(v[0])
    if(abs(j) > np.pi):
        j = j/abs(j)*np.pi - j

    i = np.arctan2(-v[1]/cos(j), v[2]/cos(j))
    j = float(j)*180/np.pi
    i = float(i)*180/np.pi
    k = -0.5*np.pi*180/np.pi
    return k, j, i

def listener():
    rospy.Subscriber("/camera/imu", Imu, ImuCallBack)
    rospy.spin()

def ImuCallBack(data):
    linear_acc = data.linear_acceleration
    x = linear_acc.x
    y = linear_acc.y
    z = linear_acc.z
    # g = np.sqrt(x**2+y**2+z**2)

    rz, ry, rx = vec_2_eulerAngle(x,y,z)
    r = np.array([rz,ry,90-abs(90-rx)])

    # print("\n", linear_acc)
    # print(g)
    # print(r)
    rospy.loginfo("[%f, %f, %f]", r[0], r[1], r[2])
    
    pub = rospy.Publisher("imuInfo", Float32, queue_size=10)
    imu_msg = Float32(data = r[2])
    pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('imu_info')
        listener()
    except rospy.ROSinterruptException:
        pass