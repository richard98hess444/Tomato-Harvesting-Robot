import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

def position_publisher():
    print('enter coordinate:')
    print('x:0~640, y:0:~480')
    x = float(input())
    y = float(input())
    position = Point()
    position.x, position.y = x, y

    pub = rospy.Publisher('RGBD_detect', Point, queue_size=10)
    pub.publish(position)

if __name__ == '__main__':
    try:
        rospy.init_node('position_hand_node')
        while(1):
            position_publisher()
    except rospy.ROSinterruptException:
        pass