# from calendar import c
# from pyexpat.model import XML_CQUANT_NONE
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import cv2
import numpy as np
bridge = CvBridge()

X = 640
Y = 480
x_center = X/2
y_center = Y/2
xu = X/2
yu = Y/2
depth = np.array([])

def boundingBoxCallBack(data):
    global x_center
    global y_center
    x_center = round(data.x)
    y_center = round(data.y)
    # print(x_center, y_center)

def usbBoundingBoxCallBack(data):
    global xu
    global yu
    xu = round(data.x)
    yu = round(data.y)

    pub = rospy.Publisher("/usb_cdn", Point, queue_size=20)
    tmt = Point()
    tmt.x, tmt.y = xu, yu
    pub.publish(tmt)

def imageColorCallback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.circle(cv_image, (int(x_center), int(y_center)), 
                              radius=10, color=(255, 0, 0), thickness=-1)
        cv_image = cv2.line(cv_image, (0, int(Y/2)), (X, int(Y/2)), 
                            (0, 255, 0), thickness=2)
        cv_image = cv2.line(cv_image, (int(X/2), 0), (int(X/2), Y), 
                            (0, 255, 0), thickness=2)
        # cv2.imshow("frame_realsense", cv_image) #我好愛做實驗
        # cv2.waitKey(3)
        ros_img = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub = rospy.Publisher("/realsense_img", Image, queue_size=20)
        pub.publish(ros_img)
    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return
    
def imageUsbCallback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.circle(cv_image, (int(xu), int(yu)), 
                              radius=10, color=(255, 0, 0), thickness=-1)
        cv_image = cv2.line(cv_image, (0, int(Y/2)), (X, int(Y/2)), 
                            (0, 255, 0), thickness=2)
        cv_image = cv2.line(cv_image, (int(X/2), 0), (int(X/2), Y), 
                            (0, 255, 0), thickness=2)
        # cv2.imshow("frame_usb", cv_image) #我好愛做實驗
        # cv2.waitKey(3)
        ros_img_usb = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub = rospy.Publisher("/webcam_img", Image, queue_size=20)
        pub.publish(ros_img_usb)
    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def imageDepthCallback(data):
    global depth
    try:
        cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
        center = [x_center, y_center, cv_image[int(y_center), int(x_center)]]
        print("Coordinate:(%d, %d, %d)" %(center[0], center[1], center[2]))
        # print(cv_image)
        # depth = np.append(depth, [center[0], center[1], center[2]])
        # np.save('/home/richa/notebook/jupyterenv/notebook/test/depth', depth)
        pub = rospy.Publisher("rgbd_coordinate", Float32MultiArray, queue_size=20)
        center_msg = Float32MultiArray(data = center)
        pub.publish(center_msg)

        rate = rospy.Rate(10)
        rate.sleep()

    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def main():
    realsense_tomato_topic = '/RGBD_detect'
    usbcam_tomato_topic = '/usbcam_detect'
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    rgb_image_topic = '/camera/color/image_raw' 
    usbcam_image_topic = '/usb_cam/image_raw'

    rospy.Subscriber(realsense_tomato_topic, Point, boundingBoxCallBack)
    rospy.Subscriber(usbcam_tomato_topic, Point, usbBoundingBoxCallBack)

    rospy.Subscriber(depth_image_topic, Image, imageDepthCallback)
    rospy.Subscriber(rgb_image_topic, Image, imageColorCallback)
    rospy.Subscriber(usbcam_image_topic, Image, imageUsbCallback)
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("center_depth_node")
    main()

