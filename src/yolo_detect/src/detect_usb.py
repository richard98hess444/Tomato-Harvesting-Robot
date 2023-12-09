import torch
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()
model = torch.hub.load('ultralytics/yolov5','custom', path='/home/richa/sinica_ws/src/yolo_detect/src/best2.pt')  # or yolov5m, yolov5l, yolov5x, etc.
im_path = '/home/richa/sinica_ws/src/yolo_detect/data/'  # or file, Path, URL, PIL, OpenCV, numpy, list

model.conf = 0.7  # NMS confidence threshold
model.iou = 0.45  # NMS IoU threshold
model.classes = [0,1,2]  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
model.max_det = 10  # maximum number of detections per image
lower_red = np.array([0,100,50])
upper_red = np.array([12,255,255])


def camera_detection(cv2_img):
    hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(cv2_img, cv2_img, mask=mask)

    frame = res
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(frame)
    results_pandas = results.pandas().xyxy[0]

    results_pandas['box'] = (results_pandas['ymax']-results_pandas['ymin']) * (results_pandas['xmax']-results_pandas['xmin'])
    results_pandas['coordinates_x'] = (results_pandas['xmax']+results_pandas['xmin'])/2
    results_pandas['coordinates_y'] = (results_pandas['ymax']+results_pandas['ymin'])/2
    results_pandas = results_pandas.sort_values('class', ascending=False)

    # print(results_pandas)
    # results.show()

    if results_pandas.empty == True:
        pass
    else:
        x = results_pandas['coordinates_x'].iloc[0]
        y = results_pandas['coordinates_y'].iloc[0]
        yield(x, y)


def camera_talker(cv2_img):
    pub = rospy.Publisher('usbcam_detect', Point, queue_size=10)
    rate = rospy.Rate(10)
    big_tomato = Point()
    for x, y in camera_detection(cv2_img):
        big_tomato.x, big_tomato.y = x, y
        print(round(x), round(y))
        pub.publish(big_tomato)
        rate.sleep()


def image_callback(data):
    try:
        cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        camera_talker(cv2_img)
    except CvBridgeError:
        print('error')

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rospy.init_node('usbcam_detector', anonymous=False)
            # /camera/color/image_raw  /usb_cam/image_raw
            rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
