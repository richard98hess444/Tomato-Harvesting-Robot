# include "ros/ros.h"
# include "std_msgs/Float32MultiArray.h"

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("I heard: [%f],[%f],[%f],[%f]", msg->data.at(0),msg->data.at(1),msg->data.at(2),msg->data.at(3));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_sservo", 1000, callback);
    ros::spin();

    return 0;
}