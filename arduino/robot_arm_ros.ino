#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400
#define FREQUENCY 50
#define STEPPIN 9
#define DIRPIN 8

int s0 = 2203;
int s1 = 2600;
int s2 = 1380;
int s3 = 650;
int s4 = 1000;

float pi = 3.1415926;
const int t = 200;
const int normal = 5000; // benchmark
int stp;

ros::NodeHandle nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


/* -------------- Servo and Gripper -------------- */
//void positionCb(const sensor_msgs::JointState joint_state){
//  s0 = 6.9556*(joint_state.position[0]*180/pi) + 1576.8; 
//  s1 = -11.889*(joint_state.position[1]*180/pi) + 1680;
//  s2 = 12.222*(joint_state.position[2]*180/pi) + 1380; 
//  s3 = joint_state.position[3]*180/pi;
//  nh.loginfo("We got something.");
//}

void positionCb(const geometry_msgs::Quaternion& joint_state){
  s0 = 6.9556*(joint_state.x*180/pi) + 1576.8; 
  s1 = -11.889*(joint_state.y*180/pi) + 1680;
  s2 = 12.222*(joint_state.z*180/pi) + 1380; 
  s3 = joint_state.w*180/pi + 650;
  nh.loginfo("joint move");
}

void gripperCb(const std_msgs::Int16 servo_msg){
  s4 = servo_msg.data;
  if(s4>0){
    s4 = 1000;
    nh.loginfo("Gripper ON.");
  }
  else{
    s4 = 1300;
    nh.loginfo("Gripper OFF.");
  }
}

/* -------------- Vertical Stepper -------------- */
void forward(int stp){
  digitalWrite(DIRPIN, HIGH);
  for (int x = 0; x < stp; x ++) {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(t);
    digitalWrite(STEPPIN, LOW);
    delayMicroseconds(t);
  }
}

void backward(int stp){
  digitalWrite(DIRPIN, LOW);
  for (int x = 0; x < stp; x ++) {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(t);
    digitalWrite(STEPPIN, LOW);
    delayMicroseconds(t);
  }
}

void stepperCb(const std_msgs::Int16& stepper_msg){
  stp = stepper_msg.data;
  
  if(stp>normal){
    nh.loginfo("UP");
    digitalWrite(13, HIGH);
    backward(stp-normal); 
  }
  else{
    nh.loginfo("DOWN");
    digitalWrite(13, LOW);
    forward(normal-stp);
  }
}

/* -------------- Subscriber -------------- */
//ros::Subscriber<sensor_msgs::JointState> sub_s("joint_states", positionCb);
ros::Subscriber<geometry_msgs::Quaternion> sub_s("cmd_servo", positionCb);
ros::Subscriber<std_msgs::Int16> sub_g("cmd_gripper", gripperCb);
ros::Subscriber<std_msgs::Int16> sub_st("cmd_stepper", &stepperCb);

void setup() {
  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(13, OUTPUT);
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  // This is the maximum PWM frequency
  
  nh.initNode();
  nh.subscribe(sub_s);
  nh.subscribe(sub_g);
  nh.subscribe(sub_st);
}
 
void loop() {
  
  int val_0 = s0;
  int val_1 = s1;
  int val_2 = s2;
  int val_3 = s3;
  int val_4 = s4; // 1000 open, 1400 closed
  
  int pulse_0;
  int pulse_1;
  int pulse_2;
  int pulse_3;
  int pulse_4;

  // 轉換脈衝寬度
  pulse_0 = int(float(val_0) / 1000000 * FREQUENCY * 4096);
  pulse_1 = int(float(val_1) / 1000000 * FREQUENCY * 4096);
  pulse_2 = int(float(val_2) / 1000000 * FREQUENCY * 4096);
  pulse_3 = int(float(val_3) / 1000000 * FREQUENCY * 4096);
  pulse_4 = int(float(val_4) / 1000000 * FREQUENCY * 4096);
  
  // 讓伺服馬達移動到該位置 
  pwm.setPWM(0, 0, pulse_0);
  pwm.setPWM(1, 0, pulse_1);
  pwm.setPWM(2, 0, pulse_2);
  pwm.setPWM(3, 0, pulse_3);
  pwm.setPWM(4, 0, pulse_4);

  nh.spinOnce();
  delay(50);
}
