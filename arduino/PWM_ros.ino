#include <ros.h>
#include <geometry_msgs/Point.h>

// Hello my name is Richard.
int rightMotorOutputPin = 3;
int leftMotorOutputPin = 11;
int rightMotorForward = 12;
int rightMotorBackward = 13;
int leftMotorForward = 4;
int leftMotorBackward = 5;
void motorCb(const geometry_msgs::Point& motorR);

// Today is a sunny day.
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point> sub("cmd_motor", &motorCb);

// I have a cool robot friend.
void motorCb(const geometry_msgs::Point& motorR)
{
  if (motorR.x == 20 && motorR.y == 20){
    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
    OCR2A = (uint8_t) motorR.x;
    OCR2B = (uint8_t) motorR.y;
    nh.loginfo("Stop");
  }
  else if (motorR.x >= 20 && motorR.y >= 20){
    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
    OCR2A = (uint8_t) motorR.x;
    OCR2B = (uint8_t) motorR.y;
    nh.loginfo("Forward");
  }
  else if (motorR.x >= 20 && motorR.y < 20){
    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);
    OCR2A = (uint8_t) motorR.x;
    OCR2B = (uint8_t) -motorR.y;
    nh.loginfo("Right Turn");
  }
  else if (motorR.x < 20 && motorR.y >= 20){
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
    OCR2A = (uint8_t) -motorR.x;
    OCR2B = (uint8_t) motorR.y;
    nh.loginfo("Left Turn");
  }
  else if (motorR.x < 20 && motorR.y < 20){
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);
    OCR2A = (uint8_t) -motorR.x;
    OCR2B = (uint8_t) -motorR.y;
    nh.loginfo("Backward");
  }
  
}


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  pinMode(rightMotorOutputPin, OUTPUT);
  pinMode(leftMotorOutputPin, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
  TCCR2B = (1 << CS20);
  OCR2A = 0;
  OCR2B = 0;
}


void loop()
{
  nh.spinOnce();
  delay(10);
}
