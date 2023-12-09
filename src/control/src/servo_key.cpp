#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_S 0x73
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_Y 0x79
#define KEYCODE_Z 0x7A

class KeyboardReader
{
    public:
        KeyboardReader()
        #ifndef _WIN32
            : kfd(0)
        #endif
        {
        #ifndef _WIN32
            // get the console in raw mode
            tcgetattr(kfd, &cooked);
            struct termios raw;
            memcpy(&raw, &cooked, sizeof(struct termios));
            raw.c_lflag &=~ (ICANON | ECHO);
            // Setting a new line, then end of file
            raw.c_cc[VEOL] = 1;
            raw.c_cc[VEOF] = 2;
            tcsetattr(kfd, TCSANOW, &raw);
        #endif
        }
        void readOne(char * c){
            #ifndef _WIN32
                int rc = read(kfd, c, 1);
                if (rc < 0){
                    throw std::runtime_error("read failed");
                }
            #else
                for(;;){
                    HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
                    INPUT_RECORD buffer;
                    DWORD events;
                    PeekConsoleInput(handle, &buffer, 1, &events);
                    if(events > 0){
                        ReadConsoleInput(handle, &buffer, 1, &events);
                        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
                        {
                            *c = KEYCODE_LEFT;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
                        {
                            *c = KEYCODE_UP;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
                        {
                            *c = KEYCODE_RIGHT;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
                        {
                            *c = KEYCODE_DOWN;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x41)
                        {
                            *c = KEYCODE_A;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
                        {
                            *c = KEYCODE_B;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
                        {
                            *c = KEYCODE_C;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
                        {
                            *c = KEYCODE_D;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
                        {
                            *c = KEYCODE_E;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
                        {
                            *c = KEYCODE_F;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
                        {
                            *c = KEYCODE_G;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
                        {
                            *c = KEYCODE_Q;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
                        {
                            *c = KEYCODE_R;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x53)
                        {
                            *c = KEYCODE_S;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
                        {
                            *c = KEYCODE_T;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
                        {
                            *c = KEYCODE_V;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x57)
                        {
                            *c = KEYCODE_W;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x58)
                        {
                            *c = KEYCODE_X;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x59)
                        {
                            *c = KEYCODE_Y;
                            return;
                        }
                        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x5A)
                        {
                            *c = KEYCODE_Z;
                            return;
                        }
                    }
                }
            #endif
        }
        void shutdown(){
            #ifndef _WIN32
                tcsetattr(kfd, TCSANOW, &cooked);
            #endif
        }

    private:
        #ifndef _WIN32
            int kfd;
            struct termios cooked;
        #endif
};

KeyboardReader input;

class TeleopServo{
    public:
        TeleopServo();
        void keyLoop();

    private:
        ros::NodeHandle nh;
        double linear_, angular_, l_scale_, a_scale_;
        ros::Publisher servo_pub;
        ros::Publisher gripper_pub;
};

TeleopServo::TeleopServo():
    linear_(0),
    angular_(0),
    l_scale_(2.0),
    a_scale_(2.0)
{
    nh.param("scale_angular", a_scale_, a_scale_);
    nh.param("scale_linear", l_scale_, l_scale_);

    servo_pub = nh.advertise<geometry_msgs::Vector3>("cmd_tmr", 1);
    // gripper_pub = nh.advertise<geometry_msgs::Vector3>("cmd_gripper", 1);
}

void quit(int sig){
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "tmr_controller");
    TeleopServo teleop_servo;

    signal(SIGINT,quit);

    teleop_servo.keyLoop();
    quit(0);

    return(0);
}

float x, y, z;

void TeleopServo::keyLoop(){
    char c;
    bool servo = false;
    bool gripper = false;

    puts("Reading from keyboard");
    puts("---------------------------");

    for(;;){
        // get the next event from the keyboard  
        try{
            input.readOne(&c);
        }
        catch (const std::runtime_error &){
            perror("read():");
            return;
        }

        linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c){
            /*------------servo------------*/
            case KEYCODE_W:
                ROS_DEBUG("x+");
                x = 1;
                y = z = 0;
                servo = true;
                break;
            case KEYCODE_X:
                ROS_DEBUG("x-");
                x = -1;
                y = z = 0;
                servo = true;
                break;
            case KEYCODE_A:
                ROS_DEBUG("y+");
                y = 1;
                x = z = 0;
                servo = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("y-");
                y = -1;
                x = z = 0;
                servo = true;
                break;
            case KEYCODE_E:
                ROS_DEBUG("z+");
                z = 1;
                x = y = 0;
                servo = true;
                break;
            case KEYCODE_C:
                ROS_DEBUG("z-");
                z = -1;
                x = y = 0;
                servo = true;
                break;
            /*------------default------------*/
            case KEYCODE_S:
                ROS_DEBUG("BACK TO ORIGIN");
                x = y = z = 1010;
                servo = true;
                break;
        }

        geometry_msgs::Vector3 v3Euclidean;
        geometry_msgs::Vector3 v3g;

        v3Euclidean.x = x;
        v3Euclidean.y = y;
        v3Euclidean.z = z;
        if(servo == true){
            servo_pub.publish(v3Euclidean);    
            servo = false;
        }

        // v3g.x = s3;
        // v3g.y = s4;
        // v3g.z = 0;
        // if(gripper == true){
        //     gripper_pub.publish(v3g);    
        //     gripper = false;
        // }
    }


    return;
}



