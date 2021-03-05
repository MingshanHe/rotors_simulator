#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <termios.h>

static struct termios initial_settings, new_settings;
static int peek_character = -1;

class keyboard_control
{
    public:
        keyboard_control()
        {
            init_keyboard();
        }
        ~keyboard_control(){};
    public:
        void init_keyboard();
        void close_keyboard();
        int kbhit();
        int readch();
    public:
        void main();
    private:
        ros::NodeHandle nh;
        ros::Publisher trajectory_pub;

        const float DEG_2_RAD = M_PI / 180.0;
        float pos_x = 0,pos_y = 0,pos_z = 1;
        float yaw_deg = 0;
        int isChange = 1;

};

void keyboard_control::init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag |= ICANON;
    new_settings.c_lflag |= ECHO;
    new_settings.c_lflag |= ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void keyboard_control::close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int keyboard_control::kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int keyboard_control::readch()
{
    char ch;

    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

void keyboard_control::main()
{
    ros::NodeHandle nh("//firefly");
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ROS_INFO("Start the way point publisher.");
    std::cout<<" q  w  e       q: up   w: forward  e: down"<<std::endl;
    std::cout<<" a  s  d       a: left s: stop     d: right"<<std::endl;
    std::cout<<"    x          x: back "<<std::endl;
    while(ros::ok())
    {
        if(kbhit())
        {
        int key = readch();
        switch(key)
        {
        case 113:
            pos_z += 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        case 101://2
            pos_z -= 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        case 97://a
            pos_y += 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        case 100://d
            pos_y -= 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        case 119://w
            pos_x += 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        case 120://x
            pos_x -= 0.5;
            isChange = 1;
            printf("%d\n",key);
            break;
        default:
            printf("%d\n",key);
            break;
        }
        }
        if(isChange == 1)
        {
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();

        Eigen::Vector3d desired_position(pos_x, pos_y,pos_z);

        double desired_yaw = yaw_deg * DEG_2_RAD;

        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);

        ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
                nh.getNamespace().c_str(),
                desired_position.x(),
                desired_position.y(),
                desired_position.z());

        trajectory_pub.publish(trajectory_msg);
        isChange = 0;
        }
        ros::spinOnce();
    }
}