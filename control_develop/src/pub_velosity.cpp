#include <iostream>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <ros/ros.h>
#include "mav_msgs/Actuators.h"
std::mutex mut;
std::queue<int> data_queue;	// 用于线程间通信的队列
std::condition_variable data_cond;
bool flag = true;

void data_preparation_thread()
{
    while(flag)
    {
        int data;
        while(std::cin>>data)
        {
            data_queue.push(data);
            char ch = getchar();
            if(ch=='\n') break;
        }
        std::lock_guard<std::mutex> lk(mut);
        data_cond.notify_one();
        std::cout<<"If enter other state value? yes(y) or no(n)"<<std::endl;
        char c;
        std::cin>>c;
        if (c == 'y')
        {
            flag = true;
        }
        else
        {
            flag = false;
        }
    }
}

void data_processing_thread(ros::NodeHandle nh)
{
    ros::Publisher vel_pub = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed",1);
    while(flag)
    {
        std::unique_lock<std::mutex> lk(mut);
        data_cond.wait(lk,[]{return !data_queue.empty();});

        mav_msgs::Actuators message;
        int i = 0;
        double vel[6];
        while (!data_queue.empty())
        {
            vel[i] = data_queue.front();
            data_queue.pop();
            std::cout<<vel[i]<<" ";
            i++;
        }
        message.angular_velocities = {vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]};
        std::cout<<" Published."<<std::endl;
        lk.unlock();
        vel_pub.publish(message);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"velosity_publisher");
    ros::NodeHandle nh;
	std::cout << "Please input the velosity value of each motors,like 10 10 10 10 10 10: " << std::endl;
    std::thread t1(data_preparation_thread);
    std::thread t2(data_processing_thread, nh);
    
    t1.join();
    t2.join();
}