#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <thread>
#include <sensor_msgs/Range.h>
std_msgs::Float32 distance_msg;
void ledCallback(const std_msgs::Float32::ConstPtr &msg)
{
    float distance = msg->data;
    ROS_INFO("Sonar Distance: %f cm", distance);
}

void subscriberThread()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("distance", 10, ledCallback);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
void publisherThread()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>("led_control", 10);
    while (ros::ok())
    {
        std_msgs::UInt8 msg;
        std::cout << "Input Led_Control: ";
        int input;
        std::cin >> input;
        msg.data = static_cast<int8_t>(input);

        pub.publish(msg);
        ros::Rate rate(10);
        ros::spinOnce();
        rate.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "multithreaded_node");
    ros::NodeHandle nh;
    std::thread subThread(subscriberThread);
    std::thread pubThread(publisherThread);
    ros::spin();
    subThread.join();
    pubThread.join();
    return 0;
}