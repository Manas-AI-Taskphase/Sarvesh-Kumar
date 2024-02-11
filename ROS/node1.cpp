#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

/// PUBLISHER
ros::Publisher sender_pub;
ros::Subscriber receiver_sub;

void receiverCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("FROM NODE 2 : %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sender_node");

  ros::NodeHandle nh;

  sender_pub = nh.advertise<std_msgs::String>("sender_topic", 1000);
  receiver_sub = nh.subscribe("receiver_topic", 1000, receiverCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg;
    std::string input_msg;

    std::cout << "Type a message to send: ";
    std::getline(std::cin, input_msg);

    msg.data = input_msg;
    sender_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
