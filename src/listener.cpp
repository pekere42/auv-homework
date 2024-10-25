#include "ros/ros.h"
#include "std_msgs/String.h"
#include "auv_homework/SensorData.h"

#include <sstream>

// to access the topic on callback function
ros::Publisher counter_pub;
// to keep track of message count
int msg_count = 0;

void sensorDataCallback(const auv_homework::SensorData::ConstPtr &data)
{
  msg_count++;

  std_msgs::String msg;
  std::stringstream ss;
  ss << "message count: " << msg_count;
  msg.data = ss.str();

  counter_pub.publish(msg);

  // log the data read from sensor_data topic, and the string published to message_count topic
  ROS_INFO("I heard sensor-%d published: %f", data->id, data->value);
  ROS_INFO("I published: %s", msg.data.c_str());
}

int main(int argc, char **argv)
{
  // init a ROS node
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // subscribe to sensor_data topic with our callback func
  ros::Subscriber sub = n.subscribe("sensor_data", 10, sensorDataCallback);

  // advertise the message_count topic with the msg_type of String
  counter_pub = n.advertise<std_msgs::String>("message_count", 10);

  ros::spin();
  return 0;
}
