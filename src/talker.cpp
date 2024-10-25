#include "ros/ros.h"
#include "std_msgs/String.h"
#include "auv_homework/SensorData.h"

#include <sstream>

int main(int argc, char **argv)
{
  // Create random sensor id between 0-127, srand to change seed every time
  srand(time(0));
  int sensor_id = rand() % 128;

  // init a ROS node
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // advertise a topic with msg_type of SensorData
  ros::Publisher pub = n.advertise<auv_homework::SensorData>("sensor_data", 10);

  // set rate to 10Hz, as told in instructions
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    auv_homework::SensorData data;

    data.id = sensor_id;
    // Create random sensor value between 0-1
    data.value = static_cast<double>(rand()) / RAND_MAX;

    // log the data before publishing
    ROS_INFO("sensor_id: %i, sensor_data: %f", data.id, data.value);
    pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}