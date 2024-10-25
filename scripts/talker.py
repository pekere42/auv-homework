#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from auv_homework.msg import SensorData
import random

sensor_id = random.randint(0, 127)


def talker():
    rospy.init_node("talker", anonymous=True)

    pub = rospy.Publisher("sensor_data", SensorData, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        data = SensorData(sensor_id, random.random())
        rospy.loginfo("sensor_id: %s, sensor_data: %s", data.id, data.value)
        pub.publish(data)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
