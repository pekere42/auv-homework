#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from auv_homework.msg import SensorData


message_count = 0
pub = None


def callback(data):
    global message_count
    message_count += 1

    message_count_log = "message_count: %s" % message_count

    pub.publish(message_count_log)

    rospy.loginfo("I heard sensor-%s published: %s", data.id, data.value)
    rospy.loginfo("I published: %s", message_count_log)


def listener():
    global pub

    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("sensor_data", SensorData, callback)

    pub = rospy.Publisher("message_count", String, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    listener()
