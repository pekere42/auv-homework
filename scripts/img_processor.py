#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

gray_pub = None
edge_pub = None
custom_pub = None


font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_thickness = 2
line_type = cv2.LINE_AA
custom_text = "ITU AUV"

logo_width = 200
logo_height = 70
script_dir = os.path.dirname(os.path.abspath(__file__))
logo_path = os.path.join(script_dir, "../assets/auvLogo.png")
logo_img = cv2.imread(logo_path, cv2.IMREAD_UNCHANGED)
logo_img = cv2.resize(logo_img, (logo_width, logo_height))


def callback(data):
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # make the image grayscale (task 1)
    gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    pub_gray_frame = br.cv2_to_imgmsg(gray_frame)
    gray_pub.publish(pub_gray_frame)

    # run the Canny edge detection algorithm on the grayscale image (task 2)
    edge_frame = cv2.Canny(gray_frame, 100, 200)
    pub_edge_frame = br.cv2_to_imgmsg(edge_frame)
    edge_pub.publish(pub_edge_frame)

    # get text size and position it on the top center of the image
    (text_width, text_height), _ = cv2.getTextSize(
        custom_text, font, font_scale, font_thickness
    )
    image_height, image_width = current_frame.shape[:2]
    x = (image_width - text_width) // 2

    # put the text on the image
    custom_frame = cv2.putText(
        current_frame,
        custom_text,
        (x, text_height + 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        font_thickness,
        line_type,
    )

    # copy the frame to add the logo
    copied_frame = custom_frame.copy()

    x_offset = (image_width - logo_width) // 2
    y_offset = image_height - logo_height - 10

    # add the logo to the image using alpha blending
    bgr_logo = logo_img[:, :, :3]
    alpha_logo = logo_img[:, :, 3] / 255.0
    for c in range(0, 3):
        copied_frame[
            y_offset : y_offset + logo_height, x_offset : x_offset + logo_width, c
        ] = (
            alpha_logo * bgr_logo[:, :, c]
            + (1 - alpha_logo)
            * copied_frame[
                y_offset : y_offset + logo_height,
                x_offset : x_offset + logo_width,
                c,
            ]
        )

    # change color space from BGR to RGB
    rgb_image = cv2.cvtColor(copied_frame, cv2.COLOR_BGR2RGB)
    pub_custom_frame = br.cv2_to_imgmsg(rgb_image, encoding="bgr8")
    custom_pub.publish(pub_custom_frame)


def listener():
    global gray_pub, edge_pub, custom_pub

    # initialize the node
    rospy.init_node("img_processor", anonymous=True)

    # create a subscriber for the camera image
    rospy.Subscriber("usb_cam/image_raw", Image, callback)

    # create publishers for the processed images
    gray_pub = rospy.Publisher("/processed_image/gray", Image, queue_size=30)
    edge_pub = rospy.Publisher("/processed_image/edge", Image, queue_size=30)
    custom_pub = rospy.Publisher("/processed_image/custom", Image, queue_size=30)

    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
