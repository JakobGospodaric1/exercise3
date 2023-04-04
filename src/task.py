#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

import message_filters

#position and orientation
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Vector3, Pose, Quaternion, PoseStamped

from visualization_msgs.msg import Marker, MarkerArray

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf2_geometry_msgs
import tf2_ros

import tf.transformations as tft

from transforms3d import quaternions

import math
import numpy as np

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

depth_image = None
depth_msh = None

#array of markers
marker_array = []
arrayMarker = MarkerArray()

marker_pub = None

marker_id = 0

# Object we use for transforming between coordinate frames
tf_buf = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buf)

# Define a callback for the Image message

global_counter = 0

def get_pose(coords,dist,stamp, rgb_image):
    # if the time of the depth image is more than 0.01 seconds before the rgb image, then the depth image is too old
    

    # Calculate the position of the detected face

    k_f = 554 # kinect focal length in pixels

    x1, x2, y1, y2 = coords

    dims = rgb_image.shape
    # rospy.loginfo(dims)

    face_x = dims[1] / 2 - (x1+x2)/2.
    face_y = dims[0] / 2 - (y1+y2)/2.

    angle_to_target = np.arctan2(face_x,k_f)

    # Get the angles in the base_link relative coordinate system
    x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

    # Define a stamped message for transformation - in the "camera rgb frame"
    point_s = PointStamped()
    point_s.point.x = -y
    point_s.point.y = 0
    point_s.point.z = x
    point_s.header.frame_id = "camera_rgb_optical_frame"
    point_s.header.stamp = stamp

    # Get the point in the "map" coordinate system
    try:
        point_world = tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z
    except Exception as e:
        print(e)
        pose = None

    return pose

def make_marker(face_position):
    #rospy.loginfo(face_position)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    global marker_id
    marker_id += 1
    marker.id = marker_id
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.frame_locked = False

    marker.pose = face_position

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # marker_pub.publish(marker)

    return marker

def find_faces_callback(img_msg, depth_msg):
    global global_counter
    rospy.loginfo(global_counter)
    global_counter += 1

    try:
        rgb_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    # Convert the images into a OpenCV (numpy) format
    try:
        rgb_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    #detect faces
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    for (x,y,w,h) in faces:
        cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(255,0,0),2)
        #log depth
        depth = 0
        i = 0

        depth = float(np.nanmean(depth_image[y:y+h, x:x+w]))
        
        # Get the time that the depth image was recieved
        depth_time = depth_msg.header.stamp

        pose = get_pose((x, x+w, y, y+h), depth, depth_time, rgb_image)

        if pose is None:
            break

        marker = make_marker(pose)

        # global marker_array
        global arrayMarker

        arrayMarker.markers.append(marker)

        #go trough all markers and group them together if they are close to each other
        del_list = np.array([])
        for i in range(len(arrayMarker.markers)):
            for j in range(i+1, len(arrayMarker.markers)):
                length = 1.5
                if abs(arrayMarker.markers[i].pose.position.x - arrayMarker.markers[j].pose.position.x) < length and abs(arrayMarker.markers[i].pose.position.y - arrayMarker.markers[j].pose.position.y) < length:
                    #make new position the average of the old and new position
                    arrayMarker.markers[i].pose.position.x = (arrayMarker.markers[i].pose.position.x + arrayMarker.markers[j].pose.position.x) / 2
                    arrayMarker.markers[i].pose.position.y = (arrayMarker.markers[i].pose.position.y + arrayMarker.markers[j].pose.position.y) / 2
                    arrayMarker.markers[i].pose.position.z = (arrayMarker.markers[i].pose.position.z + arrayMarker.markers[j].pose.position.z) / 2
                    del_list = np.append(del_list, j)

        #pop the old markers
        for i in range(len(del_list)):
            arrayMarker.markers.pop(del_list[i])

        # arrayMarker.markers.append(marker)

        marker_pub.publish(arrayMarker)

        # rospy.loginfo(arrayMarker)

    # Show the converted image
    cv2.startWindowThread()
    cv2.imshow("Image Window", rgb_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    # sub_image_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Initalize a subscriber to the "/camera/depth/image_raw" topic with the function "image_callback" as a callback
    # sub_image_depth = rospy.Subscriber("/camera/depth/image_raw", Image, image_depth)

    # Publisher for the markers
    marker_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)

    # Message filering for synchronizing the rgb and depth images
    rgm_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

    synchronizer = message_filters.ApproximateTimeSynchronizer([rgm_sub, depth_sub], 10, 0.1)
    # synchronizer = message_filters.TimeSynchronizer([rgm_sub, depth_sub], 10)
    synchronizer.registerCallback(find_faces_callback)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    # rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.spin()
        # rate.sleep()

    cv2.destroyAllWindows()