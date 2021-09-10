#!/usr/bin/env python2.7

import rospy
import tf2_ros
import tf
import time
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import geometry_msgs.msg


def callback(msg):
    """
    Callback function for use in the Subscriber. This will update a transform object with poses from the Odometry input
    and broadcast them on a defined broadcaster.
    :param msg: Input from Subscriber. This input is set to be the Odometry.
    :return: None
    :rtype: None
    """
    # instantiate a broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    # create a transforms object that can be broadcasted
    fiducial_frame = geometry_msgs.msg.TransformStamped()

    fiducial_frame.header.stamp = rospy.Time.now()  # set the time stamp
    fiducial_frame.header.frame_id = "follower_tf/camera_rgb_optical_frame"  # define the parent frame id
    fiducial_frame.child_frame_id = "fiducial_marker"  # define the child frame id
    # for all the incoming messages
    for m in msg.transforms:
        # read the pose from the message
        trans = m.transform.translation
        rot = m.transform.rotation
        # store the pose values in the transforms object we created
        fiducial_frame.transform.translation.x = trans.x
        fiducial_frame.transform.translation.y = trans.y
        fiducial_frame.transform.translation.z = trans.z
        fiducial_frame.transform.rotation.x = rot.x
        fiducial_frame.transform.rotation.y = rot.y
        fiducial_frame.transform.rotation.z = rot.z
        fiducial_frame.transform.rotation.w = rot.w
        # broadcast the transforms object
        broadcaster.sendTransform(fiducial_frame)
    # print("Broadcasted ...")


def listen_lookup_print():
    """
    Listener function that will create a listener which will listen to the frame positions of the map and the
    fiducial marker. Then, it will compute the transform between the two for use in moving the follower.
    :return: None
    :rtype: None
    """
    print("listening ...")
    listener = tf.TransformListener()  # instantiate a listener
    time.sleep(3)  # wait for a while
    rate = rospy.Rate(10)  # set sleep rate

    # lookup and print
    while not rospy.is_shutdown():
        try:
            # compute transformation world(map)-marker from camera-marker
            (trans, rot) = listener.lookupTransform('/map', 'fiducial_marker', rospy.Time(0))
            
            # print("x: ", trans[0], ", y: ", trans[1], ", r_w: ", rot[3])
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("Cant listen anything..")
            continue
        rate.sleep()


if __name__ == '__main__':

    rospy.init_node('fiducial_broadcaster', anonymous=True)
    # the subscriber reads  the fiducial transforms and broadcasts it
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)

    listen_lookup_print()
