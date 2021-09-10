#!/usr/bin/env python2.7

import tf
import rospy
import time
import rospkg
import actionlib
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_slerp
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
rospack = rospkg.RosPack()


def follower_client(goal_location_):
    """
    Client function for the follower robot.
    :param goal_location_: Goal location to be used in case the marker is lost.
    :return: Client for the follower
    """
    # instantiate a move_base action client
    action_client_ = actionlib.SimpleActionClient('/follower/move_base', MoveBaseAction)
    action_client_.wait_for_server()
    # instantiate a move_base goal message object
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # define the frame where action oocurs
    goal.target_pose.header.stamp = rospy.Time.now()  # time when execution happens
    # define the target location to reach
    goal.target_pose.pose.position.x = float(goal_location_[0])
    goal.target_pose.pose.position.y = float(goal_location_[1])
    # goal.target_pose.pose.position.z = 0.0
    # goal.target_pose.pose.orientation.x = goal_location_[-4]
    # goal.target_pose.pose.orientation.y = goal_location_[-3]
    goal.target_pose.pose.orientation.z = goal_location_[-2]
    goal.target_pose.pose.orientation.w = goal_location_[-1]
    # send the goal object to action client
    action_client_.send_goal(goal)
    return action_client_


def rotate(publisher, search_msg_, rate_, stopflag=False):
    """
    Function to spin the follower in place in case the marker is not found.
    :param publisher: Follower publisher.
    :param search_msg_: Search message for rotating that is called as Twist()
    :param rate_: Sleep rate
    :param stopflag: Boolean for stopping rotation. Default to False.
    :return: None
    :rtype: None
    """
    if not stopflag:
        search_msg_.angular.z = -1.5
    else:
        # print('stopping..')
        search_msg_.angular.z = 0.0
    publisher.publish(search_msg_)
    rate_.sleep()


def find_fiducial(listener_, margin_=0.7):
    """
    Function for finding the fiducial marker, and converting it to the follower's goal location.
    :param listener_: Listener that subscribes to the transform.
    :param margin_: Value used for a margin. Default to 0.7.
    :return: Return a tuple of the goal location.
    :rtype: tuple
    """
    # compute transformation world(map)-marker from camera-marker
    (trans, _) = listener_.lookupTransform('/map', 'fiducial_marker', rospy.Time(0))  # get trans from fiducial
    (_, rot) = listener.lookupTransform('/map', '/leader_tf/base_link', rospy.Time(0))  # get rot from leader

    rospy.loginfo("Following marker")

    # translation and rotation is available.. compute movebase here..
    goal_location_ = (trans[0] - margin_, trans[1] + margin_, trans[2], rot[0], rot[1], rot[2], rot[3])
    return goal_location_


def follower_at_goal(listener_, goal_location_, margin_=0.5):
    """
    Function for detecting if the follower is at the current goal location.
    :param listener_: Listener that subscribes to the transform.
    :param goal_location_: Location of the follower's goal for checking.
    :param margin_: Value used for a margin. Default to 0.4.
    :return: Return a bool for if the follower is at the goal.
    :rtype: bool
    """
    try:
        (trans, rot) = listener_.lookupTransform('/map', '/follower_tf/base_link', rospy.Time(0))
        if (abs(trans[0] - goal_location_[0]) < margin_) and (abs(trans[1] - goal_location_[1]) < margin_):
            # print(" Follower at GOAL: ", abs(trans[0] - goal_location_[0]), (abs(trans[1] - goal_location_[1])))
            return True
        else:
            return False
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return False


def explore(listener_):
    """
    Function called for if the follower cannot find the aruco marker and must explore for it and the current goal.
    :param listener_: Listener that subscribes to the transform.
    :return: None
    :rtype: None
    """
    # Find the current goal of the leader in the parameter server and reach it
    leader_goal_ = rospy.get_param("leader_goal")

    while True:
        try:
            goal_location_ = find_fiducial(listener_, margin_=0.5)
            rospy.loginfo("Marker found")
            break
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            goal_location_ = (leader_goal_[1])

        action_client_x = follower_client(goal_location_)
        wait_ = action_client_x.wait_for_result(rospy.Duration(0.3))
        if not wait_:
            # cancel goal and listen for new location
            action_client_x.cancel_goal()
            continue
        else:
            break


if __name__ == '__main__':

    rospy.init_node('follower', anonymous=True)
    margin = 0.5
    # velocity publisher
    search_msg = Twist()
    search_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=1)
    rotate_count = 0
    stop_rot = False
    result = False

    # listen to the lookup transform
    listener = tf.TransformListener()  # instantiate a listener
    time.sleep(2)  # wait for a while
    rate = rospy.Rate(5)  # set sleep rate

    # lookup and execute
    while not rospy.is_shutdown():
        leader_goal_ = rospy.get_param("leader_goal")
        # if we find the fiducials
        try:
            goal_location = find_fiducial(listener, margin)

            # DO NOT MOVE BASE IF THE GOAL IS CLOSE ENOUGH TO FOLLOWER - to avoid glitchy behavior
            if follower_at_goal(listener, goal_location, margin_=0.5):
                if leader_goal_[1] is None:
                    print("Goal Reached")
                    break
                rotate(search_publisher, search_msg, rate, stopflag=True)
                continue
            # instantiate an action client with the current goal
            action_client = follower_client(goal_location)

            # reset rotate states if we find the leader location
            rotate_count = 0
            stop_rot = False

            # wait for search_freq secs to reach the goal
            search_freq = rospy.Duration(0.5)
            wait = action_client.wait_for_result(search_freq)
            if not wait:
                # cancel goal and listen for new location
                action_client.cancel_goal()
                continue
            else:
                break

        # Detection for if the fiducials be found
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            # rotate 360 degrees -
            if stop_rot is False and rotate_count < 10:
                rospy.loginfo("Looking for marker")
                time.sleep(0.6)
                # publish angular velocity twist msg to leader/cmd_vel
                rotate(search_publisher, search_msg, rate)
                rotate_count += 1
                # must also look for fiducials when rotating, so `continue` back to try condition
                continue
            else:
                if rotate_count >= 10:
                    rotate(search_publisher, search_msg, rate, stopflag=True)
                stop_rot = True
                rotate_count = 0  # reset rotation state
                # continue
                explore(listener)

        rate.sleep()
