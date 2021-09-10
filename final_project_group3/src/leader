#!/usr/bin/env python2.7

import rospkg
import os
import rospy
import time
import yaml
import actionlib
import tf
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


rospack = rospkg.RosPack()
package = rospack.get_path('final_project_group3')
waypoints_path = os.path.join(package, 'yaml/waypoints.yaml')


def read_waypoints(waypoints_path="../yaml/waypoints.yaml"):
    """
    Interpret the waypoints listen in the given yaml file. Turn these into an OrderedDict that can be interpreted by
    this program.
    :param waypoints_path: Pathway to the waypoints file, located in the yaml folder.
    :return: An ordered dictionary of the goal locations
    :type waypoints_path: string
    :rtype: OrderedDict
    """
     
    def read_yaml(waypoints_path_):
        """
        Embedded function used to read the given waypoints file as a dictionary.
        :param waypoints_path_: Path to the waypoints file defined in the parent function.
        :return: A dictionary of the waypoint locations.
        :type waypoints_path_: string
        :rtype: dict
        """
        with open(waypoints_path_, 'r') as stream:
            try:
                waypoints_ = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return waypoints_
    print("Reading waypoints from ", waypoints_path)
    waypoints = read_yaml(waypoints_path)
    location_ = OrderedDict()

    for room in waypoints.keys():
        pose = []
        for pos in ['x', 'y', 'z']:
            pose.extend([waypoints[room]['position'][pos]])

        for quats in ['w', 'x', 'y', 'z']:
            pose.extend([waypoints[room]['orientation'][quats]])
        pose[-2:] = [1.0, 1.0]
        print(room, ':', pose)
        location_[room] = pose
    location_['end'] = None
    return location_


def leader_client(coordinates_):
    """
    Read coordinates and send the positional information to the leader robot.
    :param coordinates_: Read the current coordinates.
    :return: If the client is not waiting for a result, the client result is returned.
    :type coordinates_: A list taken from the read_waypoints function.
    """
    client = actionlib.SimpleActionClient('/leader/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(coordinates_[0])
    goal.target_pose.pose.position.y = float(coordinates_[1])
    goal.target_pose.pose.position.z = float(coordinates_[2])
    # goal.target_pose.pose.orientation.x = coordinates_[3]
    # goal.target_pose.pose.orientation.y = coordinates_[4]
    goal.target_pose.pose.orientation.z = coordinates_[5]
    goal.target_pose.pose.orientation.w = coordinates_[6]

    goal.target_pose.pose.orientation.z = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    rospy.init_node('leader', anonymous=True)
    locations = read_waypoints(waypoints_path)

    try:
        # Send info to command line and log the movement.
        for location, coordinates in locations.items():

            rospy.loginfo("Going to " + location)
            # print("location", location,"Coordinates:", coordinates)

            rospy.set_param('leader_goal', [location, coordinates])
            if coordinates is not None:
                result = leader_client(coordinates)
            # To verify if we had actually reached the goal:

            if result:
                rospy.loginfo(location + " Reached")

    except rospy.ROSInterruptException:
        rospy.loginfo("")
