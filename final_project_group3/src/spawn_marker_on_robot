#! /usr/bin/env python2.7

import time
import rospy
from math import pi
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


def get_model_pose(model_name):
    """
    Function to get the pose of the model.
    :param model_name: Name of the current model.
    :return: Return the model's pose.
    """
    poll_rate = rospy.Rate(1)
    for i in range(10):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, 1)
        if model_name in model_states.name:
            model_pose = model_states.pose[model_states.name.index(model_name)]
            break
        poll_rate.sleep()
    else:
        raise RuntimeError('Failed to get ' + model_name + ' model state')
    return model_pose


def main():
    """
    Main function for use when this file is called. Used to spawn the aruco marker on the robot.
    """
    rospy.init_node('spawn_marker_on_robot')
    time.sleep(10)
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        time.sleep(5)

    # Initialize service clients
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    spawn_sdf_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    link_attacher_client = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    link_attacher_client.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Initialize model names
    robot_name = rospy.get_namespace()[1:-1]
    # robot_name = 'leader'
    # rospy.logwarn(robot_name)
    marker_back_name = robot_name + '_marker_back'
    marker_front_name = robot_name + '_marker_front'

    # Compute marker pose based on current robot pose
    marker_back_pose = get_model_pose(robot_name)
    marker_back_pose.position.x -= 0.05
    marker_back_pose.position.z = 0.3
    marker_back_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, pi))

    # Compute marker pose based on current robot pose
    marker_front_pose = get_model_pose(robot_name)
    marker_front_pose.position.x -= 0.04
    marker_front_pose.position.z = 0.3
    marker_front_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

    # Compute spawn model request
    spawn_request_back = SpawnModelRequest()
    spawn_request_back.model_name = marker_back_name
    spawn_request_back.model_xml = """
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://aruco_marker</uri>
        </include>
      </world>
    </sdf>"""
    spawn_request_back.robot_namespace = robot_name
    spawn_request_back.initial_pose = marker_back_pose

    # Compute spawn model request
    spawn_request_front = SpawnModelRequest()
    spawn_request_front.model_name = marker_front_name
    spawn_request_front.model_xml = """
        <sdf version="1.6">
          <world name="default">
            <include>
              <uri>model://aruco_marker</uri>
            </include>
          </world>
        </sdf>"""
    spawn_request_front.robot_namespace = robot_name
    spawn_request_front.initial_pose = marker_front_pose

    # Spawn and attach marker to the robot
    pause_physics_client()
    spawn_sdf_model_client(spawn_request_back)
    spawn_sdf_model_client(spawn_request_front)
    link_attacher_client(robot_name, 'base_footprint', marker_back_name, 'link')
    link_attacher_client(robot_name, 'base_footprint', marker_front_name, 'link')
    unpause_physics_client()


if __name__ == '__main__':
    main()
