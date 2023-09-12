#!/usr/bin/env python
"""
.. module:: controller_client
    :platform: Unix
    :synopsis: the controller_client python script in ontological_robot_control package

.. moduleauthor:: Donya MostaghniYazdi <donya.mostaghni01@gmail.com>

Subscribes to:
    /path

Uses Action:
    /motion/controller

Gets the found path from ``planner`` node through ``/path`` topic and sends 
it as an action goal to ``/motion/controller`` action server
"""
import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionClient
# Import constant name defined to structure the architecture.
from ontological_robot_control import architecture_name_mapper as anm
import ontological_robot_control.msg  # This is required to pass the `ControlAction` type for instantiating the `SimpleActionClient`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER_CLIENT

def controller_client_callback(data):
    """
    Subscriber callback function for ``/path`` topic, runs the ``controller_client(goal)`` function

    Args:
        data(Point[])
    """
    control_result =  controller_client(data)
    # log_msg = 'Control Result:'
    # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # print(control_result)

# It uses the controller action server and cancels it if necessary.
def controller_client(goal):
    """
    Action client function for ``/motion/controller`` action server, sends the found path to the
    controller server

    Args:
        goal(Point[])
    """
    # Create an action client called "controller_client" with action definition file "ontological_robot_control.msg.ControlAction"
    client = SimpleActionClient(anm.ACTION_CONTROLLER,ontological_robot_control.msg.ControlAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    log_msg = 'waiting for controller server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # Sends the goal to the action server.
    client.send_goal(goal)
    log_msg = 'waiting for robot to reach the target'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the target is reached before timeout
    if finished_before_timeout:
        log_msg = 'Target Reached!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return client.get_result()
    else:
        log_msg = 'Action did not finish before time out!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        client.cancel_all_goals()

def main():
    """
    Main function for controller_client node, initialises the node and states that it 
    subscribes ``/path`` topic with ``controller_client_callback(data)`` function
    """
    # Initialise this node.
    rospy.init_node(anm.NODE_CONTROLLER_CLIENT, log_level=rospy.INFO)
    rospy.Subscriber('/path', ontological_robot_control.msg.PlanResult, controller_client_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
