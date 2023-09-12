#!/usr/bin/env python
"""
.. module:: planner_client
    :platform: Unix
    :synopsis: the planner_client python script in ontological_robot_control package

.. moduleauthor:: Donya MostaghniYazdi <donya.mostaghni01@gmail.com>

Subscribes to:
    /target_point

Publishes to:
    /path

Uses Action:
    /motion/planner

Gets the target point from ``finite_state_machine`` node through ``/target_point``
topic and sends it as an action goal to ``/motion/planner`` action server
"""
import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionClient
# Import the messages used by services and publishers.
from ontological_robot_control.msg import PlanGoal
# Import constant name defined to structure the architecture.
from ontological_robot_control import architecture_name_mapper as anm
import ontological_robot_control  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionClient`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER_CLIENT

pub = None

def planner_client_callback(data):
    """
    Subscriber callback function for ``/target_point`` topic, runs the ``planner_client()`` function

    Args:
        data(Point)
    """
    global pub
    plan_result = planner_client(data.target.x, data.target.y)
    # log_msg = 'Plan Result:'
    # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # print(plan_result)

    pub.publish(plan_result)

def planner_client(x, y):
    """
    Action client function for ``/motion/planner`` action server, sends the target point to the
    planner server

    Args:
        x(float)
        y(float)
    """
    # Create an action client called "planner_client" with action definition file "ontological_robot_control.msg.PlanAction"
    client = SimpleActionClient(anm.ACTION_PLANNER,ontological_robot_control.msg.PlanAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    log_msg = 'waiting for planner server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    # Sends the goal to the action server.
    client.send_goal(goal)
    log_msg = 'waiting for planner to find the path'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the plan is found before timeout
    if finished_before_timeout:
        log_msg = 'Plan found!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return client.get_result()
    else:
        log_msg = 'Action did not finish before time out!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        client.cancel_all_goals()

def main():
    """
    Initialise this node, defines subscriber to ``/target_point`` and publisher to 
    ``/path`` topic with ``planner_client_callback(data)`` function
    """
    global pub
    rospy.init_node(anm.NODE_PLANNER_CLIENT, log_level=rospy.INFO)
    rospy.Subscriber('/target_point', ontological_robot_control.msg.PlanGoal, planner_client_callback)
    pub = rospy.Publisher('/path', ontological_robot_control.msg.PlanResult, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
