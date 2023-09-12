#!/usr/bin/env python
"""
.. module:: finite_state_machine
    :platform: Unix
    :synopsis: the finite_state_machine python script in ontological_robot_control package

.. moduleauthor:: Donya MostaghniYazdi <donya.mostaghni01@gmail.com>

Publishes to:
    /target_point

Uses Service:
    /state/set_battery_level

Uses helper script:
    /utilities/ontological_robot_control/topological_map.py

Defines the states and transitions for the finite state machine of the topological map, it also 
uses ``topological_map.py`` helper script to update the ontology while the process is running, and 
retreives the target room based on last visit times, finaly sends the target room pose to the 
``planner_client`` through ``/target_point`` in order to find the path
"""
from os.path import dirname, realpath
import rospy
import smach
import smach_ros
import time
import ontological_robot_control
from ontological_robot_control.msg import PlanGoal
from ontological_robot_control.srv import SetBatteryLevel
from ontological_robot_control import architecture_name_mapper as anm
from ontological_robot_control.topological_map import TopologicalMap

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 2

pub = None
tm = None

def send_room_pose(room):
    """
    Sends target room's corresponding position by calling ``send_pose(x,y)`` function

    Args:
        room(string)
    """
    log_msg = 'Received request for robot to move to ' + room
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    if room   == 'E':
        send_pose(3.5, 1.0)
    elif room == 'R1':
        send_pose(1.0, 4.5)
    elif room == 'R2': 
        send_pose(1.0, 7.5)
    elif room == 'R3':
        send_pose(9.0, 4.5) 
    elif room == 'R4':
        send_pose(9.0, 7.5) 
    elif room == 'C1':
        send_pose(3.0, 4.5) 
    elif room == 'C2':
        send_pose(6.5, 4.5)

def send_pose(x, y):
    """
    Publishes a target pose value to ``planner_client`` through ``/target_point`` topic 

    Args:
        x(float)
        y(float)
    """
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    pub.publish(goal)
    log_msg = 'Request sent to planner server'
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


def _set_battery_level_client(battery_level):
    """
    Service client function for ``/state/set_battery_level`` Update the current robot battery level
    stored in the ``robot-state`` node

    Args:
        battery_level(int)
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_BATTERY_LEVEL)
    try:
        # Log service call.
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(battery_level)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

# define state Room E
class RoomE(smach.State):
    """
    defines the state when robot is in room "E"
    """
    def __init__(self):

        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D6','D7', 'stay'])

    def execute(self, userdata):
        """
        Since the charger is in room "E" charges the robot using ``/state/set_battery_level`` service
        performs the ontology update and sends target room pose to the ``planner_client``,
        finally considering the target room, chooses one of the transistions(doors) by returning 
        the door's name string

        Args:
            userdata(None)
            
        Returns:
            "D7" or "D6"(string)
        """
        _set_battery_level_client(20)
        log_msg = f'Battery Charged.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "C1" or target_room == "R1" or target_room == "R2":
            send_room_pose('C1')
            return 'D7'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            send_room_pose('C2')
            return 'D6'
        else:
            return 'stay'

# define state Room 1
class Room1(smach.State):
    """
    Defines the state when robot is in room "R1"
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D1', 'stay'])
        
    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``,
        finally if the target room is different from current room (R1) it returns the only door's name (D1)

        Args:
            userdata(None)
        
        Returns:
            "D1"(string)
        """
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R1":
            send_room_pose('C1')
            return 'D1'
        else:
            return 'stay'

# define state Room 2
class Room2(smach.State):
    """
    Defines the state when robot is in room "R2"
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D2', 'stay'])
        
    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``,
        finally if the target room is different from current room (R2) it returns the only door's name (D2)

        Args:
            userdata(None)
        
        Returns:
            "D2"(string)
        """
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R2":
            send_room_pose('C1')
            return 'D2'
        else:
            return 'stay'

# define state Room 3
class Room3(smach.State):
    """
    Defines the state when robot is in room "R3"
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['D3', 'stay'])
        
    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``
        ,finally if the target room is different from current room (R3) it returns the only door's name (D3)

        Args:
            userdata(None)
        
        Returns:
            "D3"(string)
        """
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R3":
            send_room_pose('C2')
            return 'D3'
        else:
            return 'stay'

# define state Room 4
class Room4(smach.State):
    """
    Defines the state when robot is in room "R4"
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['D4', 'stay'])
        
    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``, 
        finally if the target room is different from current room (R4) it returns the only door's name (D4)

        Args:
            userdata(None)
        
        Returns:
            "D4"(string)
        """
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room != "R4":
            send_room_pose('C2')
            return 'D4'
        else:
            return 'stay'
    
# define state Corridor 1
class Corridor1(smach.State):
    """
    Defines the state when robot is in corridor "C1"
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['D1','D2','D5','D7','stay'])

    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``, 
        finally considering the target room, chooses one of the transistions(doors) by returning the
        door's name string

        Args:
            userdata(None)
        
        Returns:
            "D1" or "D2" or "D5" or "D7"(string)
        """
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "R1":
            send_room_pose('R1')
            return 'D1'
        elif target_room == "R2":
            send_room_pose('R2')
            return 'D2'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            send_room_pose('C2')
            return 'D5'
        elif target_room == "E":
            send_room_pose('E')
            return 'D7'
        else:
            return 'stay'

# define state Corridor 2
class Corridor2(smach.State):
    """
    Defines the state when robot is in corridor "C2"
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['D3','D4','D5','D6','stay'])

    def execute(self, userdata):
        """
        Performs the ontology update and sends target room pose to the ``planner_client``, 
        finally considering the target room, chooses one of the transistions(doors) by returning the
        door's name string

        Args:
            userdata(None)
        
        Returns:
            "D3" or "D4" or "D5" or "D6"(string)
        """        
        time.sleep(LOOP_TIME)
        now = rospy.get_rostime()
        target_room = tm.update_ontology(now)
        if target_room == "R3":
            send_room_pose('R3')
            return 'D3'
        elif target_room == "R4":
            send_room_pose('R4')
            return 'D4'
        elif target_room == "C1" or target_room == "R1" or target_room == "R2":
            send_room_pose('C1')
            return 'D5'
        elif target_room == "E":
            send_room_pose('E')
            return 'D6'
        else:
            return 'stay'
        
def main():
    """
    The main function for finite_state_machine node, initialises the node and takes an instance of
    TopologicalMap class in the time instance now, defines the publisher to the ``/target_point`` topic
    , defines the states and transitions of the finite state machine for topological map and finally 
    starts the finite state machine process
    """
    # Initialise this node.
    global tm
    global pub

    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    now = rospy.get_rostime()
    tm = TopologicalMap(LOG_TAG, now)

    # Publish target point to planner
    pub = rospy.Publisher('/target_point', ontological_robot_control.msg.PlanGoal, queue_size=10)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('E', RoomE(), transitions={'D7':'C1', 'D6':'C2', 'stay':'E'})
        smach.StateMachine.add('C1', Corridor1(), transitions={'D7':'E', 'D1':'R1', 'D2':'R2', 'D5':'C2', 'stay':'C1'})
        smach.StateMachine.add('C2', Corridor2(), transitions={'D6':'E', 'D3':'R3', 'D4':'R4', 'D5':'C1', 'stay':'C2'})
        smach.StateMachine.add('R1', Room1(), transitions={'D1':'C1', 'stay':'R1'})
        smach.StateMachine.add('R2', Room2(), transitions={'D2':'C1', 'stay':'R2'})
        smach.StateMachine.add('R3', Room3(), transitions={'D3':'C2', 'stay':'R3'})
        smach.StateMachine.add('R4', Room4(), transitions={'D4':'C2', 'stay':'R4'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
