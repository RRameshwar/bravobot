#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from states import *

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['timeout', 'found_person', 'error'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('sys_test', IsReady(), 
                               transitions={'yes':'Standby', 
                                            'no':'error'})
        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'timeout':'timeout',
                                            'button':'Calibrate'})
        smach.StateMachine.add('Calibrate', Calibrate(), 
                               transitions={'person':'Standby',
                                            'no_person':'Standby'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()