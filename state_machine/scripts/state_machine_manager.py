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
    sm = smach.StateMachine(outcomes=['timeout', 'test_end', 'exit', 'error'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('sys_test', IsReady(), 
                               transitions={'yes':'Standby', 
                                            'no':'error'})
        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'timeout':'timeout',
                                            'button':'PersonFollow'})
        smach.StateMachine.add('Calibrate', Calibrate(), 
                               transitions={'person':'PersonFollow',
                                            'no_person':'Standby'})
        smach.StateMachine.add('PersonFollow', PersonFollow(), 
                               transitions={'timeout':'Standby',
                                            'exit':'Complain'})
        smach.StateMachine.add('Wait', Wait(), 
                               transitions={'timeout':'Standby',
                                            'button':'PersonFollow'})
        smach.StateMachine.add('Complain', Complain(), 
                               transitions={'timeout':'exit',
                                            'button':'PersonFollow'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
