#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from states import *

def main():
    rospy.init_node('speaking_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Waiting', Waiting(), 
                               transitions={'exit':'Sleep', 
                                            'newspace':'QuickComment',
                                            'samespace':'LongComment'})

        smach.StateMachine.add('Sleep', Sleep(), 
                               transitions={'start':'Waiting'})

        smach.StateMachine.add('QuickComment', QuickComment(), 
                               transitions={'done':'Waiting',
                                            'exit':'Sleep'})

        smach.StateMachine.add('LongComment', LongComment(), 
                               transitions={'done':'Waiting',
                                            'exit':'Sleep'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()