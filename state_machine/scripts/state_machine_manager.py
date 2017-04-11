#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

# define state Foo
class IsReady(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes','no'])

    def execute(self, userdata):
        rospy.loginfo('Asuming the robot is ready')
        ready = True
        if ready:
            return 'yes'
        else:
            return 'no'


# define state Bar
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['timeout', 'button'])
        self.button = False

    def on_button(self, msg):
        self.button = msg.data

    def state_init(self):
        self.button_sub = rospy.Subscriber('/button', Bool, self.on_button)
        self.button = False

    def exit(self, output):
        self.button_sub.unregister()
        return output

    def execute(self, userdata):
        rospy.loginfo('entering Standby')
        self.state_init()
        start_time = rospy.Time.now()
        while not self.button:
            if rospy.Time.now() - start_time > rospy.Duration(20):
                return self.exit('timeout')
            time.sleep(1)
        return self.exit('button')
        

class Calibrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['person', 'no_person'])
        self.person = False

    def on_person(self, msg):
        self.person = msg.data

    def state_init(self):
        self.start_calibrate = rospy.Publisher('/calibrate', Bool, queue_size=10)
        self.button_sub = rospy.Subscriber('/person', Bool, self.on_person)
        time.sleep(0.5)
        self.start_calibrate.publish(Bool(True))

    def exit(self):
        self.start_calibrate.unregister()

    def execute(self, userdata):
        rospy.loginfo('Calibrating')
        self.state_init()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(15):
            if self.person:
                return 'person'
            time.sleep(1)
        rospy.loginfo('Person Not Found')
        return 'no_person'


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
                               transitions={'person':'found_person',
                                            'no_person':'Standby'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()