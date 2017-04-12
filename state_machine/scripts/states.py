# Definition of the states for our state machine

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

class IsReady(smach.State):
    """
    State to check if the system is ready to function
    """
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
    """
    Standby state, exits on a button press or a timeout
    """
    def __init__(self):
        # initialie state - on system start
        smach.State.__init__(self, outcomes=['timeout', 'button'])
        self.button = False

    def on_button(self, msg):
        # button callback
        self.button = msg.data

    def state_init(self):
        # initialize state - on state start
        self.button_sub = rospy.Subscriber('/button', Bool, self.on_button)
        self.button = False

    def exit(self, output):
        # shutdown state - on state end
        self.button_sub.unregister() #unregister subscriber
        return output

    def execute(self, userdata):
        # state main function
        rospy.loginfo('entering Standby')
        self.state_init()
        start_time = rospy.Time.now()
        while not self.button:
            if rospy.Time.now() - start_time > rospy.Duration(20):
                # on timeout
                return self.exit('timeout')
            time.sleep(1)
        return self.exit('button') # if button is pressed
        

class Calibrate(smach.State):
    """
    Calibration state, starts up calibration node and returns either
    person found or no person found
    """
    def __init__(self):
        # initialize state - on system start
        smach.State.__init__(self, outcomes=['person', 'no_person'])
        self.person = False

    def on_person(self, msg):
        # callback for person found
        self.person = msg.data

    def state_init(self):
        # initialize state - on state start
        self.person = False
        self.start_calibrate = rospy.Publisher('/calibrator/start', Bool, queue_size=10)
        self.person_sub = rospy.Subscriber('/calibrator/stop', Bool, self.on_person)
        time.sleep(0.5) # give publisher time to initialize
        self.start_calibrate.publish(Bool(True)) # publish to start calibrate nodes

    def exit(self):
        # shutdown state - on state end
        self.start_calibrate.unregister() #unregister subscriber

    def execute(self, userdata):
        # main function of calibrate state
        rospy.loginfo('Calibrating')
        self.state_init()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(15):
            if self.person:
                return 'person' #on person found
            time.sleep(1)
        rospy.loginfo('Person Not Found')
        return 'no_person' #on timeout