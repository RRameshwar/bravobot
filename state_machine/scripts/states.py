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
                return self.exit('timeout')s
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

    def exit(self, output):
        # shutdown state - on state end
        self.person_sub.unregister() #unregister subscriber
        return output

    def execute(self, userdata):
        # main function of calibrate state
        rospy.loginfo('Calibrating')
        self.state_init()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(20):
            if self.person:
                return self.exit('person') #on person found
            time.sleep(1)
        rospy.loginfo('Person Not Found')
        return self.exit('no_person') #on timeout


class PersonFollow(smach.State):
    """
    Person Follow state: activates person following.
    """
    def __init__(self):
        # initialize state - on system start
        smach.State.__init__(self, outcomes=['exit', 'timeout'])
        self.exit_cmd = False

    def on_exit_request(self, msg):
        # callback for person found
        self.exit_cmd = msg.data

    def state_init(self):
        # initialize state - on state start
        self.exit_cmd = False
        self.start_state = rospy.Publisher('/person_follow/start', Bool, queue_size=10)
        self.exit_sub = rospy.Subscriber('/person_follow/stop', Bool, self.on_exit_request)
        time.sleep(0.5) # give publisher time to initialize
        self.start_state.publish(Bool(True)) # publish to start calibrate nodes

    def exit(self, output):
        # shutdown state - on state end
        self.exit_sub.unregister() #unregister subscriber
        return output

    def execute(self, userdata):
        # main function of calibrate state
        rospy.loginfo('Following Person')
        self.state_init()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(120):
            if self.exit_cmd:
                return self.exit('exit')
            time.sleep(1)
        rospy.loginfo('exiting')
        return self.exit('timeout') #on timeout

class Wait(smach.State):
    """
    Wait state waits for a quick return of the master or times out.
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
        rospy.loginfo('Waiting')
        self.state_init()
        start_time = rospy.Time.now()
        while not self.button:
            if rospy.Time.now() - start_time > rospy.Duration(20):
                # on timeout
                return self.exit('timeout')
            time.sleep(1)
        return self.exit('button') # if button is pressed

class Complain(smach.State):
    """
    Complain about being turned off.
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
        print 'why are you turning me off?\n are you dissatisfied with your tour?'
        self.button = False

    def exit(self, output):
        # shutdown state - on state end
        self.button_sub.unregister() #unregister subscriber
        return output

    def execute(self, userdata):
        # state main function
        rospy.loginfo('poweroff initiated')
        self.state_init()
        start_time = rospy.Time.now()
        while not self.button:
            if rospy.Time.now() - start_time > rospy.Duration(10):
                # on timeout
                return self.exit('timeout')
            time.sleep(1)
        return self.exit('button') # if button is pressed