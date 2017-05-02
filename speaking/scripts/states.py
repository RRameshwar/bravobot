# Definition of the states for our state machine

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool
from speaking.srv import PlayVoice

class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit','newspace','samespace'])
        self.enter_time = None
        self.start_time = None

    def init(self):
        self.loc_sub = rospy.Subscriber('/area_updates', String, self.locationCb)
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)
        self.locationChange = False
        self.enter_time = None
        self.start_time = rospy.Time.now()
        self.exit = False

    def sleep(self, outcome):
        self.loc_sub.unregister()
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        while not rospy.Time.now()-rospy.Duration(5) > self.start_time:
            if self.locationChange and rospy.Time.now()-rospy.Duration(1) > self.enter_time:
                return self.sleep('newspace')
            if self.exit:
                return self.sleep('exit')
        return ('samespace')

    def locationCb(self, msg):
        self.locationChange = True
        self.start_time = rospy.Time.now()
        self.enter_time = rospy.Time.now()

    def exitCb(self, msg):
        if self.msg:
            self.exit = True


class QuickComment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])
        sub = rospy.Subscriber('/area_updates', String, self.locCb)
        self.area = None
        self.exit = False
        self.start_time = None
        self.play_sound = rospy.ServiceProxy('play_sarcasm', PlayVoice)

    def init(self):
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)
        self.start_time = rospy.Time.now()

    def sleep(self, outcome):
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        print '\n\n', self.area, '\n\n'
        try:
            self.play_sound('short')
        except:
            pass
        while not rospy.Time.now()-rospy.Duration(3)>self.start_time:
            if self.exit:
                return self.sleep('exit')
        return self.sleep('done')

    def locCb(self, msg):
        self.area = msg.data

    def exitCb(self, msg):
        if self.msg:
            self.exit = True

class LongComment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])
        sub = rospy.Subscriber('/area_updates', String, self.locCb)
        self.area = None
        self.exit = False
        self.start_time = None
        self.play_sound = rospy.ServiceProxy('play_sarcasm', PlayVoice)

    def init(self):
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)
        self.start_time = rospy.Time.now()

    def sleep(self, outcome):
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        print self.area
        try:
            self.play_sound('long')
        except:
            pass
        while not rospy.Time.now()-rospy.Duration(10)>self.start_time:
            if self.exit:
                return self.sleep('exit')
        return self.sleep('done')

    def locCb(self, msg):
        self.area = msg.data

    def exitCb(self, msg):
        if self.msg:
            self.exit = True

class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def init(self):
        self.start_sub = rospy.Subscriber('start', Bool, self.startCb)
        self.start = False

    def sleep(self, outcome):
        self.start_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        while not rospy.is_shutdown():
            if self.start:
                return self.sleep('start')

    def startCb(self, msg):
        if self.msg:
            self.exit = True
