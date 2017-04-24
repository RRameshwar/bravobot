# Definition of the states for our state machine

import time
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool

class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit','newspace','samespace'])

    def init(self):
        self.loc_sub = rospy.Subscriber('/test', String, self.locationCb)
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)
        self.locationChange = False
        self.exit = False

    def sleep(self, outcome):
        self.loc_sub.unregister()
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        while not rospy.is_shutdown():
            if self.locationChange:
                return self.sleep('newspace')
            if self.exit:
                return self.sleep('exit')

    def locationCb(self, msg):
        self.locationChange = True

    def exitCb(self, msg):
        if self.msg:
            self.exit = True


class QuickComment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])
        sub = rospy.Subscriber('/test', String, self.locCb)
        self.area = None
        self.exit = False

    def init(self):
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)

    def sleep(self, outcome):
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        print '\n\n', self.area, '\n\n'
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
        sub = rospy.Subscriber('/test', String, self.locCb)
        self.area = None
        self.exit = False

    def init(self):
        self.exit_sub = rospy.Subscriber('stop', Bool, self.exitCb)

    def sleep(self, outcome):
        self.exit_sub.unregister()
        return outcome

    def execute(self, userdata):
        self.init()
        print self.area
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