#! /usr/bin/env python

import rospy
import actionlib
from turtlebot3.turtlebot3_example import Turtlebot3Action, Turtlebot3Feedback, Turtlebot3Goal, Turtlebot3Result, Turlebot3ActionFeedback, Turtlebot3ActionResult
from geometry_msgs.msg import Twist
import time

class Turtlebot3Action(object):
    """docstring for Turtlebot3Action."""
    _feedback = Turlebot3ActionFeedback()
    _result = Turtlebot3ActionResult()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Turtlebot3Action, execute_cb=self.execute_cb, auto_start=False)

    def move(self, distance):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        time.sleep(distance)

    def execute_cb(self, goal):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()

        distance = goal.goal.x
        sucess = True
        self.move(distance)

        if sucess:
            self._result = 0
            rospy.loginfo('%s: succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node("turtlebot3_action_server")
    server = Turtlebot3Action(rospy.get_name())
    rospy.spin()
