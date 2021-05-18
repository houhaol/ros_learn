#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rospy import client
from turtlebot3.turtlebot3_example import Turtlebot3Action, Turtlebot3Feedback, Turtlebot3Goal, Turtlebot3Result
import actionlib


def main():
    rospy.init_node("turtlebot_action_client")
    client = actionlib.SimpleActionClient("turtlebot3_action_server", Turtlebot3Action)
    client.wait_for_server()

    goal = Turtlebot3Goal()
    goal.goal.x = 10
    client.send_goal(goal)
    rospy.loginfo("send to goal")
    client.wait_for_result()

    rospy.loginfo(client.get_result())
    

if __name__ == "__main__":
    main()

