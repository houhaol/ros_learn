#! /usr/bin/env python

import rospy
from rospy import client
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal, MoveBaseResult
import actionlib

def main():
    rospy.init_node("send_goals_client")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    for i in range(2):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = 0.4 +i
        goal.target_pose.pose.position.y = 0.8 *i
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.76 /i
        goal.target_pose.pose.orientation.w = 0.0
        client.send_goal(goal)
        rospy.loginfo("send to goal")
        client.wait_for_result()
        rospy.loginfo(client.get_result())


if __name__ == "__main__":
    main()