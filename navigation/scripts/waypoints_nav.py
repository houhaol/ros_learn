#! /usr/bin/python

import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import *

def simple_move(x,y,z):


    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
    sac.wait_for_server()

    #create goal
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()
    #send goal
    sac.send_goal(goal)
    rospy.loginfo("send to goal")
    sac.wait_for_result()
    rospy.loginfo(sac.get_result())


if __name__ == '__main__':
    try:
        rospy.init_node('waypoints_move')

        #actually sending commands for the robot to travel
        f = [[1.3, 1.4, 0.2], [-1.3, -1.4, 0.4]]
        for coordinates in f:
			
			simple_move(float(coordinates[0]),float(coordinates[1]),float(coordinates[2]))
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
