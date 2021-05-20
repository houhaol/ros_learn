#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_srvs.srv import Empty, EmptyResponse


def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def service_callback(request):
    print "Robot pose"
    print current_pose
    return EmptyResponse

def main():
    current_pose = Pose()
    rospy.init_node("get_pose_server")
    sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    srv_server_handler = rospy.Service("/get_pose_service", Empty, service_callback)
    rospy.spin()
    

if __name__ == "__main__":
    main()