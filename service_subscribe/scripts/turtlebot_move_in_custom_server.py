#! /usr/bin/env python

import rospy
from test.srv import turtleCustomServiceMessage, turtleCustomServiceMessageResponse
from move_turtle import moveTurtle

"""float64 side
int32 repetitions
---
bool success"""
def my_callback(request):
    move_object = moveTurtle()
    
    repetitions = request.repetitions + 1
    side = request.side
    for i in range(repetitions):
        move_object.move_square(side)
    
    rospy.loginfo("finished calling")
    response = turtleCustomServiceMessageResponse()
    response.success = True
    return response

def main():
    rospy.init_node("move_servcie")
    rospy.Service("/move_turtle_in_squre", turtleCustomServiceMessage, my_callback)    
    rospy.spin()


if __name__ == "__main__":
    main()