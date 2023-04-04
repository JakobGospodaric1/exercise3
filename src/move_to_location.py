#! /usr/bin/env python3

#using interface move_base, and simpleactionserver, to move the robot to a goal location

import rospy

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Point, Pose, Quaternion

import time

def move_to_point(goal_location, goal_orientation):
    #send a goal location and orientation for the robot to reach
    goal_sent = False

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    #wait for the action server to come up
    client.wait_for_server()

    #create a goal message object
    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = 'map'

    #moving towards the goal*/
    goal.target_pose.header.stamp = rospy.Time.now()

    #set the goal position and orientation
    goal.target_pose.pose = Pose(goal_location, goal_orientation)

    #start moving
    client.send_goal(goal)

    # #allow TurtleBot up to 60 seconds to complete task
    # wait = client.wait_for_result(rospy.Duration(60))

    #while the robot hasn't reached the goal location or the time is up, keep looping
    start_time = time.time()

    while not client.get_state() == actionlib.GoalStatus.SUCCEEDED and not rospy.is_shutdown():
        #if time is up
        if time.time() - start_time > 60:
            rospy.logerr("Timed out achieving goal")
            client.cancel_goal()
            return

        #print out the current state of the robot
        state = client.get_state()
        rospy.loginfo("Current state: " + str(state))
        #sleep for 1 second
        time.sleep(1)

    rospy.loginfo("Goal execution done!")

    # #if the result doesn't arrive, assume the Server is not available
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    #     rospy.loginfo("Goal execution done!")
    
def move():
    points = [
        Point(0.6, -1.2, 0.0),
        Point(3, -0.2, 0.0),
        Point(1.74, 0.875, 0.0),
        Point(0.96, 2.69, 0.0),
        Point(-101, 1.97, 0.0),
    ]

    orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    for point in points:
        move_to_point(point, orientation)

if __name__ == '__main__':
    rospy.init_node('move_to_point', anonymous=False)
    move()
