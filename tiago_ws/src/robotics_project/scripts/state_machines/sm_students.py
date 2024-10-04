#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 4:

            # State 0: Inspect surroundings
            if self.state == 0:
                rospy.loginfo("%s: Inspect_surroundings...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'inspect_surroundings'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(200.0))

                if success_tucking:
                    rospy.loginfo("%s: Inspect surroundings: ", self.play_motion_ac.get_result())
                    self.state = 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            # State 1: Pick service
            if self.state == 1:
                try:
                    rospy.loginfo("%s: Pick service", self.node_name)
                    pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                    pick_req = pick_srv()
                    
                    if pick_req.success == True:
                        self.state = 2
                        rospy.loginfo("%s: Pick up succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Pick up failed", self.node_name)
                        self.state = 5

                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to pick up failed: %s"%e)

            # State 2:  Move the robot to the target table
            if self.state == 2:
                move_msg = Twist()
                move_msg.angular.z = -2

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 15:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 7:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3
                rospy.sleep(1)

            # State 3: Place service
            if self.state == 3:
                try:
                    rospy.loginfo("%s: Place service", self.node_name)
                    place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                    place_req = place_srv()
                    
                    if place_req.success == True:
                        self.state = 4
                        rospy.loginfo("%s: Place succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Place failed", self.node_name)
                        self.state = 5

                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to place failed: %s"%e)
        
            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
