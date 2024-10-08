# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

import numpy as np
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseArray

class Blackboard(pt.behaviour.Behaviour):
    
    """
    A blackboard to store the state of the stages, shared among all nodes.
    """

    def __init__(self):
        rospy.loginfo("Initialize the stage manager.")

        # Initialize all stages with False
        self.stages = {

            # Localization param
            "initialised":    False,  # The robot has been initialised
            "localizing":     False,  # The robot is localizing

            # State param
            "converged":      False,  # AMCL has converged, the robot know where it is precisely
            "safe":           False,  # The robot is in a safe position, may achieve this zone when the robot is moving
            "kidnapped":      True,   # The robot has been kidnapped and doesn't know where it is

            # Task param
            "pick_pose":      False,
            "detect_cube":    False,
            "pick":           False,
            "place_pose":     False,
            "place":          False,
            
            # Final
            "cube_on_table2": False,
        }

    def set(self, stage, status):
        if stage in self.stages:
            self.stages[stage] = status
        else:
            rospy.logerr("Invalid stage!")
    
    def get(self, stage):
        if stage in self.stages:
            return self.stages[stage]
        else:
            rospy.logerr("Invalid stage!")


class convergence_checker(pt.behaviour.Behaviour):

    """
    Checks if AMCL has converged by thresholding the standard 
    deviation in both x and y direction.
    """

    def __init__(self, blackboard):

        rospy.loginfo("Initialize AMCL convergence checker.")

        self.blackboard = blackboard

        # Topics
        self.particlecloud_top = '/particlecloud'
        self.amcl_pose_top = '/amcl_pose'

        # Thresholds
        self.safe_threshold = 0.5
        self.converged_threshold = 0.05

        # Become a behaviour
        super(convergence_checker, self).__init__("Convergence checker")

    def update(self):

        x_stddev, y_stddev = self.get_stddev()

        if x_stddev is None:
            return pt.common.Status.FAILURE

        # Not converged and not safe
        if (x_stddev > self.safe_threshold or y_stddev > self.safe_threshold):
            if self.blackboard.get("localizing"):
                print_status = "localizing"
            else:
                self.blackboard.set("converged", False)
                self.blackboard.set("safe", False)
                self.blackboard.set("kidnapped", True)
                print_status = "kidnapped"

        # Converged and safe
        elif (x_stddev < self.converged_threshold and y_stddev < self.converged_threshold):
            
            # End of localizing
            if self.blackboard.get("localizing"):
                self.blackboard.set("localizing", False)

            self.blackboard.set("converged", True)
            self.blackboard.set("safe", True)
            self.blackboard.set("kidnapped", False)
            print_status = "converged"
        
        # Not converged but safe
        else:
            if self.blackboard.get("localizing"):
                print_status = "localizing"
            else:
                self.blackboard.set("converged", False)
                self.blackboard.set("safe", True)
                self.blackboard.set("kidnapped", False)
                print_status = "safe"
        
        # rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
        rospy.loginfo(f"Robot is {print_status}!")

        return pt.common.Status.SUCCESS
    
    def get_stddev(self):
        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]
        
        if len(rec_particles) == 0:
            rospy.loginfo("No particles received, cannot check convergence!")
            return None, None
        
        particles_array = np.array(rec_particles)
        x_stddev = np.std(particles_array[:, 0])
        y_stddev = np.std(particles_array[:, 1])

        return x_stddev, y_stddev

class update_localization(pt.behaviour.Behaviour):

    """
    Completes an update step for the amcl
    """

    def __init__(self, blackboard):

        rospy.loginfo("Initializing global and update localization.")

        self.blackboard = blackboard

        # Global localization service
        self.global_localization_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)

        # Update localization service
        self.update_localization_srv = rospy.ServiceProxy('/request_nomotion_update', Empty)
        rospy.wait_for_service('/request_nomotion_update', timeout=30)

        # become a behaviour
        super(update_localization, self).__init__("Update Robot Localization!")

    def update(self):

        # Get the state of the robot from blackboard
        initialised = self.blackboard.get("initialised")
        localizing = self.blackboard.get("localizing")
        converged = self.blackboard.get("converged")
        safe = self.blackboard.get("safe")
        kidnapped = self.blackboard.get("kidnapped")

        if localizing:
            self.update_localization_srv_req = self.update_localization_srv()
            status = pt.common.Status.RUNNING

        elif not initialised:
            rospy.loginfo("Robot not initialised yet!")
            global_localization_req = self.global_localization_srv()
            self.blackboard.set("localizing", True)
            status = pt.common.Status.SUCCESS
        
        elif kidnapped:
            rospy.loginfo("Robot kidnapped, should cancel goal now!")
            self.blackboard.set("initialised", False)
            status = pt.common.Status.SUCCESS
        
        elif not converged and safe:
            rospy.loginfo("Robot not converged, but safe!")
            # self.update_localization_srv_req = self.update_localization_srv()
            status = pt.common.Status.SUCCESS

        else:
            # converged and safe
            rospy.loginfo("Robot is converged and safe!")
            # self.update_localization_srv_req = self.update_localization_srv()
            status = pt.common.Status.SUCCESS
        
        return status

class sendgoal(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, operation, blackboard):
        rospy.loginfo("Initializing send goal")
        
        self.blackboard = blackboard

        # Set up action client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Connect to the action server
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()
        rospy.loginfo("%s: Connected to move_base action server")

        # Get the pose from the pose topic
        operation_pose_top = rospy.get_param(rospy.get_name() + '/' + operation + '_pose_topic')
        self.operation_pose = rospy.wait_for_message(operation_pose_top, PoseStamped, timeout=5)

        # Get the goal from the pose topic
        self.goal = MoveBaseGoal()
        self.goal.target_pose = self.operation_pose
        rospy.loginfo("Received goal pose for " + operation + " operation!")

        # Execution checker, Boolean to check the task status
        self.operation = operation
        self.sent = False   

        # Become a behaviour
        super(sendgoal, self).__init__(operation)

    def get_goal(self):
        """
        Goal sequence:
            "pick_pose"  
            "detect_cube"
            "pick"      
            "place_pose"
            "place"     
        """
        if self.blackboard.get("kidnapped"):
            return "cancel"
        elif not self.blackboard.get("pick_pose"):
            return "pick"
        elif not self.blackboard.get("detect_cube"):
            return "detect_cube"
        elif not self.blackboard.get("pick"):
            return "pick"
        elif not self.blackboard.get("place_pose"):
            return "place_pose"
        elif not self.blackboard.get("place"):
            return "place"

    def update(self):
        self.goal = self.get_goal()

        exclude_goals = ["detect_cube", "pick", "place"] # Send by other nodes

        # Cancel the goal when kidnapped
        if self.goal == "cancel":
            self.move_base_ac.cancel_goal()
            rospy.loginfo("Goal cancelled successfully")
            status = pt.common.Status.SUCCESS

        # Skip the goal if operation not matching with the goal
        elif self.goal in exclude_goals or self.operation != self.goal:
            status = pt.common.Status.SUCCESS

        # Skip the goal if already done
        elif self.blackboard.get(self.operation):
            status = pt.common.Status.SUCCESS

        # Operation matched with the goal and not yet done
        elif self.operation == self.goal and not self.blackboard.get(self.operation):
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_ac.send_goal(self.goal)
            self.sent = True
            status = pt.common.Status.RUNNING

        # Already sent the goal and not yet received the result
        elif self.sent:
            self.move_base_ac.wait_for_result()
            state = self.move_base_ac.get_state()

            if state == GoalStatus.ABORTED or state == GoalStatus.REJECTED:
                status = pt.common.Status.FAILURE
            elif state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                status = pt.common.Status.RUNNING
            elif state == GoalStatus.SUCCEEDED: # TODO: Check if this works
                self.blackboard.set(self.operation, True)
                status = pt.common.Status.SUCCESS
            else:
                status = pt.common.Status.RUNNING

        # Task is successful
        elif self.move_base_ac.get_result():
            self.blackboard.set(self.operation, True)
            status = pt.common.Status.SUCCESS

        return status
