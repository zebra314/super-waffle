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

class counter(pt.behaviour.Behaviour):

    """
    Returns failure for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class wait(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising wait behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(wait, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.RUNNING if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class movecube(pt.behaviour.Behaviour):

    """
    Pick or place the cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, operation):

        rospy.loginfo("Initialising move cube behaviour.")

        # server
        mv_cube_srv = '/' + operation + '_srv'
        mv_cube_srv_nm = rospy.get_param(rospy.get_name() + mv_cube_srv)
        self.move_cube_srv = rospy.ServiceProxy(mv_cube_srv_nm, SetBool)
        rospy.wait_for_service(mv_cube_srv_nm, timeout=30)

        self.operation = operation

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movecube, self).__init__("GET EM!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_cube_req = self.move_cube_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_cube_req.success:
            rospy.loginfo("Robot finished: " + self.operation + " the cube!")
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_cube_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class detectcube(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising detect cube behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server")
            exit()
        rospy.loginfo("%s: Connected to play_motion action server")

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'inspect_surroundings' # Try to detect the cube from the surroundings
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(detectcube, self).__init__("Detect cube!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class cube_detected_on_table2(pt.behaviour.Behaviour):

    """
    Returns if cube is placed on table 2.
    """

    def __init__(self):

        rospy.loginfo("Initialising placement detection of cube on table 2.")

        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.detected_cube = False
        self.listened = False

        # become a behaviour
        super(cube_detected_on_table2, self).__init__("Is_job_done")

    def update(self):

        # If listened return result
        if self.listened:
            if self.detected_cube:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

        # try if not tried
        elif not self.listened:

            try: 
                rospy.loginfo("Checking if job is done!")
                detected_cube_pose_msg = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=10)
                rospy.loginfo("Detected cube pose on table 2 %s!", detected_cube_pose_msg)
                self.detected_cube = True
                rospy.loginfo("Job is done, I nailed it!")
            except: 
                rospy.loginfo("Cube not detected on table 2!")
                rospy.loginfo("Job not done, damn it!")
            
            self.listened = True
            # tell the tree you're running
            return pt.common.Status.RUNNING

class resetpose(pt.behaviour.Behaviour):

    """
    Resets the pose of the robot and cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising the resetting of the pose of the robot and cube.")
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  # Create a service proxy

        self.robot_state = self.get_robot_state()
        self.cube_state = self.get_cube_state()

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(resetpose, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.reset_robot_req = self.reset_srv(self.robot_state)
            self.reset_cube_req = self.reset_srv(self.cube_state)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.reset_robot_req.success and self.reset_cube_req.success:
            rospy.loginfo("Resetting of the pose of the robot and cube succeeded!.")
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.reset_robot_req.success or not self.reset_cube_req.success:   
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

    def get_robot_state(self):
        robot_state = ModelState()
        robot_state.model_name = 'tiago'
        robot_state.pose.position.x = -1.146
        robot_state.pose.position.y = -6.155
        robot_state.pose.position.z = -0.001
        robot_state.pose.orientation.x = 0
        robot_state.pose.orientation.y = 0
        robot_state.pose.orientation.z = -0.7149132
        robot_state.pose.orientation.w = 0.6992132
        robot_state.twist.linear.x = 0
        robot_state.twist.linear.y = 0
        robot_state.twist.linear.z = 0
        robot_state.twist.angular.x = 0
        robot_state.twist.angular.y = 0
        robot_state.twist.angular.z = 0
        robot_state.reference_frame = 'map'
        return robot_state
    
    def get_cube_state(self ):
        cube_state = ModelState()
        cube_state.model_name = 'aruco_cube'
        cube_state.pose.position.x = -1.130530
        cube_state.pose.position.y = -6.653650
        cube_state.pose.position.z = 0.86250
        cube_state.pose.orientation.x = 0
        cube_state.pose.orientation.y = 0
        cube_state.pose.orientation.z = 0
        cube_state.pose.orientation.w = 1
        cube_state.twist.linear.x = 0
        cube_state.twist.linear.y = 0
        cube_state.twist.linear.z = 0
        cube_state.twist.angular.x = 0
        cube_state.twist.angular.y = 0
        cube_state.twist.angular.z = 0
        cube_state.reference_frame = 'map'
        return cube_state

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
            self.update_localization_srv_req = self.update_localization_srv()
            status = pt.common.Status.SUCCESS

        else:
            # converged and safe
            rospy.loginfo("Robot is converged and safe!")
            self.update_localization_srv_req = self.update_localization_srv()
            status = pt.common.Status.SUCCESS
        
        return status

class kidnap_checker(pt.behaviour.Behaviour):

    """
    Checks if robot has been kiddnapped!
    """

    def __init__(self, sendgoal):

        rospy.loginfo("Initialising kindap checker.")

        self.particlecloud_top = '/particlecloud'

        self.amcl_pose_top = '/amcl_pose'

        self.std_position_threshold = 0.4

        self.last_position_and_yaw = None

        self.kidnapped = False

        self.sendgoal = sendgoal

        super(kidnap_checker, self).__init__("Kidnap checker ?")

    def update(self):

        # Is the robot kidnapped? 
        if self.kidnapped:
            return pt.common.Status.SUCCESS

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        #rospy.loginfo("Cloud particles received, checking convergence!")
        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        # Obtain pose estimate with covariance matrix
        pose_estimate = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, timeout=5)

        Cov_mat = pose_estimate.pose.covariance
        #Cov_mat_np = np.array(Cov_mat).reshape(6,6)
        # Extract the position and orientation covariances (x, y, and yaw)
        x_cov = Cov_mat[0]    
        y_cov = Cov_mat[7]    
        yaw_cov = Cov_mat[35]

        new_position_and_yaw = x_cov + y_cov + yaw_cov
        #new_det_cov = np.linalg.det(Cov_mat_np)
        if self.last_position_and_yaw is None:
            self.last_position_and_yaw = new_position_and_yaw
            rospy.loginfo(f'Initial covariance {new_position_and_yaw}')
            return pt.common.Status.RUNNING

        change_in_position_and_yaw = new_position_and_yaw - self.last_position_and_yaw
        rospy.loginfo(f"Change in P_xx*P_yy*P_yawyaw: {change_in_position_and_yaw}")
        #rospy.loginfo(f'Covariance in x: {x_cov}, y: {y_cov} yaw:{yaw_cov} ')
        if abs(change_in_position_and_yaw) > 2e-2:
            rospy.loginfo('-------------------------------------------------------')
            rospy.loginfo('Robot kidnapped!!!!')
        #rospy.loginfo(f'Determinant of Covariance Matrix: {new_det_cov}')
        #rospy.loginfo(f"Change in det P: {abs(new_det_cov - self.det_cov)}") 
        
        self.last_position_and_yaw = new_position_and_yaw
        #self.det_cov = new_det_cov

        if len(rec_particles) > 0:
            self.check_convergence(rec_particles)


        # tell the tree you've failed, you sucker!!
        return pt.common.Status.FAILURE
            
    def check_convergence(self, particles):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")

        # Check if both x and y standard deviations are below the threshold
        if (x_stddev + y_stddev > self.std_position_threshold):
            #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")
            rospy.loginfo("Robot is kidnapped, should cancel goal now!")
            self.kidnapped = True
            self.sendgoal.finished = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
            #rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
            #rospy.loginfo("Initial localization is not successful yet.")
    
    def get_kidnapped_state(self):
        return self.kidnapped

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

        exclude_goals = ["detect_cube", "pick", "place"]

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
            elif state == GoalStatus.SUCCEEDED:
                self.blackboard.set(self.operation, True)
                status = pt.common.Status.SUCCESS
            else:
                status = pt.common.Status.RUNNING

        # Task is successful
        elif self.move_base_ac.get_result():
            self.blackboard.set(self.operation, True)
            status = pt.common.Status.SUCCESS

        return status

class gather_cues(pt.behaviour.Behaviour):

    """
    Rotate to gather cues.
    """

    def __init__(self, angular, count_max):

        rospy.loginfo("Initialising gather cues behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.angular.z = angular

        self.count_max = count_max
        self.counter = 0

        # become a behaviour
        super(gather_cues, self).__init__("Gather cues")

    def update(self):
        
        if self.counter == 0:
            rospy.loginfo("Rotating to gather cues!")
        if self.counter >= self.count_max:
            return pt.common.Status.SUCCESS
        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()
        self.counter += 1

        # tell the tree that you're running
        return pt.common.Status.RUNNING 
