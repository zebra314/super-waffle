# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

import numpy as np
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient, GoalID
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
            rospy.loginfo('Detecting cube now!')
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

    def __init__(self, mission_done =  None, mode = 0):

        rospy.loginfo("Initialising placement detection of cube on table 2.")

        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.detected_cube = False
        self.listened = False

        self.mission_done = mission_done
        self.mode = mode

        # become a behaviour
        super(cube_detected_on_table2, self).__init__("Is_job_done")

    def update(self):

        # If listened return result
        if self.listened:
            if self.detected_cube:
                return pt.common.Status.SUCCESS
            elif (not self.detected_cube) and self.mode == 0:
                return pt.common.Status.FAILURE
            elif (not self.detected_cube) and self.mode == 1:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.listened:

            try: 
                rospy.loginfo("Checking if job is done!")
                detected_cube_pose_msg = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=10)
                rospy.loginfo("Detected cube pose on table 2 %s!", detected_cube_pose_msg)
                self.detected_cube = True
                if self.mission_done is not None:
                    self.mission_done.task_finished = True
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
    mode = 0 (default); resets the robot and the cube to initial state (Task C)
    mode = 1          ; resets the cube only to initial state (Task A)
    """

    def __init__(self, mode = 0):
        
        # mode 
        self.mode = mode
        rospy.loginfo("Initialising the resetting of the pose of the robot and cube.")

        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  # Create a service proxy

        self.robot_state = self.get_robot_state()
        self.cube_state = self.get_cube_state()

        # execution checker
        self.tried = False
        self.done = False

        self.success_robot = True 

        # become a behaviour
        super(resetpose, self).__init__("Reset cube and/or robot!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            self.reset_cube_req = self.reset_srv(self.cube_state)
            self.tried = True

            # command
            if self.mode == 0:
                self.reset_robot_req = self.reset_srv(self.robot_state)
                self.success_robot = self.reset_robot_req.success

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.success_robot and self.reset_cube_req.success:
            rospy.loginfo("Resetting of the pose of the robot and cube succeeded!.")
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.success_robot or not self.reset_cube_req.success:   
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

class amcl_convergence_checker(pt.behaviour.Behaviour):

    """
    Checks if AMCL has converged by thresholding the standard 
    deviation in both x and y direction.
    """

    def __init__(self):

        rospy.loginfo("Initialising AMCL convergence checker.")

        self.particlecloud_top = '/particlecloud'

        self.std_position_threshold = 0.05

        self.init_localization = False

        # servers to the clear costmaps
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=30)

        # become a behaviour
        super(amcl_convergence_checker, self).__init__("Localization converged?")

    def update(self):

        # Has the robot succeeded in localizing itself initially? 
        if self.init_localization:
            return pt.common.Status.SUCCESS

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        #rospy.loginfo("Cloud particles received, checking convergence!")

        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        if len(rec_particles) > 0:
            self.check_convergence(rec_particles)

        # Localization succeeded ?
        if self.init_localization:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


    def check_convergence(self, particles):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        #rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")

        # Check if both x and y standard deviations are below the threshold
        if (x_stddev < self.std_position_threshold and y_stddev < self.std_position_threshold):
            rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
            rospy.loginfo("Initial localization is successful. PF has converged!")
            self.init_localization = True
            # Clear costmaps
            clear_costmaps_req = self.clear_costmaps_srv()
            rospy.loginfo("Costmaps cleared successfully!")
        #else:
            #rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
            #rospy.loginfo("Initial localization is not successful yet.")

class update_localization(pt.behaviour.Behaviour):

    """
    Completes an update step for the amcl
    """

    def __init__(self, mode = 0):

        rospy.loginfo("Initialising global and update localization.")

        self.mode = mode

        # servers to initiate global localization and update localization
        self.global_localization_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)
        
        if self.mode == 0:
            global_localization_req = self.global_localization_srv() # initialize global localization
            rospy.loginfo("Global localization initialized successfully!")
        elif self.mode == 1:
            self.global_localization_done = False

        self.update_localization_srv = rospy.ServiceProxy('/request_nomotion_update', Empty)
        rospy.wait_for_service('/request_nomotion_update', timeout=30)

        # become a behaviour
        super(update_localization, self).__init__("Update Robot Localization!")

    def update(self):
        
        if self.mode == 1 and (not self.global_localization_done):
            global_localization_req = self.global_localization_srv() # initialize global localization
            rospy.loginfo("Global localization initialized successfully!")
            self.global_localization_done = True

        self.update_localization_srv_req = self.update_localization_srv()
        #rospy.loginfo("Localization succeeded in updating!")
        return pt.common.Status.RUNNING

class cancel_goal(pt.behaviour.Behaviour):
    """
    Cancels goal
    """
    def __init__(self, ac_client):

        rospy.loginfo("Initialising cancel goal.")

        self.ac_client = ac_client

        self.goal_cancelled = False
        # become a behaviour
        super(cancel_goal, self).__init__("Cancel Goal!")

    def update(self):
        
        if not self.goal_cancelled:
            self.ac_client.cancel_goal()
            rospy.loginfo("Goal cancelled successfully")
            self.goal_cancelled = True

        return pt.common.Status.SUCCESS

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
        #self.det_cov = 0.0

        self.kidnapped = False

        self.sendgoal = sendgoal

        # become a behaviour
        super(kidnap_checker, self).__init__("Kidnap checker ?")

    def update(self):

        # Is the robot kidnapped? 
        if self.kidnapped:
            return pt.common.Status.SUCCESS
        elif self.sendgoal.finished:
            return pt.common.Status.FAILURE

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        #rospy.loginfo("Cloud particles received, checking convergence!")
        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        # Obtain pose estimate with covariance matrix
        pose_estimate = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, timeout=5)

        Cov_mat = pose_estimate.pose.covariance
        x_cov = Cov_mat[0]    
        y_cov = Cov_mat[7]    
        yaw_cov = Cov_mat[35]
        new_position_and_yaw = x_cov + y_cov + yaw_cov

        if self.last_position_and_yaw is None:
            self.last_position_and_yaw = new_position_and_yaw
            rospy.loginfo(f'Initial covariance {new_position_and_yaw}')
            return pt.common.Status.RUNNING
        
        if len(rec_particles) > 0:
            self.check_convergence(rec_particles, new_position_and_yaw)
            self.last_position_and_yaw = new_position_and_yaw


        if self.kidnapped:
            return pt.common.Status.SUCCESS 

        # tell the tree you've failed, you sucker!!
        return pt.common.Status.FAILURE
            
    def check_convergence(self, particles, new_position_and_yaw):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")

        change_in_position_and_yaw = new_position_and_yaw - self.last_position_and_yaw

        if abs(change_in_position_and_yaw) > 1e-2:
            rospy.loginfo(f"Change in P_xx*P_yy*P_yawyaw: {change_in_position_and_yaw}")
            rospy.loginfo('Robot kidnapped!!!!')
            self.kidnapped = True
            self.sendgoal.finished = True

        elif (x_stddev + y_stddev > self.std_position_threshold):
            #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")
            rospy.loginfo("Robot is kidnapped, should cancel goal now!")
            self.kidnapped = True
            self.sendgoal.finished = True

    
    def get_kidnapped_state(self):
        return self.kidnapped

class sendgoal(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    
    @param operation: The operation to perform (e.g., pick, place, cancel).
    """

    def __init__(self, operation, client=None):
        rospy.loginfo("Initializing send goal behaviour for " + operation + " operation!")
        self.operation = operation

        # Set up action client
        if client:
            self.move_base_ac = client
        else:
            self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Connect to the action server
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()
        rospy.loginfo("%s: Connected to move_base action server")

        if operation =="cancel":
            self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
            self.cancel_msg = GoalID()
        else:
            # Get the pose from the pose topic
            operation_pose_top = rospy.get_param(rospy.get_name() + '/' + operation + '_pose_topic')
            self.operation_pose = rospy.wait_for_message(operation_pose_top, PoseStamped, timeout=5)

        # Execution checker, Boolean to check the task status
        self.operation = operation
        self.sent_goal = False
        self.finished = False

        # Become a behaviour
        super(sendgoal, self).__init__("Send goal:" + operation)

    def get_action_client(self):
        return self.move_base_ac
    
    def get_new_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose = self.operation_pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Received goal pose for " + self.operation + " operation!")
        return goal

    def update(self):
        # Already done the task
        if self.finished:
            return pt.common.Status.SUCCESS
        
        # Not sent the goal yet
        elif not self.sent_goal and self.operation == "cancel":
            self.cancel_pub.publish(self.cancel_msg)
            self.sent_goal = True
            self.finished = True
            return pt.common.Status.SUCCESS

        # Not sent the goal yet
        elif not self.sent_goal and self.operation != "cancel":
            goal = self.get_new_goal()
            self.move_base_ac.send_goal(goal)
            self.sent_goal = True
            return pt.common.Status.RUNNING

        elif self.move_base_ac.get_state() == 0 or self.move_base_ac.get_state() == 1:
            return pt.common.Status.RUNNING 

        # Task is successful
        elif self.move_base_ac.get_result():
            self.finished = True
            return pt.common.Status.SUCCESS

        # Failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # Already sent the goal and not yet received the result
        else:
            return pt.common.Status.RUNNING

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

        self.sent_message = False

        # become a behaviour
        super(gather_cues, self).__init__("Gather cues")

    def update(self):
        
        if self.counter == 0:
            rospy.loginfo("Rotating to gather cues!")
        if self.counter >= self.count_max:
            if not self.sent_message:
                rospy.loginfo('Cues Gathered')
                self.sent_message = True
            return pt.common.Status.SUCCESS
        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()
        self.counter += 1

        # tell the tree that you're running
        return pt.common.Status.RUNNING 

class mission_done(pt.behaviour.Behaviour):

    """
    Checks if mission is done.
    """
    def __init__(self):
        rospy.loginfo("Initialising mission done behaviour.")

        self.task_finished = False

        self.message_sent = False

        # become a behaviour
        super(mission_done, self).__init__("Mission done?")

    def update(self):

        if self.task_finished:
            if not self.message_sent:
                rospy.loginfo('Mission DONE, I will do nothing now!')
                self.message_sent = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class reset_tree_states(pt.behaviour.Behaviour):

    """
    Resets the tree states such that another iteration can follow.
    """
    def __init__(self, kidnap_checker, cancel_goal, update_localization, convergence_checker, send_goal_first, send_goal_kid,
                            detect_cube, pick_cube, moveto_place, place_cube, cube_detected_ont2, move_head, tuck_arm, resetpose):
        rospy.loginfo("Initialising reset tree states behaviour.")

        self.kidnap_checker = kidnap_checker  
        self.cancel_goal = cancel_goal
        self.update_localization = update_localization
        self.convergence_checker = convergence_checker
        self.send_goal_first = send_goal_first
        self.send_goal_kid = send_goal_kid
        self.detect_cube = detect_cube
        self.pick_cube = pick_cube
        self.moveto_place = moveto_place
        self.place_cube = place_cube
        self.cube_detected_ont2 = cube_detected_ont2
        self.move_head = move_head
        self.tuck_arm = tuck_arm
        self.resetpose = resetpose

        # become a behaviour
        super(reset_tree_states, self).__init__("Reset tree states!")

    def update(self):
        
        # Reset kidnap checker
        self.kidnap_checker.kidnapped = False

        # Reset cancel goal
        self.cancel_goal.goal_cancelled = False

        # Reset update localization
        self.convergence_checker.init_localization = False

        # Reset global localization
        self.update_localization.global_localization_done = False

        # Reset send goal first
        self.send_goal_first.sent_goal = False
        self.send_goal_first.finished = False

        # Reset send goal kidnap
        self.send_goal_kid.sent_goal = False
        self.send_goal_kid.finished = False

        # reset detect cube
        self.detect_cube.sent_goal = False
        self.detect_cube.finished = False

        # Reset pick cube
        self.pick_cube.tried = False
        self.pick_cube.done = False

        # Reset navigate to place
        self.moveto_place.sent_goal = False
        self.moveto_place.finished = False

        # Reset place cube
        self.place_cube.tried = False
        self.place_cube.done = False

        # Reset cube detected on table 2
        self.cube_detected_ont2.listened = False

        # Reset move head up
        self.move_head.tried = False
        self.move_head.done = False

        # Reset tuck arm
        self.tuck_arm.sent_goal = False
        self.tuck_arm.finished = False

        # Reset move head up
        self.resetpose.tried = False
        self.resetpose.done = False

        return pt.common.Status.FAILURE
