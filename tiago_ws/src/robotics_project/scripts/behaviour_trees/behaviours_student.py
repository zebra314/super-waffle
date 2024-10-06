# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

import numpy as np
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
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

class navigate_to_pick_pose(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising navigating to pick pose.")

        # Set up action client  
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()
        rospy.loginfo("%s: Connected to move_base action server")

        # personal goal setting
        self.goal = MoveBaseGoal()

        # Set the position
        self.goal.target_pose.pose.position = Point(-1.1480, -6.1, -0.001)
        # Set the orientation (quaternion)
        self.goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, -0.709307863674, 0.70489882574)
        # Send the goal
        rospy.loginfo("Sending navigation goal!")
 

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(navigate_to_pick_pose, self).__init__("Navigate to pick pose!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            # Set the goal frame ID (e.g., map frame)
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.move_base_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class amcl_convergence_checker(pt.behaviour.Behaviour):

    """
    Checks if AMCL has converged by thresholding the standard 
    deviation in both x and y directin.
    """

    def __init__(self):

        rospy.loginfo("Initialising checker of convergence of amcl.")

        self.particlecloud_top = '/particlecloud'
        self.converged = False
        self.std_position_threshold = 0.2

        # become a behaviour
        super(amcl_convergence_checker, self).__init__("Paricles Converged ?")

    def update(self):

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        rospy.loginfo("Cloud particles received, checking convergence!")

        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        if len(rec_particles) > 0:
            self.check_convergence(rec_particles)

        if self.converged:
            return pt.common.Status.SUCCESS

        # tell the tree you've failed, you sucker!!
        return pt.common.Status.FAILURE
            
    def check_convergence(self, particles):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")

        # Check if both x and y standard deviations are below the threshold
        if x_stddev < self.std_position_threshold and y_stddev < self.std_position_threshold:
            #rospy.loginfo("AMCL has converged!")
            self.converged = True
        else:
            self.converged = False
            rospy.loginfo("AMCL has not converged yet.")


class update_localization(pt.behaviour.Behaviour):

    """
    Pick or place the cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising global and update localization.")

        # servers to initiate global localization and update localization
        self.global_localization_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)
        global_localization_req = self.global_localization_srv() # initialize global localization
        rospy.loginfo("Global localization initialized successfully!")

        self.update_localization_srv = rospy.ServiceProxy('/request_nomotion_update', Empty)
        rospy.wait_for_service('/request_nomotion_update', timeout=30)

        # become a behaviour
        super(update_localization, self).__init__("Localize robot!")

    def update(self):

        self.update_localization_srv_req = self.update_localization_srv()
        rospy.loginfo("Localization succeeded in updating!")
        return pt.common.Status.RUNNING

class navigate_to_pose(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, operation):

        rospy.loginfo("Initialising navigating to " + operation + " pose.")

        # Set up action client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()
        rospy.loginfo("%s: Connected to move_base action server")

        # personal goal setting
        self.goal = MoveBaseGoal()

        if operation == "pick":
            self.goal.target_pose.pose.position = Point(-1.1480, -6.1, -0.001)
            self.goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, -0.709307863674, 0.70489882574)
        elif operation == "place":
            self.goal.target_pose.pose.position = Point(2.6009, -1.7615, 0.0)
            self.goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1)        
        else:
            rospy.loginfo("Invalid operation!")
            exit()
        rospy.loginfo("Sending navigation goal!")
 
        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(navigate_to_pose, self).__init__(operation)

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            # Set the goal frame ID (e.g., map frame)
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.move_base_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING