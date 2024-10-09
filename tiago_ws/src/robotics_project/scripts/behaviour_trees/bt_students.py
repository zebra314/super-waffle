#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		# Task C:
		# tuck the arm
		# b0 = tuckarm()

		# # detect the cube
		# b1 = detectcube()

		# # pick the cube
		# b2 = movecube("pick")

		# # go to table
		# b3 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(15, "At table?"), go("Go to table!", 0, -3)]
		# )

		# b4 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(16, "At table?"), go("Go to table!", 0.5, 0)]
		# )

		# # place the cube on second table
		# b5 = movecube("place")

		# # Create sequence to execute if job is not done!
		# sequence = RSequence(name="Job not done and Reset", children=[tuckarm(), wait(10, "Resetting!"), resetpose()])
		
		# # checks if cube is placed on table 2 and job is done
		# b6 = pt.composites.Selector(
		# 	name="Check if detected",
		# 	children=[cube_detected_on_table2(), sequence]
		# )

		# # become the tree
		# tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6])
		# super(BehaviourTree, self).__init__(tree)

		# Part A
		# b0 = tuckarm()

		# b1 = pt.composites.Selector(
		# 	name="Perform initial localization",
		# 	children=[amcl_convergence_checker(), update_localization(mode = 0)]
		# ) 

		# b212 =  pt.composites.Selector(
		# 	name="Perform re-localization",
		# 	children=[amcl_convergence_checker(), update_localization(mode = 1)]
		# ) 

		# send_goal_pick = sendgoal("pick")
		# b21 = RSequence(name="Kidnap checking sequence", children=[kidnap_checker(send_goal_pick), cancel_goal(send_goal_pick.get_action_client()), b212, gather_cues(1, 100), sendgoal("pick")]) # send goal still missing
		# b22 = send_goal_pick 

		# b2 = pt.composites.Selector(
		# 	name="Nanvigation and kidnap checking",
		# 	children=[b21, b22]
		# )

		# b3 = detectcube()
		
		# b4 = movecube('pick')	

		# b5 = sendgoal('place')

		# b6 = movecube('place')

		# b7 = cube_detected_on_table2()

		blackboard = Blackboard()

		b1 = check_convergence(blackboard)
		b2 = update_localization(blackboard)
		b3 = move_pose("pick_pose", blackboard)
		b4 = detect_cube(blackboard)
		b5 = move_cube("pick", blackboard)
		b6 = move_pose("place_pose", blackboard)
		b7 = move_cube("place", blackboard)
		b8 = check_cube(blackboard)
		b9 = check_end(blackboard)
		
		tree = RSequence(name="Main sequence", children=[b1, b2])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
