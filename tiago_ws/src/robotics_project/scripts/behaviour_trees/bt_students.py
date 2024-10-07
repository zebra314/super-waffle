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

		b0 = tuckarm()

		b1 = pt.composites.Selector(
			name="Perform localization",
			children=[amcl_convergence_checker(), update_localization()]
		)

		b2 = sendgoal("pick")
		b3 = detectcube()
		b4 = movecube("pick")

		b5 = sendgoal("place")
		b8 = sendgoal("cancel", b5.get_client())
		b6 = movecube("place")

		tree = RSequence(name="Main sequence", children=[b5, b8])
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
