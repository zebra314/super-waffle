#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# detect the cube
		b0 = detectcube()

		# pick the cube
		b1 = movecube("pick")

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(14, "At table?"), go("Go to table!", 0, -3)]
		)

		b3 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(18, "At table?"), go("Go to table!", 0.5, 0)]
		)

		# place the cube
		b4 = movecube("place")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
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
