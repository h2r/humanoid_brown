import rospy 
import time 

from control_msgs.msg import FollowJointTrajectoryActionGoal 
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np 

JOINT_NAMES = ["sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint",
			   "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint",
			   "sdh_finger_23_joint"]

EPS = 1e-3
STRONG_EPS = 5e-1
NEUTRAL = [0.0]*7
SLIGHT_OPEN = [0.5] + [-0.3]*6
JAR_CLOSED = [0.5] + [0.0, 0.2]*3
THREE_FINGER_PINCH_OPEN =  [0, -.7, 0.0, -.7, 0.0, -.7, 0.0]
THREE_FINGER_PINCH_CLOSED = [0.0, .18, 0.0, .18, 0.0, .18, 0.0]
THREE_FINGER_POWER_CLOSED = [0.0, -0.15, 0.9, -0.15, 0.9, -0.15, 0.9]
THREE_FINGER_POWER_CLOSED = [0.0, -0.15, 0.8, -0.15, 0.8, -0.15, 0.8]
THREE_FINGER_PINCH_RETRACT = [0, -.7, 0.0, -.7, 0.9, -.7, 0.9]

PREGRASP = [np.pi / 2 - EPS , -np.pi / 2 + EPS, -0.3, -0.3, -0.3, -0.3, -0.3 ]
STRONG_GRASP = [np.pi / 2 - EPS , -np.pi / 2 + EPS, 0+STRONG_EPS, 0.18+STRONG_EPS, 0+STRONG_EPS, 0.18+STRONG_EPS, 0+STRONG_EPS]
GRASP = [np.pi / 2 - EPS , -np.pi / 2 + EPS, 0, 0.18, 0, 0.18, 0 ]
# GRASP = [np.pi / 2 - EPS , -np.pi / 2 + EPS, 0, -0.085, 0, -0.085, 0 ]
PINCH_OPEN = [np.pi / 2 - EPS, -np.pi / 2 + EPS, 0.0, -np.pi / 2 + EPS, 0.0, -np.pi / 2 + EPS, 0.0]

RELEASE = [np.pi / 2 - EPS , -np.pi / 2 + EPS, 0, 0.0, 0, 0.0, 0 ]

# CAGE = [0, 0, 1.5, 0.00, 0, 0.00, 0 ]
# CAGE = [0, -np.pi / 10, np.pi / 4, 0.00, 0.75, 0.00, 0.75 ]
# CAGE = [0, 0, np.pi / 2.1, 0.00, np.pi / 2.1, 0.00, np.pi/2.1 ]



class GripperCommander(object):
	"""docstring for GripperCommander"""
	def __init__(self):

		self.pub = rospy.Publisher('/dorfl/iiwa_right/sdh_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		
		# wait for publisher/subscriber connection 
		time.sleep(2.0)

	def move_to_pose(self, pose):
		""" pose is a list length 7 corresponding to JOINT_NAMES"""

		# construct message 
		cmd = FollowJointTrajectoryActionGoal()
		cmd.header.stamp = rospy.Time.now()
		cmd.goal.trajectory.header.stamp = rospy.Time.now()
		cmd.goal.trajectory.joint_names = JOINT_NAMES

		# construct goal point 
		point = JointTrajectoryPoint()
		point.positions = pose
		point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		cmd.goal.trajectory.points = [point]
		print(cmd)

		# command! 
		self.pub.publish(cmd)
		time.sleep(2.0)

ACTION_MAPS = {
	"default": [0] * 7,
	"slight_open": SLIGHT_OPEN,
	"three_finger_pinch_open": THREE_FINGER_PINCH_OPEN,
	"three_finger_pinch_closed": THREE_FINGER_PINCH_CLOSED,
	"three_finger_power_closed": THREE_FINGER_POWER_CLOSED,
	"three_finger_pinch_retract": THREE_FINGER_PINCH_RETRACT,
	"pregrasp": PREGRASP,
	"strong_grasp": STRONG_GRASP,
	"grasp": GRASP,
	"pinch_open": PINCH_OPEN,
	"release": RELEASE,
	"jar_closed": JAR_CLOSED,
	# "cage": CAGE,
}


if __name__ == '__main__':
	import sys
	print(sys.argv)
	if len(sys.argv) > 1:
		if sys.argv[1] in ACTION_MAPS.keys():
			pose = ACTION_MAPS[sys.argv[1]]
		else:
			print("Action not found.")
			exit(1)
	else:
		print("No action specified. Available actions are:")
		for key in ACTION_MAPS.keys():
			print("   - " + key)
		exit(0)

	
	rospy.init_node('node_name')
	gc = GripperCommander()

	gc.move_to_pose(pose)
	
	# gc.move_to_pose([0]*7)
	# gc.move_to_pose(SLIGHT_OPEN)
	# gc.move_to_pose(THREE_FINGER_PINCH_OPEN)
	#gc.move_to_pose(THREE_FINGER_PINCH_RETRACT)
	# gc.move_to_pose(THREE_FINGER_PINCH_CLOSED)


	# gc.move_to_pose(NEUTRAL)
	#gc.move_to_pose(PREGRASP)
	# gc.move_to_pose(PINCH_OPEN)
	# gc.move_to_pose(GRASP)
	# gc.move_to_pose(STRONG_GRASP)
	# gc.move_to_pose(RELEASE)

	# gc.move_to_pose(CAGE)