import rospy
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from actionlib_msgs.msg import GoalID

import moveit_commander
import moveit_msgs.msg
import copy as cp

def joint_traj_state_callback(data):
	print(data.joint_names)
	print()
	print(data.actual)
	print()
	print(data.desired)
	print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

	#adjust joint positions as actual + .1
	#publish that as new joint position command


if __name__ == "__main__":
# set_point = JointTrajectoryPoint()
# #set_point.joint_names  = joint_names
# set_point.positions = desired_position
# set_point.velocities = desired_velocities
# set_point.accelerations = desired_accelerations
# set_point.time_from_start = d

# joint_goal = JointTrajectory()
# joint_goal.joint_names = joint_names
# joint_goal.points = [set_point]

# joint_action = FollowJointTrajectoryGoal()
# joint_action.trajectory = joint_goal

# joint_tolerances = []
# goal_tolerances = []

# for name in joint_names:
# 	temp_joint_tolerance = JointTolerance()
# 	temp_joint_tolerance.name = cp.deepcopy(name)
# 	temp_joint_tolerance.position = 0
# 	temp_joint_tolerance.velocity = 0
# 	temp_joint_tolerance.acceleration = 0
# 	joint_tolerances.append(cp.deepcopy(temp_joint_tolerance))

# 	temp_goal_tolerance = JointTolerance()
# 	temp_goal_tolerance.name = cp.deepcopy(name)
# 	temp_goal_tolerance.position = 0.05
# 	temp_goal_tolerance.velocity = 0.001
# 	temp_goal_tolerance.acceleration = 0.001
# 	goal_tolerances.append(cp.deepcopy(temp_goal_tolerance))

# joint_action.path_tolerance = joint_tolerances
# joint_action.goal_tolerance = goal_tolerances
# joint_action.goal_time_tolerance = d

# action_msg = FollowJointTrajectoryActionGoal()
# goal_name = GoalID()
# goal_name.id = "pos4_goal"
# goal_name.stamp = rospy.get_rostime()
# action_msg.goal_id = goal_name
# action_msg.goal = joint_action

	
	rospy.init_node('left_arm_commander', anonymous=True)
	pub = rospy.Publisher('/dorfl/iiwa_left/PositionTrajectoryController/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
	rate = rospy.Rate(10)
	

	#get the end effector pose in cartesian space
	#select a new desired position
	#call moveit to get a plan to the new position
	#execute that plan




	# joint_names = ['iiwa_left_joint_1', 'iiwa_left_joint_2', 'iiwa_left_joint_3', 'iiwa_left_joint_4', 'iiwa_left_joint_5', 'iiwa_left_joint_6', 'iiwa_left_joint_7']

	# #desired_position = [0.13672562508659666, -1.3587597351035998, 0.0027270104155634077, -0.8477916933900675, 0.013318883982122245, -0.5638574821662639, 0.8902353838663396+.1]
	
	# desired_position = [0.1367193333928094, -1.3587673451754707, 0.0027292274200457533, -0.8477884576587174, 0.013312268835191335, -0.5638593397420033,  0]
	# desired_velocities = [0,0,0,0,0,0,0]
	# desired_accelerations = [0,0,0,0,0,0,0]
	# d = rospy.Duration.from_sec(1)

	# set_point = JointTrajectoryPoint()
	# #set_point.joint_names  = joint_names
	# set_point.positions = desired_position
	# set_point.velocities = desired_velocities
	# set_point.accelerations = desired_accelerations
	# set_point.time_from_start = d

	# joint_goal = JointTrajectory()
	# joint_goal.joint_names = joint_names
	# joint_goal.points = [set_point]

	# joint_action = FollowJointTrajectoryGoal()
	# joint_action.trajectory = joint_goal

	# joint_tolerances = []
	# goal_tolerances = []

	# for name in joint_names:
	# 	temp_joint_tolerance = JointTolerance()
	# 	temp_joint_tolerance.name = cp.deepcopy(name)
	# 	temp_joint_tolerance.position = 0
	# 	temp_joint_tolerance.velocity = 0
	# 	temp_joint_tolerance.acceleration = 0
	# 	joint_tolerances.append(cp.deepcopy(temp_joint_tolerance))

	# 	temp_goal_tolerance = JointTolerance()
	# 	temp_goal_tolerance.name = cp.deepcopy(name)
	# 	temp_goal_tolerance.position = 0.05
	# 	temp_goal_tolerance.velocity = 0.001
	# 	temp_goal_tolerance.acceleration = 0.001
	# 	goal_tolerances.append(cp.deepcopy(temp_goal_tolerance))

	# joint_action.path_tolerance = joint_tolerances
	# joint_action.goal_tolerance = goal_tolerances
	# joint_action.goal_time_tolerance = d

	# action_msg = FollowJointTrajectoryActionGoal()
	# goal_name = GoalID()
	# goal_name.id = "pos4_goal"
	# goal_name.stamp = rospy.get_rostime()
	# action_msg.goal_id = goal_name
	# action_msg.goal = joint_action

	# #rospy.Subscriber("/dorfl/iiwa_left/PositionTrajectoryController/state", JointTrajectoryControllerState, joint_traj_state_callback)
	# while not rospy.is_shutdown():
	# 	pub.publish(action_msg)
	# 	# right_arm.set_target(right_arm.cartesian_state[:3], right_arm.cartesian_state[3:])
	# 	# right_arm.publish_target_command()
	# 	rate.sleep()
	# # rospy.Subscriber("dorfl/iiwa_left/joint_states", JointState, right_arm.joint_callback)
	# # rospy.Subscriber("dorfl/iiwa_left/state/JointVelocity", JointVelocity, right_arm.joint_vel_callback)