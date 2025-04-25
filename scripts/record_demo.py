
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


if __name__ == "__main__":
	rospy.init_node('dorfl_arm_commander', anonymous=True)
	rospy.Subscriber("/dorfl/iiwa_left/PositionTrajectoryController/state", JointTrajectoryControllerState, joint_traj_state_callback)
	rospy.spin()