import rospy 
import time 

from control_msgs.msg import FollowJointTrajectoryActionGoal 
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint",
			   "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint",
			   "sdh_finger_23_joint"]

rospy.init_node('node_name')
pub = rospy.Publisher('/sdh_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

# construct message 
cmd = FollowJointTrajectoryActionGoal()
cmd.header.stamp = rospy.Time.now()
cmd.goal.trajectory.header.stamp = rospy.Time.now()
cmd.goal.trajectory.joint_names = JOINT_NAMES

# construct goal point 
point = JointTrajectoryPoint()
point.positions = [0, -0.3, -0.3, -0.3, -0.3, -0.3, -0.3]
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cmd.goal.trajectory.points = [point]
print(cmd)

# wait for publisher/subscriber connection 
time.sleep(2.0)

# command! 
pub.publish(cmd)

