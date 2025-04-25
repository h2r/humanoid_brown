import rospy 
import time 
from sensor_msgs.msg import JointState

from bhand_controller.srv import ActionsRequest, ActionsResponse, Actions


JOINT_NAMES = ['j11_joint', 'j32_joint', 'j12_joint', 'j22_joint']
OPEN = 3#[0.5 , 0.5, 1.0, 1.5]
CLOSED = 2#[0.0, 0.0, 0.0, 0.0]


class BarrettCommander(object):
	"""docstring for GripperCommander"""
	def __init__(self):

		#self.pub = rospy.Publisher('/dorfl/bhand_left/bhand_node/command', JointState, queue_size=10)
		self.gripper_srv = rospy.ServiceProxy('/dorfl/bhand_left/bhand_node/actions', Actions)
		self.gripper_srv(1)
		# wait for publisher/subscriber connection 
		time.sleep(2.0)

	def move_to_pose(self, pose):
		""" pose is a list length 7 corresponding to JOINT_NAMES"""
		self.gripper_srv(pose)
		# # construct message 
		# cmd = JointState()
		# cmd.header.stamp = rospy.Time.now()
		# cmd.header.stamp = rospy.Time.now()
		# cmd.name = JOINT_NAMES
		# cmd.position = pose
		# cmd.velocity = [0]
		# cmd.effort = [0]

		# # command! 
		# self.pub.publish(cmd)
		time.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('barrett_gripper_commander', anonymous=True)
    barrett = BarrettCommander()
    barrett.move_to_pose(OPEN)