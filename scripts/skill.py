#Author is Skye Thompson, rory_thompson@brown.edu if you have questions
import rospy
import tf2_geometry_msgs
from GripperCommander import PINCH_OPEN, GRASP
from barrett_gripper_commander import OPEN, CLOSED

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def make_pose_stamped(pose, frame_id):
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.header.stamp = rospy.Time(0)
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x = pose[0]
    pose_stamped.pose.position.y = pose[1]
    pose_stamped.pose.position.z = pose[2]
    pose_stamped.pose.orientation.x = pose[3]
    pose_stamped.pose.orientation.y = pose[4]
    pose_stamped.pose.orientation.z = pose[5]
    pose_stamped.pose.orientation.w = pose[6]
    return pose_stamped

def list_to_action(pose, frame, tf_buffer):
    if len(pose) == 7:
        pose_stamped = tf_buffer.transform(make_pose_stamped(pose, frame), 'world')
        return {
            "pose": pose_stamped,
            "gripper": None
        }
    elif len(pose) == 8:
        pose_stamped = tf_buffer.transform(make_pose_stamped(pose[:7], frame), 'world')
        return {
            "pose": pose_stamped,
            "gripper": pose[7]
        }

class Skill(object):
    def __init__(self, name):
        self.name = name
        self.skill_frame_right = None
        self.skill_frame_left = None

    def get_actions(self, tfBuffer): #TODO: make it a 
        actions = []
        for left_action, right_action in zip(self.left_hand_actions, self.right_hand_actions):
            if left_action is None:
                left_hand_action = None
            else:
                left_hand_action = list_to_action(left_action, self.skill_frame_left, tfBuffer)
            
            if right_action is None:
                right_hand_action = None
            else:
                right_hand_action = list_to_action(right_action, self.skill_frame_right, tfBuffer)

            actions.append({
                "left": left_hand_action,
                "right": right_hand_action
            })
        return actions


class RaiseRightHand(Skill):
    def __init__(self):
        super(RaiseRightHand, self).__init__("raise_right_hand")
        self.skill_frame_left = 'iiwa_left_link_ee'
        self.skill_frame_right = 'iiwa_right_link_ee'
        self.right_hand_actions = [
            [0.0, 0.0, .10, 0.0, 0.0, 0.0, 1.0, PINCH_OPEN],
        ]
        self.left_hand_actions = [
            None for _ in self.right_hand_actions
        ]
    
class RaiseLeftHand(Skill):
    def __init__(self):
        super(RaiseLeftHand, self).__init__("raise_left_hand")
        self.skill_frame_left = 'iiwa_left_link_ee'
        self.skill_frame_right = 'iiwa_right_link_ee'
        self.left_hand_actions = [
            [0.0, 0.0, .10, 0.0, 0.0, 0.0, 1.0, CLOSED],
        ]
        self.right_hand_actions = [
            None for _ in self.left_hand_actions
        ]
    
class LowerBothHands(Skill):
    def __init__(self):
        super(LowerBothHands, self).__init__("lower_both_hands")
        self.skill_frame_left = 'iiwa_left_link_ee'
        self.skill_frame_right = 'iiwa_right_link_ee'
        self.left_hand_actions = [      
            [0.0, 0.0, -.10, 0.0, 0.0, 0.0, 1.0, OPEN],
        ]
        self.right_hand_actions = [
            [0.0, 0.0, -.10, 0.0, 0.0, 0.0, 1.0, GRASP],
        ]