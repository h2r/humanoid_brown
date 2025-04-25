#Author is Skye Thompson, rory_thompson@brown.edu if you have questions
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf2_geometry_msgs
import tf2_ros as tf
from tf2_msgs.msg import TFMessage
from os import environ, makedirs
from camera import TakePhoto
import pickle
from GripperCommander import GripperCommander
from barrett_gripper_commander import BarrettCommander

from skill import RaiseLeftHand, RaiseRightHand, LowerBothHands

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# ROS Setup
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
environ["ROS_NAMESPACE"]="dorfl"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("dorfl_commander", anonymous=True)
robot = moveit_commander.RobotCommander(robot_description="dorfl/robot_description")

right_move_group = moveit_commander.MoveGroupCommander("right_arm", robot_description="dorfl/robot_description",)
left_move_group = moveit_commander.MoveGroupCommander("left_arm", robot_description="dorfl/robot_description",)
move_groups = {'right': right_move_group, 'left': left_move_group}

gripper_commanders = {'right': GripperCommander(), 'left': BarrettCommander()}

# For transforms
tfBuffer = tf.Buffer()
tf_listener = tf.TransformListener(tfBuffer)
# This stalls to make sure the transform tree is available
while not rospy.is_shutdown():
    try:
        ee_tf = tfBuffer.lookup_transform("world", 'iiwa_left_link_ee', rospy.Time(0))
        break  
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

#Cameras
color_camera = TakePhoto('/multisense/left/image_rect_color')

def take_color_photo(save_name):
    success = color_camera.take_picture(save_name)
    return success

# For skill execution
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skill_sequence = [
    # Add your skill sequence here
    RaiseRightHand(),
    RaiseLeftHand(),
    LowerBothHands(),
]


sequence_id = "test_sequence"
save_file_root = "test_images"
# Take starting image
#makedirs(save_file_root + "/" + sequence_id)
take_color_photo(save_file_root + "/"+ sequence_id + "/0_True.jpg")

for step, skill in enumerate(skill_sequence):
    image_root = "/"+ sequence_id + "/" + str(step+1) + "_"

    print("Executing skill: ", skill.name)
    actions = skill.get_actions(tfBuffer) #Pass the tfBuffer for the current tf tree
    
    skill_executability = raw_input("Press Enter to execute skill. Press anything else to skip.")
    if skill_executability == '':
        for hand_poses in actions:
            for hand in 'left', 'right': #TODO: Someday it would be nice to have simultaneous execution
                if hand_poses[hand] is None:
                    continue
                gripper = hand_poses[hand]["gripper"]
                if gripper is not None:
                    gripper_commanders[hand].move_to_pose(gripper)
                    rospy.sleep(1.0)
                    
                hand_move_group = move_groups[hand]
                pose = hand_poses[hand]["pose"]
                rospy.sleep(1.0)
                pose_goal =  tf2_geometry_msgs.PoseStamped()
                pose_goal.header.stamp = rospy.Time(0)
                pose_goal.header.frame_id = "world"
                pose_goal.pose = pose.pose
                plan = hand_move_group.plan(pose_goal)

                raw_input("Press enter to execute the next action")
                hand_move_group.execute(plan)
    else:
        pass

    success = raw_input("Did the skill succeed? (y/n)")
    if success == 'y':
        take_color_photo(save_file_root + "/"+ (sequence_id) + "/" + str(step+1) + "_True.jpg")
    else:
        take_color_photo(save_file_root + "/"+ (sequence_id) + "/" + str(step+1) + "_False.jpg")
           
    #Save
    print("Skill execution complete")