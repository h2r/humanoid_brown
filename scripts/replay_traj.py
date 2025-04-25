import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf2_geometry_msgs
import tf2_ros as tf
from tf2_msgs.msg import TFMessage
from os import environ
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# takes a list [x,y,z, qx,qy,qz,qw] and a frame id
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

pose_marker_pub = rospy.Publisher("right_hand_marker", tf2_geometry_msgs.PoseStamped, None)
environ["ROS_NAMESPACE"]="dorfl"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("dorfl_commander", anonymous=True)
robot = moveit_commander.RobotCommander(robot_description="dorfl/robot_description")

right_move_group = moveit_commander.MoveGroupCommander("right_arm", robot_description="dorfl/robot_description",)
left_move_group = moveit_commander.MoveGroupCommander("left_arm", robot_description="dorfl/robot_description",)


display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

tfBuffer = tf.Buffer()
tf_listener = tf.TransformListener(tfBuffer)

# This stalls to make sure tpose_marker_pub.publish(pose)he transform is available
while not rospy.is_shutdown():
    try:
        ee_tf = tfBuffer.lookup_transform("world", 'iiwa_left_link_ee', rospy.Time(0))
        break  
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

# left_arm_with_jar_transform = make_pose_stamped([0.952415898684, 0.24956536805, 0.985405932473, 
#                                                 0., 0., 1., 0.], "world")

# pose_goal =  tf2_geometry_msgs.PoseStamped()
# pose_goal.header.stamp = rospy.Time(0)
# pose_goal.header.frame_id = "world"
# pose_goal.pose = left_arm_with_jar_transform.pose

# plan = left_move_group.plan(pose_goal)
# raw_input("Press Enter to execute the next step of the plan...")
# left_move_group.execute(plan)

#Skill Pose Sequence
# skill_frame =  "iiwa_right_link_ee"
skill_frame = "multisense/left_camera_optical_frame"
transform_sequence = []
positions = np.load("/home/wyc/Downloads/positions.npy")
for position in positions:
    position = position.tolist()
    transform = make_pose_stamped([position[0], position[1], position[2], 
                                    -0.135, 0.187, 0.690, 0.686], skill_frame)
    transform_sequence.append(transform)

world_tranform_sequence = [tfBuffer.transform(transform, "world", ) for transform in transform_sequence]


#plan to each pose in sequence
for pose in world_tranform_sequence:
    pose_marker_pub.publish(pose)
    rospy.sleep(1.0)
    pose_goal =  tf2_geometry_msgs.PoseStamped()
    pose_goal.header.stamp = rospy.Time(0)
    pose_goal.header.frame_id = "multisense/left_camera_optical_frame"
    pose_goal.pose = pose.pose

    plan = right_move_group.plan(pose_goal)

    raw_input("Press Enter to execute the next step of the plan...")
    # right_move_group.execute(plan)
    # rospy.sleep(3.0)
    # while not rospy.is_shutdown():
    #     try:
    #         print("looking for tf...")
    #         ee_tf = tfBuffer.lookup_transform('iiwa_left_link_ee', "iiwa_right_link_ee", rospy.Time(0))
    #         print(ee_tf)
    #         break  
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
        