import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from os import environ

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

environ["ROS_NAMESPACE"]="dorfl"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander(robot_description="dorfl/robot_description")
#scene = moveit_commander.PlanningSceneInterface()


group_name = "left_arm"
move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="dorfl/robot_description",)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

print(dir(robot))
print(dir(robot.get_link('iiwa_left_link_ee')))


planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())

cur_pose = move_group.get_current_pose().pose

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation = cur_pose.orientation
pose_goal.position.x = cur_pose.position.x
pose_goal.position.y = cur_pose.position.y
pose_goal.position.z = cur_pose.position.z - 0.1

plan = move_group.plan(pose_goal)
input()
move_group.execute(plan)


print("")
print(dir(move_group))
