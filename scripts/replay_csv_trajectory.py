#!/usr/bin/env python
import sys
import csv
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
import actionlib
from os import environ
import tf.transformations as tf_trans
import tf2_ros
from geometry_msgs.msg import TransformStamped

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def read_csv_poses(csv_file_path):
    """Read poses from CSV file and return as list of [x,y,z,qx,qy,qz,qw,time]"""
    poses = []
    with open(csv_file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            if len(row) >= 7:  # Ensure we have x,y,z,qx,qy,qz,qw
                pose = [float(val) for val in row[:8]]  # Include timestamp if available
                poses.append(pose)
    return poses

def make_pose_stamped(pose, frame_id):
    """Convert [x,y,z,qx,qy,qz,qw] to PoseStamped"""
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x = pose[0]
    pose_stamped.pose.position.y = pose[1]
    pose_stamped.pose.position.z = pose[2]
    pose_stamped.pose.orientation.x = pose[3]
    pose_stamped.pose.orientation.y = pose[4]
    pose_stamped.pose.orientation.z = pose[5]
    pose_stamped.pose.orientation.w = pose[6]
    return pose_stamped

def get_current_ee_pose(move_group):
    """Get current end-effector pose as [x,y,z,qx,qy,qz,qw]"""
    current_pose = move_group.get_current_pose().pose
    return [
        current_pose.position.x,
        current_pose.position.y, 
        current_pose.position.z,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ]

def apply_relative_poses_to_current(relative_poses, current_ee_pose):
    """Apply relative poses to current end-effector pose to generate absolute trajectory"""
    if not relative_poses:
        return []
    
    current_position = current_ee_pose[:3]  # [x,y,z]
    current_quat = current_ee_pose[3:7]     # [qx,qy,qz,qw]
    
    # Convert current quaternion to rotation matrix for transforming translations
    current_rotation_matrix = tf_trans.quaternion_matrix(current_quat)[:3, :3]
    
    absolute_poses = []
    
    for relative_pose in relative_poses:
        # Transform relative position by current orientation
        relative_position = relative_pose[:3]  # [x,y,z]
        
        # Rotate the relative translation by the current orientation
        rotated_relative_position = current_rotation_matrix.dot(relative_position)
        
        # Apply rotated relative position to current position
        new_position = [
            current_position[0] + rotated_relative_position[0],  # x
            current_position[1] + rotated_relative_position[1],  # y
            current_position[2] + rotated_relative_position[2]   # z
        ]
        
        # Apply relative orientation
        relative_quat = relative_pose[3:7]  # [qx,qy,qz,qw]
        new_quat = tf_trans.quaternion_multiply(current_quat, relative_quat)
        
        # Combine into full pose
        absolute_pose = new_position + list(new_quat)
        absolute_poses.append(absolute_pose)
    
    return absolute_poses

# def convert_to_relative_poses(poses):
#     """Convert absolute poses to poses relative to the first pose"""
#     if not poses:
#         return []
    
#     first_pose = poses[0]
#     first_position = first_pose[:3]  # First position [x,y,z]
#     first_quat = first_pose[3:7]     # First quaternion [qx,qy,qz,qw]
    
#     # Convert first quaternion to inverse for relative calculations
#     first_quat_inv = tf_trans.quaternion_inverse(first_quat)
    
#     relative_poses = []
    
#     for pose in poses:
#         relative_pose = copy.deepcopy(pose)
        
#         # Make position relative to first pose
#         relative_pose[0] -= first_position[0]  # x
#         relative_pose[1] -= first_position[1]  # y  
#         relative_pose[2] -= first_position[2]  # z
        
#         # Make orientation relative to first orientation
#         current_quat = pose[3:7]  # [qx,qy,qz,qw]
#         relative_quat = tf_trans.quaternion_multiply(first_quat_inv, current_quat)
#         relative_pose[3:7] = relative_quat
        
#         relative_poses.append(relative_pose)
    
#     return relative_poses

def compute_ik_no_collision(move_group, target_pose, ik_service):
    """Compute IK for target pose without collision checking using MoveIt service"""
    try:
        # Create IK request
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = move_group.get_name()
        ik_request.ik_request.pose_stamped = target_pose
        ik_request.ik_request.avoid_collisions = False  # Disable collision checking
        ik_request.ik_request.robot_state.joint_state.name = move_group.get_active_joints()
        ik_request.ik_request.robot_state.joint_state.position = move_group.get_current_joint_values()
        ik_request.ik_request.timeout = rospy.Duration(0.1)
        
        # Call IK service
        response = ik_service(ik_request)
        
        if response.error_code.val == response.error_code.SUCCESS:
            # Extract only the right arm joints from the full robot state
            solution_joint_names = response.solution.joint_state.name
            solution_positions = response.solution.joint_state.position
            
            # Find indices of right arm joints
            right_arm_joint_names = ['iiwa_right_joint_1', 'iiwa_right_joint_2', 'iiwa_right_joint_3', 
                                   'iiwa_right_joint_4', 'iiwa_right_joint_5', 'iiwa_right_joint_6', 'iiwa_right_joint_7']
            
            right_arm_positions = []
            for joint_name in right_arm_joint_names:
                try:
                    idx = solution_joint_names.index(joint_name)
                    right_arm_positions.append(solution_positions[idx])
                except ValueError:
                    rospy.logwarn("Joint {} not found in IK solution".format(joint_name))
                    return None
            
            rospy.logdebug("Full IK solution has {} joints, extracted {} right arm joints".format(
                len(solution_positions), len(right_arm_positions)))
            
            return right_arm_positions
        else:
            rospy.logwarn("IK service call failed: {}".format(response.error_code))
            return None
            
    except Exception as e:
        rospy.logwarn("IK service call failed: {}".format(e))
        return None

def compute_fk(move_group, joint_positions, fk_service):
    """Compute forward kinematics for given joint positions"""
    try:
        # Create FK request
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = "world"
        fk_request.fk_link_names = [move_group.get_end_effector_link()]
        
        # Set joint state
        fk_request.robot_state.joint_state.name = ['iiwa_right_joint_1', 'iiwa_right_joint_2', 'iiwa_right_joint_3', 
                                                  'iiwa_right_joint_4', 'iiwa_right_joint_5', 'iiwa_right_joint_6', 'iiwa_right_joint_7']
        fk_request.robot_state.joint_state.position = joint_positions
        
        # Call FK service
        response = fk_service(fk_request)
        
        if response.error_code.val == response.error_code.SUCCESS and len(response.pose_stamped) > 0:
            pose = response.pose_stamped[0].pose
            return [pose.position.x, pose.position.y, pose.position.z,
                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        else:
            rospy.logwarn("FK computation failed: {}".format(response.error_code))
            return None
            
    except Exception as e:
        rospy.logwarn("FK service call failed: {}".format(e))
        return None

def publish_trajectory_as_tfs(
        target_poses, csv_poses, current_left_ee_pose, 
        joint_solutions=None, fk_poses=None, ik_sample_rate=3, tf_sample_rate=3
    ):
    """Publish trajectory poses as TF transforms for visualization"""
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    rospy.loginfo("=== TRAJECTORY DEBUG INFO ===")
    rospy.loginfo("CSV poses (first 5 from recorded trajectory):")
    for i, pose in enumerate(csv_poses[:5]):
        rospy.loginfo("  CSV[{}]: pos=({:.3f}, {:.3f}, {:.3f}), ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
            i, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]))
    
    rospy.loginfo("Current left EE pose (reference frame):")
    rospy.loginfo("  Left EE: pos=({:.3f}, {:.3f}, {:.3f}), ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
        current_left_ee_pose[0], current_left_ee_pose[1], current_left_ee_pose[2], 
        current_left_ee_pose[3], current_left_ee_pose[4], current_left_ee_pose[5], current_left_ee_pose[6]))
    
    rospy.loginfo("Target poses for right hand (first 5 computed):")
    for i, pose in enumerate(target_poses[:5]):
        rospy.loginfo("  Target[{}]: pos=({:.3f}, {:.3f}, {:.3f}), ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
            i, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]))
    
    # Debug count mismatch
    rospy.loginfo("COUNT ANALYSIS:")
    rospy.loginfo("  CSV poses: {}".format(len(csv_poses)))
    rospy.loginfo("  Target poses: {}".format(len(target_poses)))
    rospy.loginfo("  TF sample rate: every {} pose(s)".format(tf_sample_rate))
    rospy.loginfo("  Expected IK solutions: {}".format(len(target_poses[::ik_sample_rate])))
    rospy.loginfo("  IK sample rate: every {} pose(s)".format(ik_sample_rate))
    if joint_solutions:
        rospy.loginfo("  Actual IK solutions: {}".format(len(joint_solutions)))
        rospy.loginfo("  Joint solutions (first 3):")
        for i, joints in enumerate(joint_solutions[:3]):
            rospy.loginfo("    IK[{}]: {}".format(i, [round(j, 3) for j in joints]))
    
    if fk_poses:
        rospy.loginfo("  FK verification poses: {}".format(len(fk_poses)))
        rospy.loginfo("  FK poses (first 3):")
        for i, pose in enumerate(fk_poses[:3]):
            rospy.loginfo("    FK[{}]: pos=({:.3f}, {:.3f}, {:.3f}), ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
                i, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]))
    
    # Publish target poses as TF transforms
    rate = rospy.Rate(2)  # 2 Hz for visualization
    rospy.loginfo("Publishing {} target poses as TF transforms...".format(len(target_poses[::tf_sample_rate])))
    
    for i, pose in enumerate(target_poses[::tf_sample_rate]):  # Based on sample rate config
        if rospy.is_shutdown():
            break
            
        # Create transform message
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "trajectory_point_{}".format(i)
        
        # Set translation
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        
        # Set rotation
        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]
        
        # Broadcast transform
        tf_broadcaster.sendTransform(t)
        
        rospy.loginfo("Published TF for trajectory point {}: pos=({:.3f}, {:.3f}, {:.3f})".format(
            i, pose[0], pose[1], pose[2]))
        
        rate.sleep()
    
    # Publish FK poses if available
    if fk_poses:
        rospy.loginfo("Publishing {} FK verification poses as TF transforms...".format(len(fk_poses)))
        for i, pose in enumerate(fk_poses):
            if rospy.is_shutdown():
                break
                
            # Create transform message for FK pose
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "fk_point_{}".format(i)
            
            # Set translation
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            
            # Set rotation
            t.transform.rotation.x = pose[3]
            t.transform.rotation.y = pose[4]
            t.transform.rotation.z = pose[5]
            t.transform.rotation.w = pose[6]
            
            # Broadcast transform
            tf_broadcaster.sendTransform(t)
            
            rate.sleep()
    
    rospy.loginfo("=== END TRAJECTORY DEBUG INFO ===")
    rospy.loginfo("TF transforms published. Use RViz to visualize:")
    rospy.loginfo("  - trajectory_point_* frames show computed target poses for right hand")
    rospy.loginfo("  - csv_point_* frames show original recorded relative poses")
    if fk_poses:
        rospy.loginfo("  - fk_point_* frames show FK verification from IK joint solutions")

def execute_joint_trajectory(joint_trajectory_client, joint_names, joint_positions_list, duration_per_point=1.0):
    """Execute a joint trajectory using action client"""
    if not joint_positions_list:
        rospy.logwarn("No joint positions to execute")
        return False
    
    # Create trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    
    # Add trajectory points
    for i, joint_positions in enumerate(joint_positions_list):
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration((i + 1) * duration_per_point)
        goal.trajectory.points.append(point)
    
    # Send goal and wait for resultws/src
    rospy.loginfo("Sending trajectory with {} points...".format(len(joint_positions_list)))
    rospy.loginfo("Goal details: joint_names={}, points={}".format(len(goal.trajectory.joint_names), len(goal.trajectory.points)))
    joint_trajectory_client.send_goal(goal)
    joint_trajectory_client.wait_for_result()
    
    result = joint_trajectory_client.get_result()
    import pdb; pdb.set_trace()  # Debugging breakpoint to inspect result
    return result is not None

def main():
    # Initialize ROS node
    rospy.init_node('csv_trajectory_replayer', anonymous=True)
    
    # Set ROS namespace like in the existing code
    environ["ROS_NAMESPACE"] = "dorfl"
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize robot and move groups
    robot = moveit_commander.RobotCommander(robot_description="dorfl/robot_description")
    right_move_group = moveit_commander.MoveGroupCommander("right_arm", robot_description="dorfl/robot_description")
    left_move_group = moveit_commander.MoveGroupCommander("left_arm", robot_description="dorfl/robot_description")
    
    # Initialize IK service client
    ik_service_name = "/dorfl/compute_ik"
    rospy.loginfo("Waiting for IK service: {}".format(ik_service_name))
    rospy.wait_for_service(ik_service_name, timeout=10.0)
    ik_service = rospy.ServiceProxy(ik_service_name, GetPositionIK)
    
    # Initialize FK service client
    fk_service_name = "/dorfl/compute_fk"
    rospy.loginfo("Waiting for FK service: {}".format(fk_service_name))
    rospy.wait_for_service(fk_service_name, timeout=10.0)
    fk_service = rospy.ServiceProxy(fk_service_name, GetPositionFK)
    
    # Initialize joint trajectory action client for right arm
    joint_trajectory_client = actionlib.SimpleActionClient(
        '/dorfl/iiwa_right/PositionTrajectoryController/follow_joint_trajectory', 
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("Waiting for joint trajectory action server...")
    joint_trajectory_client.wait_for_server(timeout=rospy.Duration(10.0))
    
    rospy.sleep(1.0)  # Allow ROS to initialize
    
    # Configuration
    csv_file_path = "/home/wyc/iiwa_ros_ws/src/humanoid_brown/scripts/tf_hist.csv"
    
    # Sampling configuration
    ik_sample_rate = 3  # Process every Nth pose for IK (1 = all poses, 10 = every 10th)
    tf_sample_rate = 3  # Publish every Nth pose as TF (1 = all poses, 5 = every 5th)
    
    rospy.loginfo("SAMPLING CONFIGURATION:")
    rospy.loginfo("  IK computation: every {} pose(s)".format(ik_sample_rate))
    rospy.loginfo("  TF publishing: every {} pose(s)".format(tf_sample_rate))
    
    try:
        # Read poses from CSV
        rospy.loginfo("Reading poses from {}".format(csv_file_path))
        csv_poses = read_csv_poses(csv_file_path)
        rospy.loginfo("Read {} poses from CSV".format(len(csv_poses)))
        
        if not csv_poses:
            rospy.logerr("No poses found in CSV file")
            return
        
        # Convert CSV poses to relative poses (relative to first pose in trajectory)
        rospy.loginfo("Converting to relative poses...")
        # relative_poses = convert_to_relative_poses(csv_poses)
        
        # Get current left hand end-effector pose
        rospy.loginfo("Getting current left hand pose...")
        current_left_ee_pose = get_current_ee_pose(left_move_group)
        rospy.loginfo("Current left EE pose: pos=({:.3f}, {:.3f}, {:.3f})".format(
            current_left_ee_pose[0], current_left_ee_pose[1], current_left_ee_pose[2]))
        
        # Apply relative trajectory to current left hand pose to get target poses for right hand
        rospy.loginfo("Computing target poses for right hand...")
        target_poses = apply_relative_poses_to_current(csv_poses, current_left_ee_pose)
        
        rospy.loginfo("Computing IK for {} target poses...".format(len(target_poses)))
        
        # Compute IK for each target pose (without collision checking)
        joint_solutions = []
        joint_names = ['iiwa_right_joint_1', 'iiwa_right_joint_2', 'iiwa_right_joint_3', 
                      'iiwa_right_joint_4', 'iiwa_right_joint_5', 'iiwa_right_joint_6', 'iiwa_right_joint_7']
        
        success_count = 0
        
        for i, target_pose_data in enumerate(target_poses[::ik_sample_rate]):  # Sample based on config
            # Convert to PoseStamped
            target_pose = make_pose_stamped(target_pose_data, "world")
            
            rospy.loginfo("Computing IK for pose {}/{}...".format(i+1, len(target_poses[::ik_sample_rate])))
            rospy.loginfo("  Target pose: pos=({:.3f}, {:.3f}, {:.3f}), ori=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            ))
            
            # Compute IK without collision checking
            ik_solution = compute_ik_no_collision(right_move_group, target_pose, ik_service)
            
            if ik_solution is not None:
                joint_solutions.append(ik_solution)
                success_count += 1
                rospy.loginfo("  IK solution found: {}".format([round(j, 3) for j in ik_solution]))
            else:
                rospy.logwarn("  IK failed for pose {}".format(i+1))
        
        rospy.loginfo("Successfully computed IK for {}/{} poses".format(success_count, len(target_poses[::ik_sample_rate])))
        
        # Compute FK from IK solutions for verification
        fk_poses = []
        if joint_solutions:
            rospy.loginfo("Computing FK verification for {} IK solutions...".format(len(joint_solutions)))
            for i, joint_positions in enumerate(joint_solutions):
                fk_pose = compute_fk(right_move_group, joint_positions, fk_service)
                if fk_pose is not None:
                    fk_poses.append(fk_pose)
                    rospy.loginfo("  FK[{}]: pos=({:.3f}, {:.3f}, {:.3f})".format(i, fk_pose[0], fk_pose[1], fk_pose[2]))
                else:
                    rospy.logwarn("  FK failed for IK solution {}".format(i))
        
        # Publish trajectory as TF transforms for visualization
        rospy.loginfo("Publishing trajectory as TF transforms...")
        publish_trajectory_as_tfs(target_poses, csv_poses, current_left_ee_pose, 
                                  joint_solutions, fk_poses, ik_sample_rate, tf_sample_rate)
        
        rospy.loginfo("TF publishing completed. Check RViz to visualize the trajectory.")
        
        # Execute joint trajectory if IK solutions are available
        if joint_solutions:
            rospy.loginfo("=== TRAJECTORY EXECUTION ===")
            rospy.loginfo("Ready to execute trajectory with {} joint solutions".format(len(joint_solutions)))
            
            # Ask for user confirmation before executing
            try:
                raw_input("Press Enter to execute the trajectory on the robot (Ctrl+C to cancel)...")
                
                rospy.loginfo("Executing joint trajectory...")
                success = execute_joint_trajectory(
                    joint_trajectory_client, 
                    joint_names, 
                    joint_solutions, 
                    duration_per_point=0.5  # 0.5 seconds per point for safe motion
                )
                
                if success:
                    rospy.loginfo("Trajectory execution completed successfully!")
                else:
                    rospy.logwarn("Trajectory execution failed")
                    
            except KeyboardInterrupt:
                rospy.loginfo("Trajectory execution cancelled by user")
        else:
            rospy.logwarn("No valid IK solutions found, cannot execute trajectory")
        
    except Exception as e:
        rospy.logerr("Error during trajectory replay: {}".format(e))
        import traceback
        traceback.print_exc()
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()