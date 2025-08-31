import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

#!/usr/bin/env python


def collect_tf(src_frame='iiwa_right_link_ee', target_frame='iiwa_left_link_ee'):
    rospy.init_node('tf_collector', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tf_hist = []

    try:
        while not rospy.is_shutdown():
            try:
                print("Collecting transform from {} to {}".format(src_frame, target_frame))
                transform = tf_buffer.lookup_transform(src_frame, target_frame, rospy.Time(0), rospy.Duration(0.2))
                rospy.loginfo("Transform: %s", transform)
                tf_hist.append([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                    transform.header.stamp.to_sec()
                ])
            except tf2_ros.LookupException as e:
                rospy.logwarn("Transform not available: %s", e)
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn("Extrapolation error: %s", e)

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("TF collection stopped.")
    return tf_hist

if __name__ == '__main__':
    import argparse
    import os

    parser = argparse.ArgumentParser(description='Collect TF transforms between two frames.')
    parser.add_argument('--src_frame', type=str, default='iiwa_left_link_ee', help='Source frame')
    parser.add_argument('--target_frame', type=str, default='iiwa_right_link_ee', help='Target frame')
    parser.add_argument('--skill_name', type=str, required=True, help='Skill name for output file')
    args = parser.parse_args()

    try:
        tf_hist = collect_tf(src_frame=args.src_frame, target_frame=args.target_frame)
        if not os.path.exists("trajs"):
            os.makedirs("trajs", exist_ok=True)
        with open("trajs/skill_{}.csv".format(args.skill_name), "w") as f:
            f.write("x,y,z,qx,qy,qz,qw,time\n")
            for transform in tf_hist:
                f.write(",".join(map(str, transform)) + "\n")
        rospy.loginfo("TF history saved to trajs/skill_{}.csv".format(args.skill_name))
    except rospy.ROSInterruptException:
        pass
