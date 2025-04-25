import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

#!/usr/bin/env python


def collect_tf(src_frame='torso', target_frame='left_ee'):
    rospy.init_node('tf_collector', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tf_hist = []

    try:
        while not rospy.is_shutdown():
            try:
                transform = tf_buffer.lookup_transform(src_frame, target_frame, rospy.Time(0), rospy.Duration(0.2))
                rospy.loginfo("Transform: %s", transform)
                tf_hist.append([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
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
    try:
        tf_hist = collect_tf("torso", "iiwa_right_link_ee")
        with open("tf_hist.csv", "w") as f:
            f.write("x,y,z,qx,qy,qz,qw\n")
            for transform in tf_hist:
                f.write(",".join(map(str, transform)) + "\n")
        rospy.loginfo("TF history saved to tf_hist.csv")
    except rospy.ROSInterruptException:
        pass
