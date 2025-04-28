import json
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer

OBJECT_NAMES = {
    "pb_jar": ["pb_jar", "pb_jar_cam2"],
    "bread": ["bread"],
    "cup": ["cup"],
}


class ObjectPoseTracker:
    def __init__(self):
        self.tf_buffer = Buffer(cache_time=rospy.Duration(10.0)) # long cache time
        self.tf_listener = TransformListener(self.tf_buffer)
        self.poses = {}

    def _obtain_obj_pose(self, obj_name, remove_unseen_obj=True):
        for obj_alias in OBJECT_NAMES.get(obj_name, []):
            try:
                # Get the transform
                transform = self.tf_buffer.lookup_transform("world", obj_alias, rospy.Time(0), rospy.Duration(10.0))
                self.poses[obj_name] = {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                    "qx": transform.transform.rotation.x,
                    "qy": transform.transform.rotation.y,
                    "qz": transform.transform.rotation.z,
                    "qw": transform.transform.rotation.w,
                }
                return transform
            except Exception as e:
                continue
        rospy.logwarn("Object {} not found in the TF tree.".format(obj_name))
        if remove_unseen_obj:
            self.poses.pop(obj_name, None)
        return None

    def get_updated_pose_info(self):
        rospy.loginfo("Updating object poses. Waiting 4s for transform...")
        rospy.sleep(4.0)
        for obj in OBJECT_NAMES:
            self._obtain_obj_pose(obj)
        return self.poses


if __name__ == "__main__":
    rospy.init_node("object_pose_tracker")
    tracker = ObjectPoseTracker()
    rate = rospy.Rate(10)  # 10 Hz
    poses = tracker.get_updated_pose_info()
    
    with open("object_poses.json", "w") as f:
        json.dump(poses, f)
    for obj_name, pose in poses.items():
        print("Object: {}, Pose: {}".format(obj_name, pose))
