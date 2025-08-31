import json
import os
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, TransformException, LookupException


OBJECT_NAMES = {
    # "pb_jar": ["peanut_butter_jar", "peanut_butter_jar_cam2"],
    # "bread1": ["plate_dorfl"], # offset added at the end
    "red_cup": ["025_mug_shifted"],#, "red_cup", "025_mug"],
    # "knife1": ["032_knife_shifted"],
    # "cracker_box": ["003_cracker_box"],
    # "spam_box": ["010_potted_meat_can"],
}


class ObjectPoseTracker:
    def __init__(self):
        self.tf_buffer = Buffer(cache_time=rospy.Duration(60.0)) # long cache time
        self.tf_listener = TransformListener(self.tf_buffer)
        self.poses = {}

    def _obtain_obj_pose(self, obj_name, remove_unseen_obj=True):
        for obj_alias in OBJECT_NAMES.get(obj_name, []):
            try:
                print(obj_alias)
                # Get the transform
                transform = self.tf_buffer.lookup_transform("world", obj_alias, rospy.Time(0), rospy.Duration(0.1))
                self.poses[obj_name] = {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                    "qx": transform.transform.rotation.x,
                    "qy": transform.transform.rotation.y,
                    "qz": transform.transform.rotation.z,
                    "qw": transform.transform.rotation.w,
                }
                rospy.loginfo("Found object {} at pose: {}".format(obj_name, self.poses[obj_name]))
                return transform
            except:
                pass
        rospy.logwarn("Object {} not found in the TF tree.".format(obj_name))
        if remove_unseen_obj:
            self.poses.pop(obj_name, None)
        return None

    def get_updated_pose_info(self):
        rospy.loginfo("Updating object poses. Waiting 4s for transform...")
        rospy.sleep(4.0)
        
        objects_to_find = set(OBJECT_NAMES.keys())
        
        while objects_to_find:
            found_objects = set()
            for obj in objects_to_find:
                if self._obtain_obj_pose(obj, remove_unseen_obj=False):
                    found_objects.add(obj)
            objects_to_find -= found_objects
            if objects_to_find:
                rospy.sleep(0.1)  # Brief pause before retrying remaining objects
        
        return self.poses


if __name__ == "__main__":
    rospy.init_node("object_pose_tracker")
    tracker = ObjectPoseTracker()
    rate = rospy.Rate(10)  # 10 Hz
    poses = tracker.get_updated_pose_info()
    

    # override object poses
    if os.path.exists("object_poses.json"):
        with open("object_poses.json", "r") as f:
            poses_old = json.load(f)
        for key in poses_old:
            if key not in poses:
                print("Object {} not found in new poses but found in old poses".format(key))
                print("Their value will be kept.")
                poses[key] = poses_old[key]
    with open("object_poses.json", "w") as f:
        json.dump(poses, f)
    for obj_name, pose in poses.items():
        print("Object: {}, Pose: {}".format(obj_name, pose))
