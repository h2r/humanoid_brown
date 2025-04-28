#!/usr/bin/env python

import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
from tf.transformations import quaternion_from_euler

class StaticTransformPublisher:
    def __init__(self, yaml_file):
        with open(yaml_file, 'r') as stream:
            self.obj_yaml_info = yaml.safe_load(stream).get("tags", {})
        
        self.static_tf_broadcaster = StaticTransformBroadcaster()
        self.publish_static_transforms()
    
    def publish_static_transforms(self):
        transforms = []
        for tag_id, tag_info in self.obj_yaml_info.items():
            for relative_frame, transform in tag_info['relative_frames'].items():
                transform_quaternion = quaternion_from_euler(
                    *transform[3:]
                )
                # Create a TransformStamped message
                static_transform = TransformStamped()
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = tag_id.replace("tag", "marker")
                static_transform.child_frame_id = relative_frame
                static_transform.transform.translation.x = transform[0]
                static_transform.transform.translation.y = transform[1]
                static_transform.transform.translation.z = transform[2]
                static_transform.transform.rotation.x = transform_quaternion[0]
                static_transform.transform.rotation.y = transform_quaternion[1]
                static_transform.transform.rotation.z = transform_quaternion[2]
                static_transform.transform.rotation.w = transform_quaternion[3]
                transforms.append(static_transform)
                
                rospy.loginfo("Published static transform for {} with child frame {}".format(tag_id, relative_frame))
        self.static_tf_broadcaster.sendTransform(transforms)



if __name__ == "__main__":
    import os
    rospy.init_node('static_tf_publisher', anonymous=True)
    
    # Path to the YAML file containing the static transforms
    yaml_file = os.path.dirname(os.path.realpath(__file__)) + "/../config/apriltags.yaml"
    
    static_tf_publisher = StaticTransformPublisher(yaml_file)
    
    # Keep the node running
    rospy.spin()

