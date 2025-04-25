#!/bin/python2
import yaml
import rospy
import tf2_msgs
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import Point

class ARTracker:
    def __init__(self, ar_tag_topics, yaml_file):
        with open(yaml_file, 'r') as stream:
            self.obj_yaml_info = yaml.safe_load(stream).get("tags", {})
        
        self.loc_db = {}

        self.subs = [
            rospy.Subscriber(topic, tf2_msgs.msg.TFMessage, self.update_loc_db)
            for topic in ar_tag_topics
        ]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()

    def update_loc_db(self, msg):
        # Update the location database with the new message
        # This is a placeholder for the actual implementation
        pass

    def get_obj_poses(self):
        result = {}
        for tag_id, tag_info in self.obj_yaml_info.items():
            if tag_id in self.loc_db:
                obj_pose = self.loc_db[tag_id]
                # Process the object pose as needed
                print(f"Tag ID: {tag_id}, Pose: {obj_pose}")
                result[tag_id] = obj_pose
            else:
                print(f"Tag ID {tag_id} not found in loc_db.")
        return result
