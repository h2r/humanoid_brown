#!/usr/bin/env python3

hand_links = [
    'bhand_left_finger_23_link',
    'bhand_left_finger_22_link',
    'bhand_left_finger_21_link',
    'bhand_left_finger_12_link',
    'bhand_left_finger_11_link',
    'bhand_left_finger_13_link',
    'bhand_left_finger_32_link',
    'bhand_left_finger_31_link',
    'bhand_left_finger_33_link',
    'bhand_base_link'
]

joint_names = [
    'bhand_left_j11_joint',
    'bhand_left_j12_joint',
    'bhand_left_j13_joint',
    'bhand_left_j21_joint',
    'bhand_left_j22_joint',
    'bhand_left_j23_joint',
    'bhand_left_j31_joint',
    'bhand_left_j32_joint',
    'bhand_left_j33_joint',
    'bhand_left_knuckle_joint'
]

arm_links = [
    'iiwa_left_link_1',
    'iiwa_left_link_2',
    'iiwa_left_link_3',
    'iiwa_left_link_4',
    'iiwa_left_link_5',
    'iiwa_left_link_6'
]


def generate_srdf():
    srdf = ""

    for joint in joint_names:
        srdf += f"<joint name=\"{joint}\"/>\n"
    for link in hand_links:
        srdf += f"<link name=\"{link}\"/>\n"
    
    srdf += "\n" * 10

    for link1 in hand_links:
        for link2 in hand_links:
            srdf += f"<disable_collisions link1=\"{link1}\" link2=\"{link2}\"/>\n"
        for arm_link in arm_links:
            srdf += f"<disable_collisions link1=\"{link1}\" link2=\"{arm_link}\"/>\n"
    print(srdf)

if __name__ == "__main__":
    generate_srdf()
# This script generates an SRDF file for the humanoid robot model.
# It disables collisions between the hand links and the arm links.
# The generated SRDF file can be used in simulation or for planning purposes.
# The script iterates through all combinations of hand links and arm links,
# and generates the corresponding SRDF entries.

