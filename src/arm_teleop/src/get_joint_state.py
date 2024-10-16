#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class JointAngleReader:
    def __init__(self):
        self.joint_angles = {}
        rospy.init_node('joint_angle_reader', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # msg.position contains the angles of the joints
        joint_names = msg.name
        joint_positions = msg.position
        self.joint_angles = dict(zip(joint_names, joint_positions))
        # rospy.loginfo(f"Current Joint Angles: {self.joint_angles}")
        for joint, angle in self.joint_angles.items():
            rospy.loginfo(f"Joint: {joint}, Angle: {angle:.3f}")
        
    def get_joint_angles(self):
        return self.joint_angles

if __name__ == '__main__':
    reader = JointAngleReader()
    try:
        rospy.spin()  # Keeps the node running
    except rospy.ROSInterruptException:
        pass
