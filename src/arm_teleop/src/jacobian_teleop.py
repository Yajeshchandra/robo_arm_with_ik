#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class JointTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('joint_trajectory_publisher', anonymous=True)

        self.q1 = 0.0  # Joint angle 1 (in radians)
        self.q2 = 0.0  # Joint angle 2 (in radians)

        self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.l1 = 0.51  # Link length 1
        self.l2 = 0.36 # Link length 2

        # Example end-effector velocities
        self.vx = 0.1  # Velocity in x-direction (m/s)
        self.vy = 0.1  # Velocity in y-direction (m/s)

    def joint_state_callback(self, msg):
        # Update the joint angles from the JointState message
        self.q1 = msg.position[0]  # Update q1 based on your robot's joint order
        self.q2 = msg.position[1]  # Update q2 based on your robot's joint order

    def inverse_jacobian(self, q1, q2):
        # Calculate the elements of the inverse Jacobian matrix
        denominator_1 = self.l1 * np.sin(q1) * np.sin(q1 - q2) + self.l1 * np.cos(q1) * np.cos(q1 - q2)
        denominator_2 = self.l1 * self.l2 * np.sin(q1) * np.sin(q1 - q2) + self.l1 * self.l2 * np.cos(q1) * np.cos(q1 - q2)
        
        # Elements of the inverse Jacobian matrix
        j11 = np.cos(q1 - q2) / denominator_1
        j12 = -np.sin(q1 - q2) / denominator_1
        j21 = (self.l1 * np.sin(q1) + self.l2 * np.cos(q1 - q2)) / denominator_2
        j22 = (self.l1 * np.cos(q1) - self.l2 * np.sin(q1 - q2)) / denominator_2

        # Construct the inverse Jacobian matrix
        jacobian_inv = np.array([[j11, j12], [j21, j22]])
        
        return jacobian_inv

    def publish_joint_trajectory(self):
        rate = rospy.Rate(10)  # Publish at 10 Hz
        while not rospy.is_shutdown():
            # Calculate the inverse Jacobian matrix
            jacobian_inv = self.inverse_jacobian(self.q1, self.q2)

            # Calculate joint velocities (using inverse Jacobian)
            joint_velocities = np.dot(jacobian_inv, np.array([self.vx, self.vy]))

            # Create a JointTrajectory message
            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = ['joint_1', 'joint_2']  # Replace with actual joint names

            # Create a trajectory point and set the velocities
            point = JointTrajectoryPoint()
            point.positions = [self.q1, self.q2]  # Current joint positions
            point.velocities = joint_velocities.tolist()  # Set the computed joint velocities
            point.time_from_start = rospy.Duration(1.0)  # Duration to reach this point

            joint_trajectory.points.append(point)

            # Publish the joint trajectory
            self.pub.publish(joint_trajectory)
            rospy.loginfo("Published JointTrajectory with velocities: %s", joint_velocities)

            rate.sleep()

if __name__ == '__main__':
    try:
        jt_publisher = JointTrajectoryPublisher()
        jt_publisher.publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
