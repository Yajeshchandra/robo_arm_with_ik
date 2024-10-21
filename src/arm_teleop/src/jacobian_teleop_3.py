#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class JointTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('joint_trajectory_publisher', anonymous=True)

        self.q1 = 0.0  # Joint angle 1 (in radians)
        self.q2 = 0.0  # Joint angle 2 (in radians)
        
        # Initialize redundant joints to zero or their current positions
        self.q0 = 0.0  # Joint 0 (redundant)
        self.q3 = 0.0  # Joint 3 (redundant)
        self.q4 = 0.0  # Joint 4 (redundant)

        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.l1 = 0.51  # Link length 1
        self.l2 = 0.36 # Link length 2

        # Example end-effector velocities
        self.vx = 0.0  # Velocity in x-direction (m/s)
        self.vy = 0.1  # Velocity in y-direction (m/s)
        
        self.x_cal = 0
        self.y_cal = 0
        
        # To store end-effector path
        self.path_x = []
        self.path_y = []

        # Setup the figure for real-time plotting
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-o', label='End-Effector Path')
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_title("End-Effector Path")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.grid(True)
        self.ax.legend()

        self.anim = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100)
        plt.show()

    def init_plot(self):
        """Initialize the plot background."""
        self.line.set_data([], [])
        return self.line,

    def joint_state_callback(self, msg):
        # Update the joint angles from the JointState message
        self.q0 = msg.position[0]  # Joint 0 (redundant)
        self.q1 = msg.position[1]  # Joint angle 1
        self.q2 = msg.position[2]  # Joint angle 2
        self.q3 = msg.position[3]  # Joint 3 (redundant)
        self.q4 = msg.position[4]  # Joint 4 (redundant)

    def forward_kinematics(self, l1, l2, q1, q2):
        self.x_cal = l1 * np.sin(q1) + l2 * np.cos(q2 - q1)
        self.y_cal = l2 * np.cos(q2) + l2 * np.sin(q2 - q1)

    def inv_jac(self, q1, q2):
        J = np.array([[-self.l1 * np.sin(q1) - self.l2 * np.sin(q1 + q2), -self.l2 * np.sin(q1 + q2)],
                      [self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2), self.l2 * np.cos(q1 + q2)]])
        
        det_J = np.linalg.det(J)
        if np.abs(det_J) < 1e-6:
            rospy.logwarn("Jacobian matrix near singularity. Applying pseudoinverse.")
            damping_factor = 0.01
            jacobian_inv = np.linalg.pinv(J + damping_factor * np.eye(2))
        else:
            jacobian_inv = self.inverse_jacobian(q1,q2)
        
        return jacobian_inv

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
        # Calculate the inverse Jacobian matrix
        jacobian_inv = self.inv_jac(self.q1, self.q2)

        # Calculate joint velocities (using inverse Jacobian)
        joint_velocities = np.dot(jacobian_inv, np.array([self.vx, self.vy]))

        # Ensure joint velocities are within limits (you can define max_joint_vel)
        max_joint_vel = 1.0  # Maximum allowable joint velocity (rad/s)
        joint_velocities = np.clip(joint_velocities, -max_joint_vel, max_joint_vel)

        # Create a JointTrajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']  # Include all joints

        # Create a trajectory point and set the velocities
        point = JointTrajectoryPoint()
        point.positions = [self.q0, self.q1, self.q2, self.q3, self.q4]  # Current joint positions
        point.velocities = [0, joint_velocities[0], joint_velocities[1], 0, 0]  # Only update relevant joint velocities
        point.time_from_start = rospy.Duration(1.0)  # Duration to reach this point

        joint_trajectory.points.append(point)

        # Publish the joint trajectory
        self.pub.publish(joint_trajectory)
        rospy.loginfo("Published JointTrajectory with velocities: %s", joint_velocities)

        # Forward kinematics to verify position
        self.forward_kinematics(self.l1, self.l2, self.q1, self.q2)

        # Store the calculated (x, y) positions for plotting later
        self.path_x.append(self.x_cal)
        self.path_y.append(self.y_cal)

    def update_plot(self, frame):
        """Update the plot with the latest end-effector position."""
        self.publish_joint_trajectory()
        self.line.set_data(self.path_x, self.path_y)
        return self.line,


if __name__ == '__main__':
    try:
        jt_publisher = JointTrajectoryPublisher()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
