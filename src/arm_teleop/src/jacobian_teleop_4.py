#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import threading  # For running matplotlib plot in a separate thread


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
        self.l2 = 0.36  # Link length 2

        # Desired end-effector velocities
        self.vx = 0.1  # Velocity in x-direction (m/s)
        self.vy = 0.0  # Velocity in y-direction (m/s)

        self.x_cal = 0.0
        self.y_cal = 0.0
        
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

        # Start the animation in a separate thread
        self.plot_thread = threading.Thread(target=self.start_animation)
        self.plot_thread.daemon = True  # Make sure this thread doesn't block ROS shutdown
        self.plot_thread.start()

    def start_animation(self):
        """Run the matplotlib animation."""
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
        # Calculate end-effector position using forward kinematics
        self.x_cal = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        self.y_cal = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
        
    def inverse_jacobian(self, q1, q2):
        # Calculate the elements of the inverse Jacobian matrix
        jacobian_inv = np.linalg.pinv(np.array([
            [-self.l1 * np.sin(q1) - self.l2 * np.sin(q1 + q2), -self.l2 * np.sin(q1 + q2)],
            [self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2), self.l2 * np.cos(q1 + q2)]
        ]))
        
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
            joint_trajectory.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']  # Include all joints

            # Create a trajectory point and set the velocities
            point = JointTrajectoryPoint()
            point.positions = [self.q0, self.q1, self.q2, self.q3, self.q4]  # Current joint positions
            point.velocities = [0, joint_velocities[0], joint_velocities[1], 0, 0]  # Only update relevant joint velocities
            point.time_from_start = rospy.Duration(0.1)  # Duration to reach this point

            joint_trajectory.points.append(point)

            # Publish the joint trajectory
            self.pub.publish(joint_trajectory)
            rospy.loginfo("Published JointTrajectory with velocities: %s", joint_velocities)

            # Update the forward kinematics (end-effector position)
            self.forward_kinematics(self.l1, self.l2, self.q1, self.q2)

            # Store the calculated (x, y) positions for plotting later
            self.path_x.append(self.x_cal)
            self.path_y.append(self.y_cal)

            rate.sleep()

    def update_plot(self, frame):
        """Update the plot with the latest end-effector position."""
        self.line.set_data(self.path_x, self.path_y)
        return self.line,


if __name__ == '__main__':
    try:
        jt_publisher = JointTrajectoryPublisher()
        jt_publisher.publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
