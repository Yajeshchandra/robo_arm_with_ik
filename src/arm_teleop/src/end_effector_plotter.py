#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from matplotlib.animation import FuncAnimation

class EndEffectorPlotter:
    def __init__(self):
        rospy.init_node('end_effector_plotter', anonymous=True)

        self.q1 = 0.0  # Joint angle 1 (in radians)
        self.q2 = 0.0  # Joint angle 2 (in radians)

        self.l1 = 0.51  # Link length 1
        self.l2 = 0.36  # Link length 2

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

        # Subscribe to the joint states to get the joint angles
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Start the animation
        self.anim = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=100)

    def joint_state_callback(self, msg):
        # Update the joint angles from the JointState message
        self.q1 = msg.position[1]  # Joint angle 1
        self.q2 = msg.position[2]  # Joint angle 2

    def forward_kinematics(self, l1, l2, q1, q2):
        # Calculate end-effector position using forward kinematics
        x_cal = l1 * np.cos(q1) + l2 * np.cos(q2-q1)
        y_cal = l1 * np.sin(q2) + l2 * np.sin(q2-q1)
        return x_cal, y_cal

    def init_plot(self):
        """Initialize the plot background."""
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        """Update the plot with the latest end-effector position."""
        # Calculate the end-effector position using forward kinematics
        x, y = self.forward_kinematics(self.l1, self.l2, self.q1, self.q2)

        # Store the calculated (x, y) positions for plotting later
        self.path_x.append(x)
        self.path_y.append(y)

        # Update the plot
        self.line.set_data(self.path_x, self.path_y)
        return self.line,

    def start(self):
        plt.show()


if __name__ == '__main__':
    try:
        plotter = EndEffectorPlotter()
        plotter.start()
    except rospy.ROSInterruptException:
        pass
