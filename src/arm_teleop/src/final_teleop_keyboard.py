#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupSequenceAction, MoveGroupSequenceGoal
from geometry_msgs.msg import PoseStamped

class KeyboardMoveGroupNode:
    def __init__(self):
        rospy.init_node('keyboard_move_group_node')

        # Create an action client that connects to the /sequence_move_group action server
        self.client = actionlib.SimpleActionClient('/sequence_move_group', MoveGroupSequenceAction)
        rospy.loginfo("Waiting for /sequence_move_group action server...")
        self.client.wait_for_server()
        rospy.loginfo("/sequence_move_group action server is available.")

        # Initialize goal pose
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "world"
        self.goal_pose.pose.orientation.w = 1.0
        self.goal_pose.pose.position.x = 0.4
        self.goal_pose.pose.position.y = 0.4
        self.goal_pose.pose.position.z = 0.4

    def get_keyboard_input(self):
        print("Use 'W', 'A', 'S', 'D' for X/Y movement, 'I'/'K' for Z movement, and 'Q' to quit.")
        while not rospy.is_shutdown():
            key = input("Enter direction: ").lower()
            if key == 'q':
                break
            elif key == 'w':
                self.goal_pose.pose.position.x += 0.1
            elif key == 's':
                self.goal_pose.pose.position.x -= 0.1
            elif key == 'a':
                self.goal_pose.pose.position.y += 0.1
            elif key == 'd':
                self.goal_pose.pose.position.y -= 0.1
            elif key == 'i':
                self.goal_pose.pose.position.z += 0.1
            elif key == 'k':
                self.goal_pose.pose.position.z -= 0.1
            else:
                print("Invalid key. Please use 'W', 'A', 'S', 'D', 'I', 'K', or 'Q'.")
                continue

            # Publish the goal to the /sequence_move_group
            self.send_goal()

    def send_goal(self):
        # Create a new goal
        goal = MoveGroupSequenceGoal()

        # Fill in the goal with the desired pose
        goal.request.sequence_poses.append(self.goal_pose)

        # Send the goal to the action server and wait for the result
        self.client.send_goal(goal)
        rospy.loginfo("Goal sent to /sequence_move_group.")
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("Goal result: %s", result)

if __name__ == '__main__':
    try:
        node = KeyboardMoveGroupNode()
        node.get_keyboard_input()
    except rospy.ROSInterruptException:
        pass
