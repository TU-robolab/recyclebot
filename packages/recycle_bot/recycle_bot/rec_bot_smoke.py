#!/usr/bin/env python3
import rclpy
from moveit.robot import Robot
from moveit.move_group_interface import MoveGroupInterface

def main():
    rclpy.init()
    node = rclpy.create_node("ur16e_move_client")

    # ❶ Create a Robot wrapper (reads URDF + SRDF from /robot_description)
    robot = Robot(node=node, description="robot_description")

    # ❷ Create an interface to the manipulator planning group
    mg = MoveGroupInterface(
        node=node,
        name="ur_manipulator",          # this is the default group in the UR MoveIt config
        robot_description="robot_description")

    # ❸ Plan & execute a simple joint-space goal (radians)
    goal = [0.0, -1.4, 1.4, -1.4, -1.57, 0.0]
    ok = mg.move_to_joint_positions(goal)
    print("Execution success:", ok)

    # ❹ Keep the node spinning long enough for execution feedback
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == "__main__":
    main()