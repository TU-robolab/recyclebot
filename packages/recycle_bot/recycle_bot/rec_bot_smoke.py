#!/usr/bin/env python3
import rclpy
import rclpy
from moveit.planning import MoveItPy

rclpy.init()
moveit = MoveItPy(node_name="ur16e_move_client")

# obtain a PlanningComponent for the manipulator group
arm = moveit.get_planning_component("ur_manipulator")

# plan to a joint targets
goal = [0.0, -1.4, 1.4, -1.4, -1.57, 0.0]
plan_result = arm.plan_to_joint_positions(goal)
if plan_result:
    arm.execute()     