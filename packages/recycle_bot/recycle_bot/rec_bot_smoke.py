#!/usr/bin/env python3
import rclpy
import rclpy
from moveit.planning import MoveItPy

rclpy.init()
moveit = MoveItPy(node_name="ur16e_move_client")

# obtain a PlanningComponent for the manipulator group
arm = moveit.get_planning_component("ur_arm")

# plan to a joint targets
goal = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.4,
    "elbow_joint": 1.4,
    "wrist_1_joint": -1.4,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}

plan_result = arm.plan(goal)           # ⬅️  was plan_to_joint_positions()
   # execute the returned plan
if plan_result:
    arm.execute(plan_result.trajectory)   