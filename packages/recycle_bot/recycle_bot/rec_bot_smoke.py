#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

rclpy.init()
moveit = MoveItPy(node_name="ur16e_move_client")

# obtain a PlanningComponent for the manipulator group
arm = moveit.get_planning_component("ur_arm")

# ------------------------------------------------------------------
# Plan to a Cartesian pose target for the tool frame ("tool0")
# ------------------------------------------------------------------
pose_goal = PoseStamped()
pose_goal.header.frame_id = "base"          # reference frame
pose_goal.pose.position.x = 0.4
pose_goal.pose.position.y = 0.0
pose_goal.pose.position.z = 0.2
pose_goal.pose.orientation.w = 1.0          # facing forward
# todo set quaternion x/y/z for orientation

# use current state as the start and the pose_goal as the target
arm.set_start_state_to_current_state()
arm.set_goal_state(pose_stamped_msg: pose_goal, pose_link:"tool0")       # tool0 = UR16e TCP link

plan_result = arm.plan()
if plan_result:
    arm.execute(plan_result.trajectory)