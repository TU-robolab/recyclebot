#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

rclpy.init()
moveit = MoveItPy(node_name="ur16e_move_client")

# obtain a PlanningComponent for the manipulator group
arm = moveit.get_planning_component("ur_arm")


planning_state_mon = moveit.get_planning_scene_monitor()
with planning_state_mon.read_only() as scene:
    pose = scene.current_state.get_global_link_transform("tool0")
    ps = PoseStamped()
    ps.header.stamp    = rclpy.time.Time().to_msg()
    ps.header.frame_id = scene.planning_scene.world.frame_id
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose.translation
    ps.pose.orientation.x, ps.pose.orientation.y, \
    ps.pose.orientation.z, ps.pose.orientation.w = pose.rotation  # shorthand
    print(ps)


# ------------------------------------------------------------------
# Plan to a Cartesian pose target for the tool frame ("tool0")
# ------------------------------------------------------------------
# pose_goal = PoseStamped()
# pose_goal.header.frame_id = "base"          # reference frame
# pose_goal.pose.position.x = 0.4
# pose_goal.pose.position.y = 0.0
# pose_goal.pose.position.z = 0.2
# pose_goal.pose.orientation.w = 1.0          # facing forward
# # todo set quaternion x/y/z for orientation

# # use current state as the start and the pose_goal as the target
# arm.set_start_state_to_current_state()
# arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")       # tool0 = UR16e TCP link

# plan_result = arm.plan()
# if plan_result:
#     moveit.execute(plan_result.trajectory, controllers=["scaled_joint_trajectory_controller"])
