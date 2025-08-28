#!/usr/bin/env python3
import rclpy
import math

from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    # https://docs.ros.org/en/rolling/p/rclpy/rclpy.logging.html
    log = get_logger("ur16e_move_client")

    try:    
        moveit = MoveItPy(node_name="ur16e_move_client")
        
        # obtain a PlanningComponent for the manipulator group
        arm = moveit.get_planning_component("ur_arm")
        
        
        # planning_state_mon = moveit.get_planning_scene_monitor()
        # with planning_state_mon.read_only() as scene:
        #     pose = scene.current_state.get_global_link_transform("tool0")
        #     ps = PoseStamped()
        #     ps.header.frame_id = scene.planning_scene.world.frame_id
        #     ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose.translation
        #     ps.pose.orientation.x, ps.pose.orientation.y, \
        #     ps.pose.orientation.z, ps.pose.orientation.w = pose.rotation  # shorthand
        #     print(ps)
        
        
        # ------------------------------------------------------------------
        # plan cartesian pose target for the tool end ("tool0") in base frame 
        # ------------------------------------------------------------------
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base"          # reference frame
        pose_goal.pose.position.x = 0.116
        pose_goal.pose.position.y = -0.468
        pose_goal.pose.position.z = 0.874
        pose_goal.pose.orientation.w = 0.324          # facing forward
        pose_goal.pose.orientation.x = 0.330 
        pose_goal.pose.orientation.y = -0.646
        pose_goal.pose.orientation.z = 0.606 
        
        # set quaternion x/y/z for orientation
        q = pose_goal.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        
        if norm <= 0.0:
            log.error("Invalid quaternion (norm=0), aborting.")
            return
            
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm 
        pose_goal.pose.orientation = q
        
        # use current state as the start and the pose_goal as the target, start plan
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")       # tool0 = UR16e TCP link
        plan_result = arm.plan()

        print(plan_result)
        log.info("planning trajectory successful")
        trajectory = plan_result.trajectory
        
        # per plan speed scaling with TOTG (https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.robot_trajectory.html)
        # trajectory_retimed = trajectory.apply_totg_time_parameterization(
        #     velocity_scaling_factor=0.2,      
        #     acceleration_scaling_factor=0.2   
        # ) 


        # execute the trajectory with scaled joint planner
        moveit.execute(trajectory, controllers=["scaled_joint_trajectory_controller"])
        log.info("Executed trajectory.")
    
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()