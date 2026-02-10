#!/usr/bin/env python3
import rclpy
import math

from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

ERROR_CODE_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
                    if isinstance(v, int) and k.isupper()}


def main():
    rclpy.init()
    # https://docs.ros.org/en/rolling/p/rclpy/rclpy.logging.html
    log = get_logger("ur16e_move_client")
    moveit = None

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
        # log current tool0 pose as PoseStamped
        # ------------------------------------------------------------------
        psm = moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current = scene.current_state.get_global_link_transform("tool0")
            current_pose = PoseStamped()
            current_pose.header.frame_id = moveit.get_robot_model().model_frame
            current_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
            current_pose.pose.position.x = current.translation[0]
            current_pose.pose.position.y = current.translation[1]
            current_pose.pose.position.z = current.translation[2]
            current_pose.pose.orientation.x = current.rotation[0]
            current_pose.pose.orientation.y = current.rotation[1]
            current_pose.pose.orientation.z = current.rotation[2]
            current_pose.pose.orientation.w = current.rotation[3]
            log.info(f"Current tool0 PoseStamped:\n{current_pose}")

        # ------------------------------------------------------------------
        # plan cartesian pose target for the tool end ("tool0") in base frame
        # ------------------------------------------------------------------
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base"     # UR controller coordinate frame
        pose_goal.header.stamp = rclpy.clock.Clock().now().to_msg() 
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

        if not plan_result:
            log.error("Planning failed: no plan result returned")
            return

        # Support both result variants used by MoveItPy APIs.
        error_code = getattr(plan_result, "error_code", None)
        error_code_val = getattr(error_code, "val", None) if error_code else None
        plan_success = getattr(plan_result, "success", None)
        trajectory = getattr(plan_result, "trajectory", None)

        if error_code_val is not None:
            if error_code_val != MoveItErrorCodes.SUCCESS:
                name = ERROR_CODE_NAMES.get(error_code_val, "UNKNOWN")
                log.error(f"Planning failed: {name} ({error_code_val})")
                return
        elif plan_success is not None:
            if not plan_success:
                status = getattr(plan_result, "status", "unknown")
                log.error(f"Planning failed: {status}")
                return
        elif trajectory is None:
            log.error("Planning failed: missing success/error_code and no trajectory")
            return

        log.info("planning trajectory successful")
        if trajectory is None:
            log.error("Planning failed: plan result contains no trajectory")
            return
        
        # per plan speed scaling with TOTG (https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.robot_trajectory.html)
        trajectory_retimed = trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=0.2,      
            acceleration_scaling_factor=0.2   
        ) 
        
        if not trajectory_retimed:
            log.warn("time parameterization failed, executing raw plan")

        # execute the trajectory with scaled joint planner (blocking)
        exec_result = moveit.execute(trajectory, controllers=["scaled_joint_trajectory_controller"])

        if exec_result:
            log.info("Trajectory execution completed successfully.")
        else:
            log.error("Trajectory execution failed.")
    
    finally:
        if moveit is not None:
            try:
                moveit.shutdown()
            except Exception as exc:
                log.warn(f"MoveIt shutdown failed: {exc}")
        rclpy.shutdown()



if __name__ == "__main__":
    main()
