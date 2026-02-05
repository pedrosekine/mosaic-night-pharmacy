#!/usr/bin/env python3
"""
xArm6 + MoveItPy demo (ROS 2 Humble)
- mirrors the official MoveItPy tutorial structure
- shows named-state planning, random RobotState goal, PoseStamped goal,
  joint-constraint goal, and (optional) multi-pipeline planning
- prints current EE pose before each motion
"""

import time
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time # Import the Time class

# moveit python api
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

import tf2_ros
#from tf2_ros import Buffer, TransformListener, TransformException
from bitbots_tf_buffer import Buffer, TransformListener
from std_srvs.srv import Trigger

# msgs
from geometry_msgs.msg import PoseStamped
import os

import tf2_geometry_msgs 
from xarm_msgs.srv import VacuumGripperCtrl


GROUP_NAME = "xarm6"         
BASE_LINK = "link_base"
EE_LINK = "link1"         # sometimes "link_eef" depending on your xArm config
        
class TestMoveIt(Node):
    def __init__(self):
        super().__init__(
            'xarm_planner_node',
            # picks up params from your launch without explicit declare()
            automatically_declare_parameters_from_overrides=True

        )

        # Get the marker_id parameter (automatically declared due to automatically_declare_parameters_from_overrides=True)
        # Check if the parameter was provided
        if not self.has_parameter('marker_id'):
            self.get_logger().error('marker_id parameter is required! Use: --ros-args -p marker_id:=marker_X')
            raise ValueError('marker_id parameter must be provided')

        self.marker_id = self.get_parameter('marker_id').value
        self.get_logger().info(f'Using marker: {self.marker_id}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

        moveit_config = (
            MoveItConfigsBuilder(
                        #robot_ip="192.168.1.209",
                        robot_ip="192.168.1.196",
                        controllers_name="controllers",
                        ros2_control_plugin="uf_robot_hardware/UFRobotSystemHardware",
                        robot_type="xarm",
                        add_oakd_poe=True,
                        add_vacuum_gripper=True,
                        dof=6,
                    )
                    .to_moveit_configs()
        )
        moveit_config_dict = moveit_config.to_dict()
        moveit_config_dict["planning_pipelines"]= {
                "pipeline_names": ["ompl"],
                "default_planning_pipeline": "ompl",
            }
        moveit_config_dict["plan_request_params"] = {
                    "planning_attempts": 5,
                    "planning_pipeline": "ompl",
                    "max_velocity_scaling_factor": 0.1,
                    "max_acceleration_scaling_factor": 1.0
        }
        self.moveit = MoveItPy(
            node_name="xarm_planner_node",
            config_dict=moveit_config_dict
        )

        self.xarm = self.moveit.get_planning_component(GROUP_NAME)
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        self.vacuum_gripper_client = self.create_client(VacuumGripperCtrl, "/xarm/set_vacuum_gripper")
        while not self.vacuum_gripper_client.wait_for_service(timeout_sec=1.0):
            rclpy.spin_once(self)
            time.sleep(0.5)
            print("Waiting for vacuum gripper service...")
        self.transform_to_marker = None


    def on_timer(self):
        try:
            self.transform_to_marker = self.tf_buffer.lookup_transform(
                target_frame='link_base', 
                source_frame = self.marker_id,
                time=rclpy.time.Time())
            print(f'Found transform from link_base to {self.marker_id}') # from current position to target position
        except Exception as ex:
            print(f'Could not transform {self.marker_id} to link_base')
            self.transform_to_marker = None

    def go_to_default(self, position):
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_goal_state(configuration_name = position)
        planning_result = self.xarm.plan()
        if planning_result:
            print(f'Found plan! Going there now! go_to_default')
            robot_trajectory = planning_result.trajectory
            return_value = self.moveit.execute(robot_trajectory=robot_trajectory,
                                               controllers=[])
            print(f"The status of the execution is {return_value}")
        else:
            print("Was not able to find a plan :(")

    def go_to_position(self, x, y, z):
        self.xarm.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "link_base"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = 1. 
        goal_pose.pose.orientation.y = 0.
        goal_pose.pose.orientation.z = 0.
        goal_pose.pose.orientation.w = 0.
        self.xarm.set_goal_state(pose_stamped_msg=goal_pose, pose_link="link_tcp")
        planning_result = self.xarm.plan()
        if planning_result:
            print("Found plan! Going there now!")
            robot_trajectory = planning_result.trajectory
            return_value = self.moveit.execute(robot_trajectory=robot_trajectory,
                                               controllers=[])
            print(f"The status of the execution is {return_value}")
        else:
            print("Was not able to find a plan :(")

    def go_to_aruco(self):
        while not self.tf_buffer.can_transform("link_base", self.marker_id, rclpy.time.Time()):
            rclpy.spin_once(self)
            time.sleep(2.0)
            print("Cannot transform in go_to_aruco")

        print("Found Transform in go_to_aruco")

        while self.transform_to_marker is None:
            rclpy.spin_once(self)
            time.sleep(1.0)
            print("Cannot transform in go_to_aruco")
        transform = self.transform_to_marker

        self.xarm.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "link_base"
        goal_pose.pose.position.x = transform.transform.translation.x
        goal_pose.pose.position.y = transform.transform.translation.y + 0.1
        goal_pose.pose.position.z = transform.transform.translation.z - 0.05

        print(f'Goal pose position: {goal_pose.pose.position}')
        
        goal_pose.pose.orientation.x = 0.5 
        goal_pose.pose.orientation.y = 0. 
        goal_pose.pose.orientation.z = 0. 
        goal_pose.pose.orientation.w = 0.8660254
        self.xarm.set_goal_state(pose_stamped_msg=goal_pose, pose_link="link_tcp")
        planning_result = self.xarm.plan()
        if planning_result:
            print("Found plan! Going there now!")
            robot_trajectory = planning_result.trajectory
            return_value = self.moveit.execute(robot_trajectory=robot_trajectory,
                                               controllers=[])
            print(f"The status of the execution is {return_value}")
        else:
            print("Was not able to find a plan :(")

    # Function that is going be great for the medicine boxes
    # def move_to_box(self):
    #     rclpy.spin_once(self)

    #     target_frame = "link_base"
    #     source_frame = "link_tcp"
    #     self.xarm.set_start_state_to_current_state()
    #     pose_goal = PoseStamped()
    #     pose_goal.header.frame_id = source_frame #
    #     pose_goal.header.stamp = Time().to_msg() 
    #     pose_goal.pose.orientation.x = 0. #
    #     pose_goal.pose.orientation.y = 0. #
    #     pose_goal.pose.orientation.z = 0. #
    #     pose_goal.pose.orientation.w = 1.0 #
    #     pose_goal.pose.position.x = 0. #
    #     pose_goal.pose.position.y = 0. #
    #     pose_goal.pose.position.z = 0.105 #

    #     # Wait for the transform to be available
    #     while not self.tf_buffer.can_transform(
    #         target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0)):
    #         rclpy.spin_once(self)
    #         time.sleep(0.5)
    #         print("Cannot move to box")
    #     print("Approaching the box")

    #     # Perform the transformation
    #     transformed_pose = self.tf_buffer.transform(pose_goal,
    #         target_frame,
    #         timeout=Duration(seconds=2.0)
    #     )    
    #     self.xarm.set_start_state_to_current_state()
    #     self.xarm.set_goal_state(pose_stamped_msg=transformed_pose, pose_link="link_tcp")
    #     planning_result = self.xarm.plan()
    #     if planning_result:
    #         print("Found plan for Move to Box! Going there now!")
    #         robot_trajectory = planning_result.trajectory
    #         return_value = self.moveit.execute(robot_trajectory=robot_trajectory,
    #                                            controllers=[])
    #         print(f"The status of the execution is {return_value}")
    #     else:
    #         print("Was not able to find a plan to Move to Box:(")

    def move_relative_to_tcp(self, x, y, z):
        rclpy.spin_once(self)

        target_frame = "link_base"
        source_frame = "link_tcp"
        self.xarm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = source_frame #
        pose_goal.header.stamp = Time().to_msg() 
        pose_goal.pose.orientation.x = 0. #
        pose_goal.pose.orientation.y = 0. #
        pose_goal.pose.orientation.z = 0. #
        pose_goal.pose.orientation.w = 1.0 #
        pose_goal.pose.position.x = x #
        pose_goal.pose.position.y = y #
        pose_goal.pose.position.z = z #

        # Wait for the transform to be available
        while not self.tf_buffer.can_transform(
            target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0)):
            rclpy.spin_once(self)
            time.sleep(0.5)
            print("Cannot transform")
        print("Transforming")

        # Perform the transformation
        transformed_pose = self.tf_buffer.transform(pose_goal,
            target_frame,
            timeout=Duration(seconds=2.0)
        )    
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_goal_state(pose_stamped_msg=transformed_pose, pose_link="link_tcp")
        planning_result = self.xarm.plan()
        if planning_result:
            print("Found plan! Going there now!")
            robot_trajectory = planning_result.trajectory
            return_value = self.moveit.execute(robot_trajectory=robot_trajectory,
                                               controllers=[])
            print(f"The status of the execution is {return_value}")
        else:
            print("Was not able to find a plan :(")


    def vacuum_activator(self, on):
        request = VacuumGripperCtrl.Request()
        request.on = on
        self.vacuum_gripper_client.call_async(request)
        print("Box picked up!")

    def wait_seconds(self, seconds):
        start = time.time()
        while time.time() - start < seconds:
            rclpy.spin_once(self)
            time.sleep(0.01)

def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()    
    node = TestMoveIt()
    node.vacuum_activator(False)

    node.go_to_default("look-diagonal") 

    node.wait_seconds(0.5)

    node.go_to_aruco()
    node.wait_seconds(0.5)
    node.move_relative_to_tcp(0.015, 0.0, 0.101)

    node.wait_seconds(0.5)
    node.vacuum_activator(True)

    node.wait_seconds(0.5)

    node.move_relative_to_tcp(0.0, 0.10, 0.0)

    node.wait_seconds(0.5)

    node.move_relative_to_tcp(0.0, 0.0, -0.15)
    node.wait_seconds(0.5)
    
    node.go_to_position(0., 0.44, 0.18)

    node.wait_seconds(0.5)
    node.vacuum_activator(False)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()