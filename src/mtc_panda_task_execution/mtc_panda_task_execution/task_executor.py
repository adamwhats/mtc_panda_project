import time
from typing import Union

import rclpy
from mtc_panda_task_execution.collision_objects import add_panda_cell, add_walls
from mtc_panda_task_execution.task import *
from mtc_panda_task_execution.task_definitions import demo_task
from mtc_panda_task_execution.utils import plan_and_execute, tf2_to_homogenous
from moveit import MoveItPy
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
from transitions import Machine, State


class TaskExecutor(Node):

    def __init__(self):
        super().__init__('task_executor')  # type: ignore

        # Parameters
        self.declare_parameter('use_fake_hardware', False)
        self.use_sim = self.get_parameter('use_fake_hardware').get_parameter_value()

        # Define state machine
        states = [
            State(name='initializing'),
            State(name='select_task', on_enter=self.select_task_callback),
            State(name='planned_move', on_enter=self.planned_move_callback),
            State(name='servo_move', on_enter=self.servo_move_callback),
            State(name='toggle_vacuum', on_enter=self.toggle_vacuum_callback),
            State(name='inspect', on_enter=self.inspect_request_callback),
            State(name='error', on_enter=self.error_callback)
        ]
        transitions = [
            # Trigger                      Source                            Destination
            ['finish_init',               'initializing',                   'select_task'],
            ['exec_planned_move',         'select_task',                    'planned_move'],
            ['exec_servo_move',           'select_task',                    'servo_move'],
            ['exec_toggle_vacuum',        'select_task',                    'toggle_vacuum'],
            ['exec_inspect',              'select_task',                    'inspect'],
            ['complete_move',             ['planned_move', 'servo_move'],   'select_task'],
            ['complete_action',           ['toggle_vacuum', 'inspect'],     'select_task'],
            ['enter_error_state',         '*',                              'error']
        ]
        self.state_machine = Machine(model=self, states=states, transitions=transitions,
                                     initial='initializing', queued=True, auto_transitions=False)

        # Task sequence management
        self.task_sequence = [*demo_task]
        self.task_counter = 0
        self.current_task = Union[motion_task, modbus_task]

        # Instantiate MoveItPy instance and get planning component
        self.max_velocity_scaling_factor = 0.5
        self.max_acceleration_scaling_factor = 0.3
        self.panda = MoveItPy(node_name="moveit_py_planning_scene")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.get_logger().info("MoveItPy instance created")

        # Populate planning scene
        cell_tf = tf2_to_homogenous(['0.52326', '0.070703', '0.0', '0.0', '0.0', '-0.38269', '0.923877'])
        self.planning_scene_monitor = self.panda.get_planning_scene_monitor()
        add_walls(self.planning_scene_monitor, back_spacing=0.2, left_spacing=0.05, right_spacing=0.5, cell_tf=cell_tf)
        add_panda_cell(self.planning_scene_monitor, cell_tf=cell_tf)
        self.get_logger().info("Planning scene populated")

        # Instantiate moveit servo client
        # self.servo = VisualServoController(ee_frame_name='panda_link8')

        # # Create modbus io client to toggle vacuum
        # from mtc_beckhoff_io_interfaces.srv import SetIO
        # self.vacuum_client = self.create_client(SetIO, 'update_io')
        # while not self.vacuum_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('modbus service not available, waiting again...')

        # Move to ready state
        time.sleep(1)
        self.finish_init()  # type: ignore

    def select_task_callback(self) -> None:
        """
        Based on the task_counter, selects the next task to be executed and transitions to the appropriate state
        """
        # Return the task counter to 0 if if the sequence has completed
        if self.task_counter >= len(self.task_sequence):
            self.task_counter = 0

        # Select the appropriate state for the next task
        self.current_task = self.task_sequence[self.task_counter]
        if isinstance(self.current_task, motion_task):
            self.exec_planned_move()  # type: ignore
        elif isinstance(self.current_task, modbus_task):
            self.exec_toggle_vacuum()  # type: ignore
        else:
            self.get_logger().error(f"Failed to select to suitable state for task of type'{type(self.current_task)}'")
            self.enter_error_state()  # type: ignore

    def planned_move_callback(self) -> None:
        """ 
        Moves from the current robot position to the goal position defined by the current_place_position.
        """
        self.get_logger().info(f"Entered state '{self.state}', task = {self.current_task}")  # type: ignore
        if not isinstance(self.current_task, motion_task):
            self.enter_error_state()  # type: ignore
            return

        # Plan and execute move
        self.panda_arm.set_start_state_to_current_state()

        if isinstance(self.current_task, move_configuration):
            self.panda_arm.set_goal_state(configuration_name=self.current_task.goal)
        elif isinstance(self.current_task, move_pose):
            self.panda_arm.set_goal_state(pose_stamped_msg=self.current_task.goal, pose_link="magtec_tcp")
        elif isinstance(self.current_task, move_joint):
            robot_state = RobotState(self.panda.get_robot_model())
            robot_state.joint_positions = {
                'panda_joint1': self.current_task.goal.q1,
                'panda_joint2': self.current_task.goal.q2,
                'panda_joint3': self.current_task.goal.q3,
                'panda_joint4': self.current_task.goal.q4,
                'panda_joint5': self.current_task.goal.q5,
                'panda_joint6': self.current_task.goal.q6,
                'panda_joint7': self.current_task.goal.q7
            }
            joint_constraint = construct_joint_constraint(
                robot_state=robot_state,
                joint_model_group=self.panda.get_robot_model().get_joint_model_group("panda_arm"),
                tolerance=0.001
            )
            self.panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        plan_parameters = PlanRequestParameters(self.panda, self.current_task.planner)
        plan_parameters.max_velocity_scaling_factor = self.current_task.vel_scale
        plan_parameters.max_acceleration_scaling_factor = self.current_task.acc_scale
        plan_parameters.planning_attempts = 3
        plan_parameters.planning_time = 3.0
        success = plan_and_execute(
            self.panda, self.panda_arm, self.get_logger(), single_plan_parameters=plan_parameters)

        # Progress to next state
        if success:
            self.task_counter += 1
            self.complete_move()  # type: ignore
        else:
            self.get_logger().error("move failed")
            self.enter_error_state()  # type: ignore

    def servo_move_callback(self) -> None:
        """
        Performs visual servoing to better align the gripper before performing the pick/place action
        """
        time.sleep(0.1)
        current_step = self.task_sequence[self.task_counter]
        self.get_logger().info(f"Entered state '{self.state}', {current_step = }")
        self.servo.start_teleop()
        finish_time = time.time() + 0
        while time.time() < finish_time:
            self.servo.publish_command()
        self.servo.stop_teleop()
        time.sleep(0.1)
        self.finish_servo_move()

    def toggle_vacuum_callback(self) -> None:
        """
        Activates or deactivates the vacuum according to the task target
        """
        self.get_logger().info(f"Entered state '{self.state}'")  # type: ignore
        if not isinstance(self.current_task, modbus_task):
            self.enter_error_state()  # type: ignore
            return
        
        if self.use_sim:
            pass
        else:
            req = SetIO.Request()
            req.data = list(self.current_task)
            self.vacuum_client.call_async(req)

        time.sleep(1)
        self.task_counter += 1
        self.complete_action()  # type: ignore

    def inspect_request_callback(self) -> None:
        time.sleep(1)
        
        if self.use_sim:
            time.sleep(1)
            self.complete_action()
        else:
            inspect_request = InspectMagnets.Goal()
            inspect_request.trigger = Empty()
            future = self.inspect_client.send_goal_async(inspect_request)
            future.add_done_callback(self.inspect_response_callback)

    def inspect_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Magnet inspect goal rejected")
            self.enter_error_state()  # type: ignore
            return
        self.get_logger().info("Magnet inspect goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.inspect_result_callback)

    def inspect_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info("Magnet inspect result received")
        self.task_counter += 1
        self.complete_action()  # type: ignore

    def error_callback(self) -> None:
        """ 
        # TODO: Handles error states
        """
        self.get_logger().info(f"Entered state '{self.state}'")


def main(args=None):
    rclpy.init(args=args)
    task_executor = TaskExecutor()
    rclpy.spin(task_executor)
    task_executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
