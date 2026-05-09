#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.parameter import Parameter
import threading
from ament_index_python.packages import get_package_share_directory

from msgs.srv import SetController
from controller_manager_msgs.srv import SwitchController, ListControllers


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('subsystem', Parameter.Type.STRING),
                ('full_arm', Parameter.Type.STRING_ARRAY),
                ('arm', Parameter.Type.STRING_ARRAY),
                ('wrist', Parameter.Type.STRING_ARRAY),
                ('end_effector', Parameter.Type.STRING_ARRAY)
            ])
        self.subsystem = self.get_parameter('subsystem').value
        self.full_arm_controllers = []
        self.arm_controllers = []
        self.wrist_controllers = []
        self.end_effector_controllers = []
        
        # Create separate callback groups
        self.service_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Create separate callback groups
        self.service_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Controllers to always keep active or ignore
        self.always_active = ["joint_state_broadcaster", "motor_status_broadcaster"]
        self.ignore_controllers = []
        
        # Lock to prevent concurrent service processing
        self.service_lock = threading.Lock()
        
        # Create service with callback group
        self.srv = self.create_service(
            SetController, 
            'set_controller', 
            self.set_controller_callback,
            callback_group=self.service_cb_group
        )
        
        # Create clients with separate callback group
        self.switch_client = self.create_client(
            SwitchController, 
            '/controller_manager/switch_controller',
            callback_group=self.client_cb_group
        )
        self.list_client = self.create_client(
            ListControllers, 
            '/controller_manager/list_controllers',
            callback_group=self.client_cb_group
        )
        
        # Wait for services with timeout
        self.get_logger().info('Waiting for controller_manager services...')
        services_ready = False
        for _ in range(30):  # Try for 30 seconds
            if (self.switch_client.wait_for_service(timeout_sec=1.0) and 
                self.list_client.wait_for_service(timeout_sec=1.0)):
                services_ready = True
                break
            self.get_logger().warn('Still waiting for controller_manager services...')
        
        if not services_ready:
            self.get_logger().error('Controller manager services not available!')
            raise RuntimeError('Controller manager services timeout')
            
        self.get_logger().info('ControllerSwitcher ready!')

    def set_controller_callback(self, request, response):
        # Use lock to ensure only one service call is processed at a time
        with self.service_lock:
            return self._handle_set_controller(request, response)

    def _handle_set_controller(self, request, response):
        try:
            requested_controllers = list(request.controller_names) if request.controller_names else []
            
            # Empty list means deactivate all (except always_active and ignored)
            if not requested_controllers:
                self.get_logger().info("Empty controller list - will deactivate all except protected controllers")
            else:
                self.get_logger().info(f"Received request to switch to: {requested_controllers}")
            
            # Get list of all controllers
            list_req = ListControllers.Request()
            self.get_logger().debug("Calling list_controllers service...")
            
            # Call and wait for response
            try:
                list_result = self.list_client.call(list_req)
            except Exception as e:
                response.success = False
                response.message = f"Failed to call list_controllers: {str(e)}"
                return response
            
            if list_result is None:
                response.success = False
                response.message = "Failed to get controller list"
                return response
            
            # Extract controller names and their states
            all_controllers = []
            active_controllers = []
            for c in list_result.controller:
                all_controllers.append(c.name)
                if c.state == 'active':
                    active_controllers.append(c.name)
            
            self.get_logger().info(f"Available controllers: {all_controllers}")
            self.get_logger().info(f"Currently active: {active_controllers}")
            
            if (self.subsystem == 'arm'):
                param_controllers = self.full_arm_controllers + self.arm_controllers + self.wrist_controllers + self.end_effector_controllers + self.always_active + self.ignore_controllers
                if set(all_controllers) != set(param_controllers):
                    missing_controllers = set(all_controllers) - set(param_controllers) 
                    extra_controllers = set(param_controllers) - set(all_controllers)
                    self.get_logger().error(f"Controller list from controller_manager does not match expected controllers from parameters! Here are the missing controllers: {missing_controllers} and here are the extra controllers: {extra_controllers}")
            
            # Check if all requested controllers exist (only if not empty)
            if requested_controllers:
                missing_controllers = []
                for controller in requested_controllers:
                    if controller not in all_controllers:
                        missing_controllers.append(controller)
                
                if missing_controllers:
                    response.success = False
                    response.message = f"Controllers not found: {missing_controllers}. Available: {all_controllers}"
                    return response
            
            # Build lists for activation/deactivation
            controllers_to_activate = []
            controllers_to_deactivate = []

            
            # Always ensure joint_state_broadcaster is active
            if "joint_state_broadcaster" not in active_controllers and "joint_state_broadcaster" not in controllers_to_activate:
                controllers_to_activate.append("joint_state_broadcaster")

            
            def controller_handler(controller, subsystem_controllers):
                if controller not in active_controllers:
                    controllers_to_activate.append(controller)
                controllers_to_deactivate.extend(
                    x for x in subsystem_controllers
                    if x != controller
                    and x in active_controllers
                    and x not in self.always_active
                    and x not in self.ignore_controllers
                )
            
            
            # Arm specific controller switching logic
            if self.subsystem == 'arm' and requested_controllers:
                self.full_arm_controllers = self.get_parameter('full_arm').value
                self.arm_controllers = self.get_parameter('arm').value
                self.wrist_controllers = self.get_parameter('wrist').value
                self.end_effector_controllers = self.get_parameter('end_effector').value

                # Check if any full arm controller is currently active
                active_full_arm = [c for c in active_controllers if c in self.full_arm_controllers]

                if active_full_arm:
                    # Option 1: Switching to another full arm controller
                    if all(c in self.full_arm_controllers for c in requested_controllers):
                        controllers_to_deactivate.extend(active_full_arm) # Deactivate current active full arm controller(s)
                        controllers_to_activate.append(requested_controllers[0])  # Assume only one full arm controller can be active at a time

                    # Option 2: Switching to a subgroup setup (must define one from each type)
                    elif (any(c in self.arm_controllers for c in requested_controllers) and
                        any(c in self.wrist_controllers for c in requested_controllers) and
                        any(c in self.end_effector_controllers for c in requested_controllers)):
                        
                        controllers_to_deactivate.extend(active_full_arm) # Deactivate current active full arm controller(s)
                        controllers_to_activate.extend(list(set(requested_controllers)))

                    # Invalid request: not full arm, not complete subgroup
                    else:
                        self.get_logger().error("Requested controllers must be either a full arm controller or one from each arm/wrist/end-effector.")
                        response.success = False
                        response.message = "Requested controllers must be either a full arm controller or one from each arm/wrist/end-effector."
                        return response

                else:
                    for controller in requested_controllers:
                        if controller in self.always_active or controller in self.ignore_controllers:
                            continue  # Skip protected controllers
                        # Activates requested controller if it is part of a subsection 
                        # and deactivates the rest in the subsection
                        if controller in self.arm_controllers: 
                            controller_handler(controller, self.arm_controllers)
                        elif controller in self.wrist_controllers: 
                            controller_handler(controller, self.wrist_controllers)
                        elif controller in self.end_effector_controllers:
                            controller_handler(controller, self.end_effector_controllers)
                        elif controller in self.full_arm_controllers:
                            if controller not in active_controllers:
                                controllers_to_activate.append(controller)
                            controllers_to_deactivate = [
                                c for c in active_controllers
                                if c not in self.always_active and c not in self.ignore_controllers
                            ]

            else:
                # Default behavior (non-arm or empty request)
                # If not empty, add requested controllers that aren't already active
                if requested_controllers:
                    for controller in requested_controllers:
                        if controller not in active_controllers:
                            controllers_to_activate.append(controller)
            
                for controller in active_controllers:
                    if controller in self.always_active or controller in self.ignore_controllers:
                        continue  # Skip protected controllers
                    if not requested_controllers or controller not in requested_controllers:
                        controllers_to_deactivate.append(controller)
            
            # Check if already in desired state
            if requested_controllers:
                all_requested_active = all(c in active_controllers for c in requested_controllers)
                if all_requested_active and len(controllers_to_deactivate) == 0:
                    response.success = True
                    response.message = f"Controllers {requested_controllers} already active with correct configuration"
                    self.get_logger().info("Already in desired state")
                    return response
            else:
                # For empty list, check if only protected controllers are active
                if len(controllers_to_deactivate) == 0 and len(controllers_to_activate) == 0:
                    response.success = True
                    response.message = "Only protected controllers are active"
                    self.get_logger().info("Already in desired state")
                    return response
            
            # Ensure there are no duplicates in activation/deactivation lists
            controllers_to_activate = list(set(controllers_to_activate))
            controllers_to_deactivate = list(set(controllers_to_deactivate))

            print(self.subsystem)

            self.get_logger().info(f"Will activate: {controllers_to_activate}")
            self.get_logger().info(f"Will deactivate: {controllers_to_deactivate}")
            
            # Skip if nothing to do
            if not controllers_to_activate and not controllers_to_deactivate:
                response.success = True
                response.message = "No changes needed"
                return response
            
            # Perform the switch
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = controllers_to_activate
            switch_req.deactivate_controllers = controllers_to_deactivate
            switch_req.strictness = SwitchController.Request.BEST_EFFORT
            switch_req.start_asap = True
            switch_req.timeout = Duration(seconds=10).to_msg()
            
            self.get_logger().debug("Calling switch_controller service...")
            
            try:
                switch_result = self.switch_client.call(switch_req)
            except Exception as e:
                response.success = False
                response.message = f"Failed to call switch_controller: {str(e)}"
                return response
            
            if switch_result and switch_result.ok:
                response.success = True
                if requested_controllers:
                    active_list = requested_controllers + self.always_active
                    response.message = f"Successfully activated controllers: {requested_controllers}. Active: {active_list}"
                else:
                    response.message = f"Successfully deactivated all controllers except: {self.always_active + self.ignore_controllers}"
                self.get_logger().info(f"Success! {response.message}")
            else:
                response.success = False
                response.message = "Controller switch failed"
                self.get_logger().error("Switch failed")
                
        except Exception as e:
            self.get_logger().error(f"Error in set_controller_callback: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Exception: {str(e)}"
        
        self.get_logger().info(f"Returning response: success={response.success}, message={response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ControllerSwitcher()
        # Use MultiThreadedExecutor with more threads
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start ControllerSwitcher: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()