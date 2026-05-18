ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: propulsion_fl_joint, request_rate: 0}"

ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: stepper_motor_a, request_rate: -1}"

ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: dc_auger, request_rate: -1}"

ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: stepper_motor_a,
  request_rate: 1,
  command_id: 0x60,
  i32_data: [],
  i16_data: [],
  u8_data: [1]
}"

ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: dc_auger,
  request_rate: 1,
  command_id: 0x10,
  i32_data: [],
  i16_data: [],
  u8_data: [2]
}"

ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: propulsion_fl_joint,
  request_rate: -1,
  command_id: 0x81,
  i32_data: [],
  i16_data: [],
  u8_data: []
}"

ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: wrist_roll,
  request_rate: -1,
  command_id: 0x40,
  i32_data: [],
  i16_data: [],
  u8_data: [150]
}"

ros2 topic pub --once /power_module_gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues " 
interface_groups:
- drive_power_module
interface_values:
- interface_names:
  - kill_jetson
  values:
  - 1.0
"

ros2 topic pub --once /drive_led_gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues " 
interface_groups:
- drive_led
interface_values:
- interface_names:
  - red
  values:
  - 255.0
"
