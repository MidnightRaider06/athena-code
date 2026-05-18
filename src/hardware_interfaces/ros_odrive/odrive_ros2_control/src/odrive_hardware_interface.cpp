
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void logger_function();
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(const Axis& axis);

    bool active_;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;

    // Hardware Interface Parameters
    int update_rate;
    double elapsed_update_time; // Time since last hardware interface update
    double elapsed_time; // Time since first hardware interface update
    double elapsed_logger_time; // Time since last logger update
    int logger_rate; // Logger update rate
    int logger_state; // Logger on/off state
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, int gear_ratio) : can_intf_(can_intf), node_id_(node_id), gear_ratio_(gear_ratio){}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    int gear_ratio_ = 1;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]


    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    uint32_t axis_error_ = 0;
    uint8_t axis_state_ = 0;
    uint8_t procedure_result_ = 0;
    uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = 0; // [rad] ** CHANGED **
    double vel_estimate_ = 0; // [rad/s] ** CHANGED **
    double iq_setpoint_ = NAN;
    double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    uint32_t active_errors_ = 0;
    uint32_t disarm_reason_ = 0;
    double fet_temperature_ = NAN;
    double motor_temperature_ = NAN;
    double bus_voltage_ = NAN;
    double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

void ODriveHardwareInterface::logger_function(){
    int num_joints = static_cast<int>(info_.joints.size());

    // Prevents breaking the logger
    if (num_joints == 0) return;

    // Building Message
    std::string log_msg = "\033[2J\033[H \nODrive Logger";

    // HWI Specific
    std::ostringstream oss;

    oss << "\n--- HWI Specific ---\n"
        << "CAN Interface: " << can_intf_name_
        << " | HWI Update Rate: " << update_rate
        << " | Logger Update Rate: " << logger_rate << "\n"
        << "Elapsed Time since first update: " << elapsed_time << "\n"
        << "\n--- Joint Specific ---";

    for (int i = 0; i < num_joints; ++i) {
        Axis& axis = axes_[i];

        std::string control_mode;
        if (axis.pos_input_enabled_) control_mode = "POSITION";
        else if (axis.vel_input_enabled_) control_mode = "VELOCITY";
        else if (axis.torque_input_enabled_) control_mode = "TORQUE";
        else control_mode = "UNDEFINED";

        // Print parameters, commanded setpoints, enabled inputs, state estimates, and telemetry
        oss << "\nJOINT: " << info_.joints[i].name << "\n"
            << "Parameters: Node ID: 0x" << std::hex << axis.node_id_ << std::dec
            << " | Gear Ratio: " << axis.gear_ratio_ << "\n"
            << "-- Commands (requested) --\n"
            << "Control Mode: " << control_mode << "\n"
            << "Position Setpoint (rad): " << axis.pos_setpoint_
            << " | Motor Input Position (rev): " << (axis.pos_setpoint_ * axis.gear_ratio_) / (2 * M_PI) << "\n"
            << "Velocity Setpoint (rad/s): " << axis.vel_setpoint_
            << " | Motor Input Velocity (rev/s): " << (axis.vel_setpoint_ * axis.gear_ratio_) / (2 * M_PI) << "\n"
            << "Torque Setpoint (Nm): " << axis.torque_setpoint_
            << " | Motor Input Torque (Nm at motor): " << (axis.torque_setpoint_ / std::max(1, axis.gear_ratio_)) << "\n"
            << "Inputs Enabled: pos=" << axis.pos_input_enabled_
            << " vel=" << axis.vel_input_enabled_
            << " torque=" << axis.torque_input_enabled_ << "\n"
            << "-- State (estimates) --\n"
            << "Joint Position (est): " << axis.pos_estimate_
            << " | Joint Velocity (est): " << axis.vel_estimate_
            << " | Joint Torque (est): " << axis.torque_estimate_ << "\n"
            << "-- Telemetry --\n"
            << "FET Temperature: " << axis.fet_temperature_ << " C"
            << " | Motor Temperature: " << axis.motor_temperature_ << " C"
            << " | Bus Voltage: " << axis.bus_voltage_ << " V"
            << " | Bus Current: " << axis.bus_current_ << " A\n"
            << "IQ Setpoint: " << axis.iq_setpoint_ << " | IQ Measured: " << axis.iq_measured_ << "\n"
            << "Active Errors: 0x" << std::hex << axis.active_errors_ << std::dec
            << " | Disarm Reason: " << axis.disarm_reason_ << "\n"
            << "Axis Error: " << axis.axis_error_ << " | Axis State: " << static_cast<int>(axis.axis_state_)
            << " | Procedure Result: " << static_cast<int>(axis.procedure_result_)
            << " | Trajectory Done Flag: " << static_cast<int>(axis.trajectory_done_flag_) << "\n";
    }

    log_msg += oss.str();
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "%s", log_msg.c_str());
}

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
    logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
    logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
    can_intf_name_ = info_.hardware_parameters["can"];

    elapsed_update_time = 0.0;
    elapsed_time = 0.0;
    elapsed_logger_time = 0.0;

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")), std::stoi(joint.parameters.at("gear_ratio")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            "motor_temperature", 
            &axes_[i].motor_temperature_
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            "torque_current",
            &axes_[i].bus_current_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration& period) {
    
    double update_period = 1.0/update_rate;
    elapsed_update_time+=period.seconds();
    elapsed_time+=period.seconds();

    // Logger update
    elapsed_logger_time+=period.seconds();
    double logging_period = 1.0/logger_rate;
    if(elapsed_logger_time > logging_period){
        elapsed_logger_time = 0.0;
        if (logger_state == 1) {
            logger_function();
        }
    }

      // HWI can only go as fast as the controller manager. To limit frequency of bus messages,
    // keep track of time passed over iterations of this function and if it exceeds the 
    // desired frequency of the HWI, skip message
    if(elapsed_update_time > update_period){
        elapsed_update_time = 0.0;
        for (auto& axis : axes_) {
            // Send the CAN message that fits the set of enabled setpoints
            // If I want my arm to have 10 Nm of torque, the motor must use 10 / gear_ratio
            // Very important that this is taken care of here
            if (axis.pos_input_enabled_) {
                Set_Input_Pos_msg_t msg;
                msg.Input_Pos = (axis.pos_setpoint_ * axis.gear_ratio_) / (2 * M_PI);
                msg.Vel_FF = axis.vel_input_enabled_ ? ((axis.vel_setpoint_ * axis.gear_ratio_) / (2 * M_PI)) : 0.0f;
                msg.Torque_FF = axis.torque_input_enabled_ ? (axis.torque_setpoint_ / axis.gear_ratio_) : 0.0f;
                // RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Writing positions for ODrive %d Setpoint: %f, Joint angle of motor (rev): %f", axis.node_id_, axis.pos_setpoint_, (axis.pos_setpoint_ * axis.gear_ratio_) / (2 * M_PI));

                axis.send(msg);
            } else if (axis.vel_input_enabled_) {
                Set_Input_Vel_msg_t msg;
                msg.Input_Vel = (axis.vel_setpoint_ * axis.gear_ratio_) / (2 * M_PI);
                msg.Input_Torque_FF = axis.torque_input_enabled_ ? (axis.torque_setpoint_ / axis.gear_ratio_): 0.0f;
                // RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Writing velocities for ODrive %d Joint velocity of motor (rev/s): %f", axis.node_id_, (axis.vel_setpoint_ * axis.gear_ratio_) / (2 * M_PI));

                axis.send(msg);
            } else if (axis.torque_input_enabled_) {
                Set_Input_Torque_msg_t msg;
                msg.Input_Torque = axis.torque_setpoint_ / axis.gear_ratio_;
                axis.send(msg);
            } else {
                // no control enabled - don't send any setpoint
            }
        }
    }
    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
        case Get_Temperature_msg_t::cmd_id: {
            if(Get_Temperature_msg_t msg; try_decode(msg)) {
                fet_temperature_ = msg.FET_Temperature;
                motor_temperature_ = msg.Motor_Temperature;
            }
        }  break;
        case Get_Bus_Voltage_Current_msg_t::cmd_id: {
            if(Get_Bus_Voltage_Current_msg_t msg; try_decode(msg)) {
                bus_voltage_= msg.Bus_Voltage;
                bus_current_ = msg.Bus_Current;
            }
        } break;
        case Get_Error_msg_t::cmd_id: {
            if(Get_Error_msg_t msg; try_decode(msg)) {
                active_errors_= msg.Active_Errors;
                disarm_reason_ = msg.Disarm_Reason;
            }
        } break;
        case Get_Iq_msg_t::cmd_id: {
            if(Get_Iq_msg_t msg; try_decode(msg)) {
                iq_setpoint_= msg.Iq_Setpoint;
                iq_measured_ = msg.Iq_Measured;
            }
        } break;
        case Heartbeat_msg_t::cmd_id: {
            if(Heartbeat_msg_t msg; try_decode(msg)) {
                axis_error_ = msg.Axis_Error;
                axis_state_ = msg.Axis_State;
                procedure_result_ = msg.Procedure_Result;
                trajectory_done_flag_ = msg.Trajectory_Done_Flag;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
