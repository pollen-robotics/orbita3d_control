#ifndef _ORBITA3D_SYSTEM_HWI
#define _ORBITA3D_SYSTEM_HWI

#include "rclcpp/macros.hpp"

#include "rclcpp/clock.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#define LOG_THROTTLE_DURATION 30000

namespace orbita3d_system_hwi
{
  using namespace hardware_interface;
class Orbita3dSystem : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Orbita3dSystem)

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  double hw_states_torque_;
  double hw_commands_torque_;

  double hw_states_error_;
  double hw_commands_error_;

  double hw_states_position_[3];
  double hw_states_velocity_[3];
  double hw_states_effort_[3];
  double hw_commands_position_[3];


  double hw_states_torque_limit_[3];
  double hw_states_speed_limit_[3];
  double hw_states_motor_velocities_[3];
  double hw_states_motor_currents_[3];
  double hw_states_motor_temperatures_[3];
  double hw_states_board_temperatures_[3];

    double hw_states_axis_sensors_[3];


  double hw_states_p_gain_[3];
  double hw_states_i_gain_[3];
  double hw_states_d_gain_[3];

  double hw_commands_speed_limit_[3];
  double hw_commands_torque_limit_[3];

  double hw_commands_p_gain_[3];
  double hw_commands_i_gain_[3];
  double hw_commands_d_gain_[3];

  // Store time between update loops
  rclcpp::Clock clock_;
  rclcpp::Time last_timestamp_;
  rclcpp::Time current_timestamp;  // Avoid initialization on each read

  uint32_t uid;
  uint32_t loop_counter_read;
  uint32_t loop_counter_write;


};

}

#endif // _ORBITA3D_SYSTEM_HWI
