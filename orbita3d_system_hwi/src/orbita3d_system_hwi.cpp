#include "orbita3d_system_hwi/orbita3d_system_hwi.hpp"

#include "orbita3d.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

std::vector<float> parse_string_as_vec(std::string s)
{
  std::string delimiter = ",";

  std::vector<float> v;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos)
  {
    token = s.substr(0, pos);
    v.push_back(std::stof(token));
    s.erase(0, pos + delimiter.length());
  }
  v.push_back(std::stof(s));

  return v;
}

namespace orbita3d_system_hwi
{
  CallbackReturn
  Orbita3dSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    long unsigned int nb_joints_expected = 3;
    if (info.joints.size() != nb_joints_expected)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Orbita3dSystem"),
                   "Incorrect number of joints, expected %ld, got \"%s\"", nb_joints_expected,
                   std::to_string(info.joints.size()).c_str());
      return CallbackReturn::ERROR;
    }

    long unsigned int nb_gpios_expected = 4; // Actuator + raw_motor_1 + raw_motor_2 + raw_motor_3
    if (info.gpios.size() != nb_gpios_expected)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Orbita3dSystem"),
                   "Incorrect number of gpios, expected %ld, got \"%s\"", nb_gpios_expected,
                   std::to_string(info.gpios.size()).c_str());
      return CallbackReturn::ERROR;
    }

    const char *config_file;
    bool from_config = false;

    for (auto const &params : info.hardware_parameters)
    {
      if (params.first == "config_file")
      {
        config_file = params.second.c_str();
        from_config = true;
      }
      // else, init with "manual" parameters
      else
      {
        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "Parameter \"%s\" = %s", params.first.c_str(), params.second.c_str());
      }
    }

    if (from_config)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("Orbita3dSystem"),
          "Loading config file: %s", config_file);

      if (orbita3d_controller_from_config(config_file, &(this->uid)) != 0)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("Orbita3dSystem"),
            "Error loading config file: %s", config_file);
        return CallbackReturn::ERROR;
      }
    }
    else
    {
      // TODO
      return CallbackReturn::ERROR;
    }

    this->clock_ = rclcpp::Clock();
    RCLCPP_INFO(
        rclcpp::get_logger("Orbita3dSystem"),
        "System \"%s\" init!", info_.name.c_str());

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  Orbita3dSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Set some default values
    auto ret = CallbackReturn::SUCCESS;

    hw_states_torque_ = std::numeric_limits<double>::quiet_NaN();
    loop_counter_read = 0;
    loop_counter_write = 0;

    for (int i = 0; i < 3; i++)
    {
      hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();

      hw_states_torque_limit_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_speed_limit_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();

      hw_states_motor_velocities_[i]=std::numeric_limits<double>::quiet_NaN();
      hw_states_motor_currents_[i]=std::numeric_limits<double>::quiet_NaN();
      hw_states_motor_temperatures_[i]=std::numeric_limits<double>::quiet_NaN();
      hw_states_board_temperatures_[i]=std::numeric_limits<double>::quiet_NaN();

      hw_states_axis_sensors_[i]=std::numeric_limits<double>::quiet_NaN();

    }



    bool initOk=false;
    int nb_tries=1;

    while(!initOk && nb_tries<5)
    {
      initOk=true;

    double rpy[3];

    if (orbita3d_get_target_rpy_orientation(this->uid, &rpy) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "Error getting target orientation");
      // ret= CallbackReturn::ERROR;
      initOk=false;

    }
    else
    {
      // quaternion_to_intrinsic_roll_pitch_yaw(&q, &hw_states_position_);
      hw_states_position_[0] = rpy[0];
      hw_states_position_[1] = rpy[1];
      hw_states_position_[2] = rpy[2];

    }

    rclcpp::sleep_for(std::chrono::milliseconds(10));

    bool torque_on = false;
    if (orbita3d_is_torque_on(this->uid, &torque_on) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "Error getting torque status");
      // ret= CallbackReturn::ERROR;
      initOk=false;

    }
    hw_commands_torque_ = torque_on ? 1.0 : 0.0;
    hw_states_torque_ = torque_on ? 1.0 : 0.0;

    // init the states to the read values
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // Velocity
    if (orbita3d_get_current_velocity(this->uid, &hw_states_velocity_) != 0)
    {

      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ VELOCITY ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // Current torque
    if (orbita3d_get_current_torque(this->uid, &hw_states_effort_) != 0)
    {

      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ CURRENT TORQUE ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }

    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // Torque limit
    if (orbita3d_get_raw_motors_torque_limit(this->uid, &hw_states_torque_limit_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // velocity limit
    if (orbita3d_get_raw_motors_velocity_limit(this->uid, &hw_states_speed_limit_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }


    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // raw motors current
    if (orbita3d_get_raw_motors_current(this->uid, &hw_states_motor_currents_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ RAW MOTOR CURRENTS!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }

    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // raw motors velocity
    if (orbita3d_get_raw_motors_velocity(this->uid, &hw_states_motor_velocities_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ RAW MOTOR VELOCITIES!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }


    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // raw motors temperature
    if (orbita3d_get_motor_temperatures(this->uid, &hw_states_motor_temperatures_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ MOTOR TEMPERATURES !", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }


    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // boards temperature
    if (orbita3d_get_board_temperatures(this->uid, &hw_states_board_temperatures_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ BOARD TEMPERATURES !", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }


    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // absolute sensors
    if (orbita3d_get_axis_sensors(this->uid, &hw_states_axis_sensors_) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ ABSOLUTE SENSORS !", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }



    // PID gains
    double pids[3][3];
    if (orbita3d_get_raw_motors_pid_gains(this->uid, &pids) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ PID GAINS ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }
    else
    {
      for (int i = 0; i < 3; i++)
      {
        hw_states_p_gain_[i] = pids[i][0];
        hw_states_i_gain_[i] = pids[i][1];
        hw_states_d_gain_[i] = pids[i][2];
      }
    }

    // set the commands to the read values (otherwise some strange behaviour can happen)
    hw_commands_torque_ = hw_states_torque_;

    for (int i = 0; i < 3; i++)
    {
      hw_commands_position_[i] = hw_states_position_[i];
      hw_commands_torque_limit_[i] = hw_states_torque_limit_[i];
      hw_commands_speed_limit_[i] = hw_states_speed_limit_[i];
      hw_commands_p_gain_[i] = hw_states_p_gain_[i];
      hw_commands_i_gain_[i] = hw_states_i_gain_[i];
      hw_commands_d_gain_[i] = hw_states_d_gain_[i];
    }

    hw_states_error_ = 0;
    hw_commands_error_ = 0;
    std::uint8_t errors = 0;
    if (orbita3d_get_board_state(this->uid, &errors) != 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ BOARD STATE ERROR!", info_.name.c_str());
      // ret= CallbackReturn::ERROR;
            initOk=false;

    }
    hw_states_error_ = errors;

    //wait
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    if(!initOk && nb_tries<5)
      RCLCPP_INFO_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) INIT FAILED! RETRY (nb_tries: %d)", info_.name.c_str(),nb_tries);



    if(nb_tries==5)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) INIT FAILED! ABANDON!", info_.name.c_str());
      return CallbackReturn::ERROR;
    }
    nb_tries++;
    }



    this->last_timestamp_ = clock_.now();

    RCLCPP_INFO(
        rclcpp::get_logger("Orbita3dSystem"),
        "System \"%s\" successfully started!", info_.name.c_str());
    return ret;
  }

  CallbackReturn
  Orbita3dSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    // TODO: disable torque?

    RCLCPP_INFO(
        rclcpp::get_logger("Orbita3dSystem"),
        "System \"%s\" successfully deactivated!", info_.name.c_str());
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Orbita3dSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (std::size_t i = 0; i < 3; i++)
    {
      auto joint = info_.joints[i];

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
    }

    // motor index (not corresponding to the GPIO index)
    size_t motor_index = 0;

    // be careful GPIO index != motor or actuator index
    for (std::size_t i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];

      if (gpio.name == info_.name.c_str())
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "torque", &hw_states_torque_));

        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "export state interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "errors", &hw_states_error_));

        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "export state interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
      else if (gpio.name.find("raw_motor") != std::string::npos)
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "motor_temperature", &hw_states_motor_temperatures_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "board_temperature", &hw_states_board_temperatures_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "motor_velocities", &hw_states_motor_velocities_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "motor_currents", &hw_states_motor_currents_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "torque_limit", &hw_states_torque_limit_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "speed_limit", &hw_states_speed_limit_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "p_gain", &hw_states_p_gain_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "i_gain", &hw_states_i_gain_[motor_index]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                                        gpio.name, "d_gain", &hw_states_d_gain_[motor_index]));


	// // Temporary location for axis sensors in gpios (it will be used as the standard axis hardware_interface::HW_IF_POSITION)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
					  gpio.name, "axis_sensor", &hw_states_axis_sensors_[motor_index]));


        // next motor
        motor_index++;

        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "export state interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
      else
      {
        RCLCPP_WARN(
            rclcpp::get_logger("Orbita3dSystem"),
            "Unknown state interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
    }

    if (motor_index != 3)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "Orbita3d HWI: Number of motors not correct: expected 3 found %ld! Stopping operation!", motor_index);
      std::abort();
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Orbita3dSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < 3; i++)
    {
      auto joint = info_.joints[i];

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));

      RCLCPP_INFO(
          rclcpp::get_logger("Orbita3dSystem"),
          "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str());
    }

    // motor index (not corresponding to the GPIO index)
    size_t motor_index = 0;

    // be careful GPIO index != motor or actuator index
    for (std::size_t i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];

      if (gpio.name == info_.name.c_str())
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.name.c_str(), "torque", &hw_commands_torque_));

        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "export command interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
      else if (gpio.name.find("raw_motor") != std::string::npos)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "speed_limit", &hw_commands_speed_limit_[motor_index]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "torque_limit", &hw_commands_torque_limit_[motor_index]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "p_gain", &hw_commands_p_gain_[motor_index]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "i_gain", &hw_commands_i_gain_[motor_index]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "d_gain", &hw_commands_d_gain_[motor_index]));

        // next motor
        motor_index++;

        RCLCPP_INFO(
            rclcpp::get_logger("Orbita3dSystem"),
            "export command interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
      else
      {
        RCLCPP_WARN(
            rclcpp::get_logger("Orbita3dSystem"),
            "Unkown command interface (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
    }

    if (motor_index != 3)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "Orbita3d HWI: Number of motors not correct: expected 3 found %ld! Stopping operation!", motor_index);
      std::abort();
    }

    return command_interfaces;
  }

  hardware_interface::return_type
  Orbita3dSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    current_timestamp = clock_.now();
    rclcpp::Duration duration = current_timestamp - last_timestamp_;
    last_timestamp_ = current_timestamp;
    auto ret = hardware_interface::return_type::OK;

    /*
      // The position is returned in the write feedback
    //Position
    double q[4];
    if (orbita3d_get_current_orientation(this->uid, &q) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ ORIENTATION ERROR!", info_.name.c_str()
        );
    } else {
      quaternion_to_intrinsic_roll_pitch_yaw(&q, &hw_states_position_);
    }
    */

    // rclcpp::sleep_for(std::chrono::milliseconds(1));

    // Torque on/off
    bool torque_on = false;
    if (orbita3d_is_torque_on(this->uid, &torque_on) != 0)
    {
      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) Error getting torque status!", info_.name.c_str());
    }
    hw_states_torque_ = torque_on ? 1.0 : 0.0;

    uint8_t errors = 0;

    if (loop_counter_read == 100)
    {
      if (orbita3d_get_board_state(this->uid, &errors) != 0)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("Orbita3dSystem"),
            "(%s) READ BOARD STATE ERROR!", info_.name.c_str());
        // ret= CallbackReturn::ERROR;
      }

      if (orbita3d_get_motor_temperatures(this->uid, &hw_states_motor_temperatures_) != 0)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ MOTOR TEMPERATURES !", info_.name.c_str());
        // ret= CallbackReturn::ERROR;
      }

      // boards temperature
      if (orbita3d_get_board_temperatures(this->uid, &hw_states_board_temperatures_) != 0)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("Orbita3dSystem"),
          "(%s) READ BOARD TEMPERATURES !", info_.name.c_str());
        // ret= CallbackReturn::ERROR;

      }


      loop_counter_read = 0;
    }
    else
    {
      hw_states_error_ = errors;
      loop_counter_read++;
    }

    // rclcpp::sleep_for(std::chrono::milliseconds(1));


    // //Velocity
    if (orbita3d_get_current_velocity(this->uid, &hw_states_velocity_) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ VELOCITY ERROR!", info_.name.c_str()
        );
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(1));

    //Current torque
    if (orbita3d_get_current_torque(this->uid, &hw_states_effort_) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ CURRENT TORQUE ERROR!", info_.name.c_str()
        );
    }


    // Torque limit

    // rclcpp::sleep_for(std::chrono::milliseconds(1));

    if (orbita3d_get_raw_motors_torque_limit(this->uid, &hw_states_torque_limit_) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str()
        );
  }
    // rclcpp::sleep_for(std::chrono::milliseconds(1));

    //velocity limit
    if (orbita3d_get_raw_motors_velocity_limit(this->uid, &hw_states_speed_limit_) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str()
        );
  }


    if (orbita3d_get_raw_motors_velocity(this->uid, &hw_states_motor_velocities_) != 0) {

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ MOTOR VELOCITIES ERROR!", info_.name.c_str()
        );
    }


    if (orbita3d_get_raw_motors_current(this->uid, &hw_states_motor_currents_) != 0) {

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ MOTOR CURRENTS ERROR!", info_.name.c_str()
        );
    }


    // Absolute sensors

    if (orbita3d_get_axis_sensors(this->uid, &hw_states_axis_sensors_) != 0) {

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ ABSOLUTE SENSORS ERROR!", info_.name.c_str()
        );
    }



    // rclcpp::sleep_for(std::chrono::milliseconds(1));

    //PID gains
    double pids[3][3];
    if (orbita3d_get_raw_motors_pid_gains(this->uid, &pids) != 0) {

      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR(
        rclcpp::get_logger("Orbita3dSystem"),
        "(%s) READ PID GAINS ERROR!", info_.name.c_str()
        );
    }
    else{
      for (int i=0; i < 3; i++) {
          hw_states_p_gain_[i] = pids[i][0];
          hw_states_i_gain_[i] = pids[i][1];
          hw_states_d_gain_[i] = pids[i][2];
      }
    }

    return ret;
  }

  hardware_interface::return_type
  Orbita3dSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    auto ret = hardware_interface::return_type::OK;

    //Quaternion mode
    /*
    double q[4];
    intrinsic_roll_pitch_yaw_to_quaternion(&hw_commands_position_, &q);

    double fb[4];
    if (orbita3d_set_target_orientation_fb(this->uid, &q, &fb) != 0)
    {
      // ret=hardware_interface::return_type::ERROR; //do not return error here, this will block the controller... cf: https://design.ros2.org/articles/node_lifecycle.html

      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) WRITE TARGET ORIENTATION ERROR!", info_.name.c_str());

      // Still try to read the position
      double q2[4];
      if (orbita3d_get_current_orientation(this->uid, &q2) != 0)
      {

        // ret=hardware_interface::return_type::ERROR;

        RCLCPP_ERROR(
            rclcpp::get_logger("Orbita3dSystem"),
            "(%s) READ ORIENTATION ERROR!", info_.name.c_str());
      }
      else
      {
        quaternion_to_intrinsic_roll_pitch_yaw(&q2, &hw_states_position_);
      }
    }
    else
    {
      q[0] = fb[0];
      q[1] = fb[1];
      q[2] = fb[2];
      q[3] = fb[3];
      quaternion_to_intrinsic_roll_pitch_yaw(&q, &hw_states_position_);
    }
    // hw_states_velocity_[0] = fb[4];
    // hw_states_velocity_[1] = fb[5];
    // hw_states_velocity_[2] = fb[6];

    // hw_states_effort_[0] = fb[7];
    // hw_states_effort_[1] = fb[8];
    // hw_states_effort_[2] = fb[9];

    */


      // check if turn on and if yes make sure to reset the target to the current position
      if ( (hw_commands_torque_ != 0)  && (hw_commands_torque_ != hw_states_torque_))
      {
        hw_commands_position_[0] = hw_states_position_[0];
        hw_commands_position_[1] = hw_states_position_[1];
        hw_commands_position_[2] = hw_states_position_[2];
      }


    // RPY multiturn mode

    double fb[3];
    if (orbita3d_set_target_rpy_orientation_fb(this->uid, &hw_commands_position_, &fb) != 0)
    {
      // ret=hardware_interface::return_type::ERROR; //do not return error here, this will block the controller... cf: https://design.ros2.org/articles/node_lifecycle.html

      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) WRITE TARGET ORIENTATION ERROR!", info_.name.c_str());


      //Still try to read the position
      double rpy[3];
      if (orbita3d_get_current_rpy_orientation(this->uid, &rpy) != 0) {

        // ret=hardware_interface::return_type::ERROR;

        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
        "(%s) READ ORIENTATION ERROR!", info_.name.c_str()
          );
      } else {
        hw_states_position_[0] = rpy[0];
        hw_states_position_[1] = rpy[1];
        hw_states_position_[2] = rpy[2];

      }

    }
    else{
      hw_states_position_[0] = fb[0];
      hw_states_position_[1] = fb[1];
      hw_states_position_[2] = fb[2];

    }


    // rclcpp::sleep_for(std::chrono::milliseconds(1)); // This one should not be necessary in cached mode but otherwise I have some error (drawback: it reduces the frequency)
    // We only change torque for all axes

    // TODO: we can go with orbita3d_set_raw_torque_limit
    if (hw_commands_torque_ != 0.0)
    {

      // RCLCPP_ERROR_THROTTLE(
      //   rclcpp::get_logger("Orbita3dSystem"),
      //   clock_,
      //   LOG_THROTTLE_DURATION,
      //   "(%s) WRITE TORQUE!", info_.name.c_str()
      //   );

      if (orbita3d_enable_torque(this->uid, true) != 0)
      { // do not reset target
        // ret=hardware_interface::return_type::ERROR;

        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("Orbita3dSystem"),
            clock_,
            LOG_THROTTLE_DURATION,
            "(%s) WRITE TORQUE ERROR!", info_.name.c_str());
      }
    }
    else
    {
      if (orbita3d_disable_torque(this->uid) != 0)
      {
        // ret=hardware_interface::return_type::ERROR;

        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("Orbita3dSystem"),
            clock_,
            LOG_THROTTLE_DURATION,
            "(%s) WRITE TORQUE ERROR!", info_.name.c_str());
      }
    }

    uint8_t errors = hw_commands_error_;

    if (loop_counter_write == 100)
    {
      if (orbita3d_set_board_state(this->uid, &errors) != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("Orbita3dSystem"),
            clock_,
            LOG_THROTTLE_DURATION,
            "(%s) WRITE BOARD STATE ERROR!", info_.name.c_str());
        // ret= CallbackReturn::ERROR;
      }
      loop_counter_write = 0;
    }
    else
    {
      loop_counter_write++;
    }

    // if(torque)
    // {
    //   if (orbita3d_enable_torque(this->uid, false) != 0) { //do not reset target
    //     // ret=hardware_interface::return_type::ERROR;

    //     RCLCPP_ERROR_THROTTLE(
    //       rclcpp::get_logger("Orbita3dSystem"),
    //       clock_,
    //       LOG_THROTTLE_DURATION,
    //       "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
    //       );
    //   }
    // }
    // else{
    //     if (orbita3d_disable_torque(this->uid) != 0) {
    //       // ret=hardware_interface::return_type::ERROR;

    //       RCLCPP_ERROR_THROTTLE(
    //         rclcpp::get_logger("Orbita3dSystem"),
    //         clock_,
    //         LOG_THROTTLE_DURATION,
    //         "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
    //         );
    //     }
    // }

    // speed limit

    // rclcpp::sleep_for(std::chrono::milliseconds(1));
    // float hw_commands_speed_limit_1[3] = {0.2,0.3,0.4};
    if (orbita3d_set_raw_motors_velocity_limit(this->uid, &hw_commands_speed_limit_) != 0)
    {
      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) WRITE SPEED LIMIT ERROR!", info_.name.c_str());
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(1));
    // float hw_commands_torque_limit_1[3] = {0.9,0.8,0.7};
    // torque limit
    if (orbita3d_set_raw_motors_torque_limit(this->uid, &hw_commands_torque_limit_) != 0)
    {
      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("Orbita3dSystem"),
          clock_,
          LOG_THROTTLE_DURATION,
          "(%s) WRITE TORQUE LIMIT ERROR!", info_.name.c_str());
    }

    // rclcpp::sleep_for(std::chrono::milliseconds(1));
    // pid gains
    double pids[3][3];
    for (int i=0; i < 3; i++) {
      pids[i][0] = hw_commands_p_gain_[i];
      pids[i][1] = hw_commands_i_gain_[i];
      pids[i][2] = hw_commands_d_gain_[i];
    }
    if(orbita3d_set_raw_motors_pid_gains(this->uid, &pids) != 0)
    {
      // ret=hardware_interface::return_type::ERROR;

      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("Orbita3dSystem"),
        clock_,
        LOG_THROTTLE_DURATION,
        "(%s) WRITE PID GAINS ERROR!", info_.name.c_str()
        );
    }

    return ret;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    orbita3d_system_hwi::Orbita3dSystem,
    hardware_interface::SystemInterface)
