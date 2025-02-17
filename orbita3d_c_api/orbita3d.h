#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Kinematics model for Orbita3d
struct Orbita3dKinematicsModel {
  double alpha;
  double gamma_min;
  double offset;
  double beta;
  double gamma_max;
  bool passiv_arms_direct;
};

extern "C" {

/// Create the Orbita3dController from a config file.
///
/// # Arguments
/// * configfile: *const libc::c_char - The path to the config file.
/// * uid: *mut u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the controller was created successfully, 1 otherwise.
int32_t orbita3d_controller_from_config(const char *configfile, uint32_t *uid);

/// Check if controller is activated (are the motors on)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * is_on: *mut bool - The result of the check.
/// # Returns
/// * i32 - 0 if the controller is activated, 1 otherwise.
int32_t orbita3d_is_torque_on(uint32_t uid, bool *is_on);

/// Enable the controller (turn on the motors)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * reset_target: bool - Reset the target orientation to the current orientation.
/// # Returns
/// * i32 - 0 if the controller was enabled successfully, 1 otherwise.
int32_t orbita3d_enable_torque(uint32_t uid, bool reset_target);

/// Disable the controller (turn off the motors)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the controller was disabled successfully, 1 otherwise.
int32_t orbita3d_disable_torque(uint32_t uid);

/// Get the current orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *mut [f64; 4] - The current orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_current_orientation(uint32_t uid, double (*orientation)[4]);

/// Get the current orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *mut [f64; 3] - The current orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_current_rpy_orientation(uint32_t uid, double (*rpy)[3]);

/// Get the current velocity of the platform
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *mut [f64; 3] - The current velocity of the platform.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_current_velocity(uint32_t uid, double (*velocity)[3]);

/// Get the current torque applied by the actuator
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * torque: *mut [f64; 3] - The current torque applied to the platform.
/// # Returns
/// * i32 - 0 if the torque was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_current_torque(uint32_t uid, double (*torque)[3]);

/// Get the current target orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *mut [f64; 4] - The current target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_target_orientation(uint32_t uid, double (*orientation)[4]);

/// Get the current target orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *mut [f64; 3] - The current target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_target_rpy_orientation(uint32_t uid, double (*rpy)[3]);

/// Set the target orientation of the platform (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *const [f64; 4] - The target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
int32_t orbita3d_set_target_orientation(uint32_t uid, const double (*orientation)[4]);

/// Set the target orientation of the platform (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *const [f64; 3] - The target orientation of the platform.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
int32_t orbita3d_set_target_rpy_orientation(uint32_t uid, const double (*rpy)[3]);

/// Set the tatget orientation and return the current orientation (quaternion)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * orientation: *const [f64; 4] - The target orientation of the platform.
/// * feedback: *mut [f64; 10] - The feedback of the controller.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
int32_t orbita3d_set_target_orientation_fb(uint32_t uid,
                                           const double (*orientation)[4],
                                           double (*feedback)[4]);

/// Set the tatget orientation and return the current orientation (roll, pitch, yaw)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * rpy: *const [f64; 3] - The target orientation of the platform.
/// * feedback: *mut [f64; 10] - The feedback of the controller.
/// # Returns
/// * i32 - 0 if the orientation was set successfully, 1 otherwise.
int32_t orbita3d_set_target_rpy_orientation_fb(uint32_t uid,
                                               const double (*rpy)[3],
                                               double (*feedback)[3]);

/// Get the current velocity of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *mut [f64; 3] - The current velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_raw_motors_velocity_limit(uint32_t uid, double (*limit)[3]);

/// Set the current velocity of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * velocity: *const [f64; 3] - The current velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was set successfully, 1 otherwise.
int32_t orbita3d_set_raw_motors_velocity_limit(uint32_t uid, const double (*limit)[3]);

/// Get the current torque limit of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * limit: *mut [f64; 3] - The current torque limit of the motors.
/// # Returns
/// * i32 - 0 if the torque limit was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_raw_motors_torque_limit(uint32_t uid, double (*limit)[3]);

/// Set the current torque limit of the motors (0 - 1.0)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * limit: *const [f64; 3] - The current torque limit of the motors.
/// # Returns
/// * i32 - 0 if the torque limit was set successfully, 1 otherwise.
int32_t orbita3d_set_raw_motors_torque_limit(uint32_t uid, const double (*limit)[3]);

/// Get the current PID gains of the motors
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * gains: *mut [[f64; 3]; 3] - The current PID gains of the motors.
/// # Returns
/// * i32 - 0 if the PID gains were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_raw_motors_pid_gains(uint32_t uid, double (*gains)[3][3]);

/// Set the current PID gains of the motors
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * gains: *const [[f64; 3]; 3] - The current PID gains of the motors.
/// # Returns
/// * i32 - 0 if the PID gains were set successfully, 1 otherwise.
int32_t orbita3d_set_raw_motors_pid_gains(uint32_t uid, const double (*gains)[3][3]);

/// Get the raw motor current of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * current: *mut [f64; 3] - The current of the motors.
/// # Returns
/// * i32 - 0 if the current was retrieved successfully, 1 otherwise.
uint32_t orbita3d_get_raw_motors_current(uint32_t uid, double (*raw_motors_current)[3]);

/// Get the raw motor velocity of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * raw_motors_velocity: *mut [f64; 3] - The velocity of the motors.
/// # Returns
/// * i32 - 0 if the velocity was retrieved successfully, 1 otherwise.
uint32_t orbita3d_get_raw_motors_velocity(uint32_t uid, double (*raw_motors_velocity)[3]);

/// Get the raw motor position of the actuators
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * raw_motors_position: *mut [f64; 3] - The position of the motors.
/// # Returns
/// * i32 - 0 if the position was retrieved successfully, 1 otherwise.
uint32_t orbita3d_get_raw_motors_position(uint32_t uid, double (*raw_motors_position)[3]);

/// Get the state of the board (BoardState - u8)
/// - See more info [here](../../poulpe_ethercat_controller/register/enum.BoardStatus.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * state: *mut u8 - The state of the board.
/// # Returns
/// * i32 - 0 if the state was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_board_state(uint32_t uid, uint8_t *state);

/// Set the state of the board (BoardState - u8)
/// - See more info [here](../../poulpe_ethercat_controller/register/enum.BoardStatus.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * state: *const u8 - The state of the board.
/// # Returns
/// * i32 - 0 if the state was set successfully, 1 otherwise.
int32_t orbita3d_set_board_state(uint32_t uid, const uint8_t *state);

/// Get the raw axis sensors values
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * axis: *mut [f64; 3] - The axis sensors values.
/// # Returns
/// * i32 - 0 if the axis sensors values were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_axis_sensors(uint32_t uid, double (*axis)[3]);

/// Get the raw axis sensor zeros
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * axis: *mut [f64; 3] - The axis sensor zeros.
/// # Returns
/// * i32 - 0 if the axis sensor zeros were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_axis_sensor_zeros(uint32_t uid, double (*axis)[3]);

/// Get the raw motor temperatures
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * temp: *mut [f64; 3] - The motor temperatures.
/// # Returns
/// * i32 - 0 if the motor temperatures were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_motor_temperatures(uint32_t uid, double (*temp)[3]);

/// Get the raw board temperatures
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * temp: *mut [f64; 3] - The board temperatures.
/// # Returns
/// * i32 - 0 if the board temperatures were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_board_temperatures(uint32_t uid, double (*temp)[3]);

/// Get the error codes
/// - see more info [here](../../poulpe_ethercat_controller/state_machine/struct.ErrorFlags.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * error: *mut [i32; 3] - The error codes.
/// # Returns
/// * i32 - 0 if the error codes were retrieved successfully, 1 otherwise.
int32_t orbita3d_get_error_codes(uint32_t uid, int32_t (*error)[3]);

/// Get the control mode ( CiA402 control mode  - u8) : 1 - Profile Position Mode, 3 - Profile Velocity Mode, 4 - Profile Torque Mode
/// - See more info [here](../../poulpe_ethercat_controller/state_machine/enum.CiA402ModeOfOperation.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * mode: *mut u8 - The control mode.
/// # Returns
/// * i32 - 0 if the control mode was retrieved successfully, 1 otherwise.
int32_t orbita3d_get_control_mode(uint32_t uid,
                                  uint8_t *mode);

/// Set the control mode ( CiA402 control mode  - u8) : 1 - Profile Position Mode, 3 - Profile Velocity Mode, 4 - Profile Torque Mode
/// - See more info [here](../../poulpe_ethercat_controller/state_machine/enum.CiA402ModeOfOperation.html)
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// * mode: *const u8 - The control mode.
/// # Returns
/// * i32 - 0 if the control mode was set successfully, 1 otherwise.
int32_t orbita3d_set_control_mode(uint32_t uid,
                                  const uint8_t *mode);

/// Send the emergency stop signal to the controller
///
/// # Arguments
/// * uid: u32 - The unique identifier of the controller.
/// # Returns
/// * i32 - 0 if the emergency stop signal was sent successfully, 1 otherwise.
int32_t orbita3d_emergency_stop(uint32_t uid);

/// Create a new Orbita3dKinematicsModel.
Orbita3dKinematicsModel create_orbita3d_kinematics_model(double alpha,
                                                         double gamma_min,
                                                         double offset,
                                                         double beta,
                                                         double gamma_max,
                                                         bool passiv_arms_direct);

/// Compute the forward position kinematics.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * quat - Holder for the platform orientation as a quaternion result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_forward_position(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             double (*quat)[4]);

/// Compute the forward velocity.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * thetas_velocity - The motor velocities as a 3-element array.
/// * output_velocity - Holder for the platform orientation velocity as a velocity pseudo vector.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_forward_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*thetas_velocity)[3],
                                             double (*output_velocity)[3]);

/// Compute the forward torque.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * thetas_torque - The motor torques as a 3-element array.
/// * output_torque - Holder for the platform orientation torque as a pseudo vector result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_forward_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*thetas_torque)[3],
                                           double (*output_torque)[3]);

/// Compute the inverse position kinematics.
///
/// # Arguments
/// * quat - The platform orientation as a quaternion.
/// * thetas - Holder for the motor angles as a 3-element array result.
/// # Returns
/// * 0 if success, 1 if no solution, 2 if invalid solution.
int32_t orbita3d_kinematics_inverse_position(const Orbita3dKinematicsModel *self,
                                             const double (*quat)[4],
                                             double (*thetas)[3]);

/// Compute the inverse velocity.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * output_velocity - The platform orientation velocity as a velocity pseudo vector.
/// * thetas_velocity - Holder for the motor velocities as a 3-element array result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_inverse_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*output_velocity)[3],
                                             double (*thetas_velocity)[3]);

/// Compute the inverse torque.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * output_torque - The platform orientation torque as a pseudo vector.
/// * thetas_torque - Holder for the motor torques as a 3-element array result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_inverse_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*output_torque)[3],
                                           double (*thetas_torque)[3]);

int32_t quaternion_to_intrinsic_roll_pitch_yaw(const double (*quat)[4], double (*rpy)[3]);

int32_t intrinsic_roll_pitch_yaw_to_quaternion(const double (*rpy)[3], double (*quat)[4]);

} // extern "C"
