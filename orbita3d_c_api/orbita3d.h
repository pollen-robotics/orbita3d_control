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

int32_t orbita3d_controller_from_config(const char *configfile, uint32_t *uid);

int32_t orbita3d_is_torque_on(uint32_t uid, bool *is_on);

int32_t orbita3d_enable_torque(uint32_t uid, bool reset_target);

int32_t orbita3d_disable_torque(uint32_t uid);

int32_t orbita3d_get_current_orientation(uint32_t uid, double (*orientation)[4]);

int32_t orbita3d_get_current_velocity(uint32_t uid, double (*velocity)[3]);

int32_t orbita3d_get_current_torque(uint32_t uid, double (*torque)[3]);

int32_t orbita3d_get_target_orientation(uint32_t uid, double (*orientation)[4]);

int32_t orbita3d_set_target_orientation(uint32_t uid, const double (*orientation)[4]);

int32_t orbita3d_set_target_orientation_fb(uint32_t uid,
                                           const double (*orientation)[4],
                                           double (*feedback)[4]);

int32_t orbita3d_get_raw_motors_velocity_limit(uint32_t uid, double (*limit)[3]);

int32_t orbita3d_set_raw_motors_velocity_limit(uint32_t uid, const double (*limit)[3]);

int32_t orbita3d_get_raw_motors_torque_limit(uint32_t uid, double (*limit)[3]);

int32_t orbita3d_set_raw_motors_torque_limit(uint32_t uid, const double (*limit)[3]);

int32_t orbita3d_get_raw_motors_pid_gains(uint32_t uid, double (*gains)[3][3]);

int32_t orbita3d_set_raw_motors_pid_gains(uint32_t uid, const double (*gains)[3][3]);

int32_t orbita3d_get_board_state(uint32_t uid, uint8_t *state);

int32_t orbita3d_set_board_state(uint32_t uid, const uint8_t *state);

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
