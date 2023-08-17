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
/// * quat_velocity - Holder for the platform orientation velocity as a quaternion result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_forward_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*thetas_velocity)[3],
                                             double (*quat_velocity)[4]);

/// Compute the forward torque.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * thetas_torque - The motor torques as a 3-element array.
/// * quat_torque - Holder for the platform orientation torque as a quaternion result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_forward_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*thetas_torque)[3],
                                           double (*quat_torque)[4]);

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
/// * quat_velocity - The platform orientation velocity as a quaternion.
/// * thetas_velocity - Holder for the motor velocities as a 3-element array result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_inverse_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*quat_velocity)[4],
                                             double (*thetas_velocity)[3]);

/// Compute the inverse torque.
///
/// # Arguments
/// * thetas - The motor angles as a 3-element array.
/// * quat_torque - The platform orientation torque as a quaternion.
/// * thetas_torque - Holder for the motor torques as a 3-element array result.
/// # Returns
/// * 0 if success, 1 if error.
int32_t orbita3d_kinematics_inverse_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*quat_torque)[4],
                                           double (*thetas_torque)[3]);

} // extern "C"
