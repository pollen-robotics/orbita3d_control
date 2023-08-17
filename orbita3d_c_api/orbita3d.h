#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

struct Orbita3dKinematicsModel {
  double alpha;
  double gamma_min;
  double offset;
  double beta;
  double gamma_max;
  bool passiv_arms_direct;
};

extern "C" {

Orbita3dKinematicsModel create_orbita3d_kinematics_model(double alpha,
                                                         double gamma_min,
                                                         double offset,
                                                         double beta,
                                                         double gamma_max,
                                                         bool passiv_arms_direct);

int32_t orbita3d_kinematics_forward_position(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             double (*quat)[4]);

int32_t orbita3d_kinematics_forward_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*thetas_velocity)[3],
                                             double (*quat_velocity)[4]);

int32_t orbita3d_kinematics_forward_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*thetas_torque)[3],
                                           double (*quat_torque)[4]);

int32_t orbita3d_kinematics_inverse_position(const Orbita3dKinematicsModel *self,
                                             const double (*quat)[4],
                                             double (*thetas)[3]);

int32_t orbita3d_kinematics_inverse_velocity(const Orbita3dKinematicsModel *self,
                                             const double (*thetas)[3],
                                             const double (*quat_velocity)[4],
                                             double (*thetas_velocity)[3]);

int32_t orbita3d_kinematics_inverse_torque(const Orbita3dKinematicsModel *self,
                                           const double (*thetas)[3],
                                           const double (*quat_torque)[4],
                                           double (*thetas_torque)[3]);

} // extern "C"
