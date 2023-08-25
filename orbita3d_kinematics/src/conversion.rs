use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};

/// Convert a quaternion to a rotation matrix.
pub fn quaternion_to_rotation_matrix(qx: f64, qy: f64, qz: f64, qw: f64) -> Rotation3<f64> {
    Rotation3::from(UnitQuaternion::from_quaternion(Quaternion::new(
        qw, qx, qy, qz,
    )))
}

/// Convert a rotation matrix to a quaternion.
pub fn rotation_matrix_to_quaternion(rot: Rotation3<f64>) -> [f64; 4] {
    let q = UnitQuaternion::from_rotation_matrix(&rot);
    [q.i, q.j, q.k, q.w]
}

/// Convert intrinsic roll-pitch-yaw angles to a rotation matrix.
pub fn intrinsic_roll_pitch_yaw_to_matrix(roll: f64, pitch: f64, yaw: f64) -> Rotation3<f64> {
    let mx = Rotation3::from_axis_angle(&Vector3::x_axis(), roll);
    let my = Rotation3::from_axis_angle(&Vector3::y_axis(), pitch);
    let mz = Rotation3::from_axis_angle(&Vector3::z_axis(), yaw);

    mx * my * mz
}

/// Convert a rotation martix to intrinsic roll-pitch-yaw angles.
pub fn matrix_to_intrinsic_roll_pitch_yaw(rot: Rotation3<f64>) -> [f64; 3] {
    let roll = -rot[(1, 2)].atan2(rot[(2, 2)]);
    let pitch = rot[(0, 2)].atan2((1.0 - rot[(0, 2)].powf(2.0)).sqrt());
    let yaw = -rot[(0, 1)].atan2(rot[(0, 0)]);

    [roll, pitch, yaw]
}

/// Convert from Vector3<f64> to [f64;3]
pub fn vector3_to_array(v: Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}
/// Convert from [f64;3] to Vector3<f64>
pub fn array_to_vector3(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}

#[cfg(test)]
mod tests {
    use rand::Rng;
    use std::f64::consts::PI;

    use super::*;

    const EPSILON: f64 = 1e-6;

    #[test]
    fn rpy() {
        let mut rng = rand::thread_rng();

        let roll = rng.gen_range(-PI..PI).to_radians();
        let pitch = rng.gen_range(-PI..PI).to_radians();
        let yaw = rng.gen_range(-PI..PI).to_radians();

        let rot = intrinsic_roll_pitch_yaw_to_matrix(roll, pitch, yaw);
        let rpy = matrix_to_intrinsic_roll_pitch_yaw(rot);

        assert!((roll - rpy[0]).abs() < EPSILON);
        assert!((pitch - rpy[1]).abs() < EPSILON);
        assert!((yaw - rpy[2]).abs() < EPSILON);
    }

    #[test]
    fn quat() {
        let mut rng = rand::thread_rng();

        // Use euler angles to generate a roation matrix
        let roll = rng.gen_range(-PI..PI).to_radians();
        let pitch = rng.gen_range(-PI..PI).to_radians();
        let yaw = rng.gen_range(-PI..PI).to_radians();
        let rot = intrinsic_roll_pitch_yaw_to_matrix(roll, pitch, yaw);

        // Convert the rotation matrix to a quaternion
        let quat = rotation_matrix_to_quaternion(rot);
        let reconstructed = quaternion_to_rotation_matrix(quat[0], quat[1], quat[2], quat[3]);

        // Check that the reconstructed rotation matrix is the same as the original
        for i in 0..3 {
            for j in 0..3 {
                assert!((rot[(i, j)] - reconstructed[(i, j)]).abs() < 0.0001);
            }
        }
    }
}
