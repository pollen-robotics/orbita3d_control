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
