use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};

pub fn quaternion_to_rotation_matrix(qx: f64, qy: f64, qz: f64, qw: f64) -> Rotation3<f64> {
    Rotation3::from(UnitQuaternion::from_quaternion(Quaternion::new(
        qw, qx, qy, qz,
    )))
}

pub fn intrinsic_roll_pitch_yaw_to_matrix(roll: f64, pitch: f64, yaw: f64) -> Rotation3<f64> {
    let mx = Rotation3::from_axis_angle(&Vector3::x_axis(), roll);
    let my = Rotation3::from_axis_angle(&Vector3::y_axis(), pitch);
    let mz = Rotation3::from_axis_angle(&Vector3::z_axis(), yaw);

    mx * my * mz
}
