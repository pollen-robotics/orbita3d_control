#include "orbita3d.h"
#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// Compile using
// g++ controller.c -I../ -L../../target/debug -lorbita3d_c_api -o controller
// Run using
// ./controller path_to_config_file

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s path_to_config_file\n", argv[0]);
        return 1;
    }

    uint32_t uid;
    if (orbita3d_controller_from_config(argv[1], &uid) != 0) {
        return 1;
    }
    printf("Controller uid: %d\n", uid);

    bool is_on;
    if (orbita3d_is_torque_on(uid, &is_on) != 0) {
        return 1;
    }
    printf("Torque is %s\n", is_on ? "on" : "off");

    printf("Turning torque on\n");
    if (orbita3d_enable_torque(uid, true) != 0) {
        return 1;
    }
    if (orbita3d_is_torque_on(uid, &is_on) != 0) {
        return 1;
    }
    printf("Torque is %s\n", is_on ? "on" : "off");


    printf("Turning torque off\n");
    if (orbita3d_disable_torque(uid) != 0) {
        return 1;
    }
    if (orbita3d_is_torque_on(uid, &is_on) != 0) {
        return 1;
    }
    printf("Torque is %s\n", is_on ? "on" : "off");


    printf("Getting current vel\n");
    double v[3];
    if (orbita3d_get_current_velocity(uid, &v) != 0) {
        return 1;
    }
    printf("\tvel: %f %f %f\n", v[0], v[1], v[2]);


    printf("Getting current torque\n");
    double t[3];
    if (orbita3d_get_current_torque(uid, &t) != 0) {
        return 1;
    }
    printf("\ttorque: %f %f %f\n", t[0], t[1], t[2]);



    printf("Getting current orientation\n");
    double q[4];
    if (orbita3d_get_current_orientation(uid, &q) != 0) {
        return 1;
    }
    printf("\tquat: %f %f %f %f\n", q[0], q[1], q[2], q[3]);

    double rpy[3];
    quaternion_to_intrinsic_roll_pitch_yaw(&q, &rpy);
    printf("\trpy: %f %f %f\n", rpy[0], rpy[1], rpy[2]);

    double target_rpy[3] = {0.0, 0.0, 0.25};
    printf("Setting target orientation to %f %f %f\n", target_rpy[0], target_rpy[1], target_rpy[2]);
    double target_q[4];
    intrinsic_roll_pitch_yaw_to_quaternion(&target_rpy, &target_q);
    if (orbita3d_set_target_orientation(uid, &target_q) != 0) {
        return 1;
    }

    printf("Getting target orientation\n");
    if (orbita3d_get_target_orientation(uid, &q) != 0) {
        return 1;
    }
    quaternion_to_intrinsic_roll_pitch_yaw(&q, &rpy);

    printf("\trpy: %f %f %f\n", rpy[0], rpy[1], rpy[2]);

    printf("Getting current orientation\n");
    if (orbita3d_get_current_orientation(uid, &q) != 0) {
        return 1;
    }
    quaternion_to_intrinsic_roll_pitch_yaw(&q, &rpy);
    printf("\trpy: %f %f %f\n", rpy[0], rpy[1], rpy[2]);


    double feedback[10];
    //Set target orientation with feedback
    if (orbita3d_set_target_orientation_fb(uid, &target_q, &feedback) != 0) {
        return 1;
    }
    printf("Feedback: orientation %f %f %f %f velocity: %f %f %f torque: %f %f %f\n", feedback[0], feedback[1], feedback[2], feedback[3],feedback[4],feedback[5],feedback[6],feedback[7],feedback[8],feedback[9]);



    return 0;
}
