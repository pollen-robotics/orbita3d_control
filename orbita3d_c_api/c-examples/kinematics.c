#include "orbita3d.h"
#include <math.h>

#define DEG2RAD (M_PI / 180.0)

// Compile using
// g++ kinematics.c -I../ -L../../target/debug -lorbita3d_c_api -o kinematics

int main() {
    Orbita3dKinematicsModel m = create_orbita3d_kinematics_model(54.0 * DEG2RAD, 0.0, 0.0, 90.0 * DEG2RAD, 180.0 * DEG2RAD, true);

    double disks[3] = {0.0, 1.0, 0.0};
    double q[4] = {0.0, 0.0, 0.0, 0.0};
    orbita3d_kinematics_forward_position(&m, &disks, &q);

    for (int i = 0; i < 4; i++) {
        printf("%f\n", q[i]);
    }

    double rec[3];
    orbita3d_kinematics_inverse_position(&m, &q, &rec);

    for (int i = 0; i < 3; i++) {
        printf("%f %f\n", rec[i], disks[i]);
    }

    return 0;
}