#include "chassis_leg_math.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define CLOSE(a,b,tol) CHECK(fabsf((a)-(b)) < (tol))

static void velocity_and_virtual_work(void)
{
    /* Compare analytic velocity with independent position differences.
     * Then verify motor power equals the virtual-leg force/torque power. */
    const float poses[][2] = {{.7f,.9f},{1.0f,1.4f},{1.4f,.8f},
                             {1.8f,1.6f},{2.0f,1.2f},{1.2f,2.0f}};
    const float first_speed = .3f, second_speed = -.2f, step = .0002f;
    for (unsigned i = 0; i < sizeof(poses)/sizeof(poses[0]); ++i) {
        chassis_leg_posture_t center = {0}, before = {0}, after = {0};
        chassis_leg_forward_kinematics(&center, poses[i][0], first_speed,
                                       poses[i][1], second_speed);
        chassis_leg_forward_kinematics(&before, poses[i][0]-first_speed*step, 0,
                                       poses[i][1]-second_speed*step, 0);
        chassis_leg_forward_kinematics(&after, poses[i][0]+first_speed*step, 0,
                                       poses[i][1]+second_speed*step, 0);
        CLOSE(center.leg_dlength,
              (after.leg_length_raw-before.leg_length_raw)/(2*step), .002f);
        CLOSE(center.leg_gyro_local,
              (after.leg_angle_local-before.leg_angle_local)/(2*step), .002f);

        const chassis_leg_jacobian_t n = chassis_leg_inverse_jacobian(&center);
        CLOSE(n.n11*center.leg_dlength + n.n12*center.leg_gyro_local, -first_speed, .0001f);
        CLOSE(n.n21*center.leg_dlength + n.n22*center.leg_gyro_local, second_speed, .0001f);
        for (int right_leg = 0; right_leg <= 1; ++right_leg) {
            const float force = 35, model_torque = .4f;
            const chassis_leg_motor_torque_t torque = chassis_leg_map_wrench(n, force, model_torque, right_leg);
            const float motor_power = (torque.support_first_nm+torque.balance_first_nm)*first_speed +
                                      (torque.support_second_nm+torque.balance_second_nm)*second_speed;
            const float local_torque = right_leg ? -model_torque : model_torque;
            CLOSE(motor_power, force*center.leg_dlength + local_torque*center.leg_gyro_local, .0001f);
        }
    }
}

static void singular_mapping(void)
{
    const chassis_leg_jacobian_t singular = {1,2,2,4};
    const chassis_leg_motor_torque_t torque = chassis_leg_map_wrench(singular, 35, .4f, 0);
    CHECK(torque.support_first_nm == 0 && torque.support_second_nm == 0);
    CHECK(torque.balance_first_nm == 0 && torque.balance_second_nm == 0);
    const chassis_leg_posture_t collapsed = {0};
    const chassis_leg_jacobian_t n = chassis_leg_inverse_jacobian(&collapsed);
    CHECK(n.n11 == 0 && n.n12 == 0 && n.n21 == 0 && n.n22 == 0);
}

int main(void)
{
    velocity_and_virtual_work();
    singular_mapping();
    puts("leg math checks passed: position derivatives, inverse Jacobian, virtual work, singular mapping");
    return 0;
}
