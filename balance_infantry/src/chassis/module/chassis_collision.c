#include "chassis_types.h"
#include "chassis_collision.h"
#include "chassis_mode.h"
#include "chassis_observer.h"
#include "kalman_service.h"
#include <math.h>

/* Initial thresholds require restrained vehicle trials. The mass comes from
 * the existing model: body 12.01 + 2*leg 1.01 + 2*wheel .51 kg. */
#define COLLISION_MASS_KG 15.05f
#define COLLISION_PUSH_FORCE_N 18.0f
#define COLLISION_HIT_FORCE_N 35.0f
#define COLLISION_HIT_DELTA_V_M_S 0.04f
#define COLLISION_HIT_MIN_TIME_S 0.008f
#define COLLISION_RETREAT_SPEED_M_S 0.18f
#define COLLISION_RETREAT_DISTANCE_M 0.20f
#define COLLISION_RETREAT_TIMEOUT_S 1.8f
#define COLLISION_WHEEL_CURRENT_PER_NM 2598.20f

chassis_collision_t chassis_collision __attribute__((section(".bss.chassis_collision")));

uint8_t chassisCollisionIsActive(void)
{
    return chassis_collision.state != COLLISION_IDLE;
}

static void hold(chassis_collision_t *s)
{
    s->state = COLLISION_HOLD;
    s->stage_time_s = s->neutral_time_s = 0.0f;
    s->speed_ref_m_s = 0.0f;
}

void chassisCollisionProcess(chassis_move_t *c, float dt_s)
{
    chassis_collision_t *s = &chassis_collision;
    chassis_posture_info_t *p = &c->chassis_posture_info;
    /* Special manoeuvres own their commands. Do not reinterpret a stair/jump
     * or recovery as a collision, and never automatically retry a retreat. */
    if (!chassisIsBalancingOnGround(c) || fabsf(c->chassis_data_->wz_set) >= 0.1f)
    {
        const uint32_t events = s->events;
        *s = (chassis_collision_t){.events = events};
        return;
    }
    const float requested = p->foot_speed_set;
    const float speed = chassis_odometry.speed_m_s;
    const float accel = chassis_odometry.forward_accel_m_s2 - ChassisVelocityKF_Get()->accel_bias_m_s2;
    s->observer_valid = chassis_odometry.sample_valid && ChassisVelocityKF_Get()->valid &&
        isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.05f && isfinite(accel) &&
        isfinite(p->yaw_angle_total) && isfinite(speed) && isfinite(chassis_odometry.distance_m) &&
        fabsf(p->pitch_angle) < 0.30f && fabsf(p->roll_angle) < 0.20f &&
        fabsf(p->yaw_gyro) < 0.40f && fabsf(p->pitch_gyro) < 0.60f &&
        fabsf(p->chassis_posture_L.leg_gyro) < 0.5f && fabsf(p->chassis_posture_R.leg_gyro) < 0.5f &&
        fabsf(p->chassis_posture_L.leg_dlength_jacobian) < 0.04f &&
        fabsf(p->chassis_posture_R.leg_dlength_jacobian) < 0.04f;
    if (!s->observer_valid)
    {
        s->ready_time_s = s->push_time_s = s->hit_time_s = 0.0f;
        s->force_filtered_n = s->external_force_n = 0.0f;
        s->impact_delta_v_m_s = 0.0f;
        if (s->state != COLLISION_IDLE) hold(s);
    }
    else
    {
        /* Read measured wheel current, not the unclipped LQR request.
         * Motor polarity is L negative/R positive for forward model force. */
        const float drive_force = ((float)c->foot_motor_R.motor_measure->given_current -
                                    (float)c->foot_motor_L.motor_measure->given_current) /
                                   (COLLISION_WHEEL_CURRENT_PER_NM * WHEEL_RADIUS);
        s->external_force_n = COLLISION_MASS_KG * accel - drive_force;
        s->force_filtered_n += dt_s / (0.04f + dt_s) * (s->external_force_n - s->force_filtered_n);
        s->ready_time_s = fminf(s->ready_time_s + dt_s, 0.3f);
        const uint8_t pushing = fabsf(requested) > 0.2f && fabsf(speed) < 0.08f &&
            s->force_filtered_n * requested < -COLLISION_PUSH_FORCE_N * fabsf(requested);
        /* A short impact can finish before the 40 ms force filter reaches its
         * threshold. Integrate the unfiltered unexplained acceleration instead;
         * require a consistent sign and a minimum duration to reject spikes. */
        const uint8_t hit = fabsf(s->external_force_n) > COLLISION_HIT_FORCE_N && fabsf(accel) > 1.8f &&
            s->external_force_n * accel > 0.0f;
        s->push_time_s = pushing ? s->push_time_s + dt_s : 0.0f;
        if (!hit || s->external_force_n * s->impact_delta_v_m_s < 0.0f)
            s->hit_time_s = s->impact_delta_v_m_s = 0.0f;
        if (hit)
        {
            s->hit_time_s += dt_s;
            s->impact_delta_v_m_s += s->external_force_n / COLLISION_MASS_KG * dt_s;
        }
        const uint8_t impact_confirmed = s->hit_time_s >= COLLISION_HIT_MIN_TIME_S &&
            fabsf(s->impact_delta_v_m_s) >= COLLISION_HIT_DELTA_V_M_S;
        if (s->state == COLLISION_IDLE && s->ready_time_s >= 0.3f &&
            (s->push_time_s >= 0.35f || impact_confirmed))
        {
            s->state = impact_confirmed ? COLLISION_RETREAT : COLLISION_BRAKE;
            const float disturbance = impact_confirmed ? s->impact_delta_v_m_s : s->force_filtered_n;
            s->retreat_direction = disturbance < 0.0f ? -1.0f : 1.0f;
            s->start_distance_m = chassis_odometry.distance_m;
            s->heading_rad = p->yaw_angle_total;
            s->stage_time_s = s->neutral_time_s = s->speed_ref_m_s = 0.0f;
            /* Yield immediately to an impact. A sustained drive into a wall
             * retains the braking stage before requesting reverse motion. */
            if (impact_confirmed && speed * s->retreat_direction > 0.0f)
                s->speed_ref_m_s = s->retreat_direction * fminf(fabsf(speed), COLLISION_RETREAT_SPEED_M_S);
            ++s->events;
        }
    }
    if (s->state == COLLISION_IDLE) return;

    if (s->observer_valid)
    {
        s->stage_time_s += dt_s;
        if (s->state == COLLISION_BRAKE)
        {
            if (s->stage_time_s >= 0.12f && fabsf(speed) < 0.08f)
            {
                s->state = COLLISION_RETREAT;
                s->stage_time_s = 0.0f;
                s->start_distance_m = chassis_odometry.distance_m;
            }
            else if (s->stage_time_s >= 0.6f) hold(s);
        }
        if (s->state == COLLISION_RETREAT)
        {
            const float distance = s->retreat_direction * (chassis_odometry.distance_m - s->start_distance_m);
            const uint8_t blocked_again = s->stage_time_s > 0.25f &&
                s->force_filtered_n * s->retreat_direction < -COLLISION_PUSH_FORCE_N;
            if (distance >= COLLISION_RETREAT_DISTANCE_M || s->stage_time_s >= COLLISION_RETREAT_TIMEOUT_S ||
                distance < -0.05f || blocked_again)
                hold(s);
            else
            {
                const float target = s->retreat_direction * COLLISION_RETREAT_SPEED_M_S;
                s->speed_ref_m_s += fp32_constrain(target - s->speed_ref_m_s, -0.5f * dt_s, 0.5f * dt_s);
            }
        }
        if (s->state == COLLISION_HOLD)
        {
            /* Release-to-rearm prevents an unchanged stick command driving
             * straight back into the obstacle after the retreat. */
            s->neutral_time_s = fabsf(requested) < 0.02f ? s->neutral_time_s + dt_s : 0.0f;
            if (s->neutral_time_s >= 0.2f)
            {
                const uint32_t events = s->events;
                *s = (chassis_collision_t){.events = events};
            }
        }
    }
    p->foot_speed_set = s->speed_ref_m_s;
    /* Release the old parking target immediately, including the detection cycle. */
    p->foot_distance_set = p->target_distance_set = chassis_odometry.distance_m;
    p->position_lock_flag = p->position_lock_state = 0u;
}
