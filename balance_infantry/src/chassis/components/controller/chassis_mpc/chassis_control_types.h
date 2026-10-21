#ifndef WHU_CHASSIS_CONTROL_TYPES_H
#define WHU_CHASSIS_CONTROL_TYPES_H

/* Experimental model interface only; not included by the active controller.
 * State order matches LQR_Legged_Wheeled.m and LQR[4][10].
 * Angles/rates are ground-frame radians/radians per second, not motor angles.
 * Current feedback convention:
 *   theta_L = Q0_L + pitch; theta_R = -Q0_R + pitch.
 * Rates use the corresponding derivatives. Yaw must be unwrapped/continuous.
 * Keep the source model's calibrated sign convention for s, yaw and pitch.
 * No motor polarity conversion or reference/offset subtraction occurs here.
 */
#define CHASSIS_CONTROL_STATE_DIM 10
#define CHASSIS_CONTROL_INPUT_DIM 4

typedef enum
{
    CHASSIS_STATE_S = 0,             /* m */
    CHASSIS_STATE_DS,                /* m/s */
    CHASSIS_STATE_YAW,               /* rad, continuous */
    CHASSIS_STATE_DYAW,              /* rad/s */
    CHASSIS_STATE_THETA_L,           /* rad, ground-frame virtual leg */
    CHASSIS_STATE_DTHETA_L,          /* rad/s */
    CHASSIS_STATE_THETA_R,           /* rad, ground-frame virtual leg */
    CHASSIS_STATE_DTHETA_R,          /* rad/s */
    CHASSIS_STATE_PITCH,             /* rad */
    CHASSIS_STATE_DPITCH             /* rad/s */
} chassis_control_state_index_t;

typedef enum
{
    CHASSIS_INPUT_T_WL = 0,          /* left wheel model torque, N*m */
    CHASSIS_INPUT_T_WR,               /* right wheel model torque, N*m */
    CHASSIS_INPUT_T_BL,               /* left virtual leg model torque, N*m */
    CHASSIS_INPUT_T_BR                /* right virtual leg model torque, N*m */
} chassis_control_input_index_t;

/* x means model coordinates. The model is linearized around zero;
 * a future adapter must consistently account for calibrated equilibrium
 * offsets and references before using it for prediction/control.
 */
typedef struct
{
    float x[CHASSIS_CONTROL_STATE_DIM];
} chassis_control_state_t;

/* u is NOT four joint-motor commands. Hip torque here is the generalized
 * virtual-leg torque T_bl/T_br. Existing VMC maps it, together with support
 * force, to the four physical joint motors. Wheel polarity conversion also
 * belongs to the existing actuator layer and must be applied only once.
 */
typedef struct
{
    float u[CHASSIS_CONTROL_INPUT_DIM];
} chassis_model_input_t;

#endif /* WHU_CHASSIS_CONTROL_TYPES_H */
