#ifndef WHU_CHASSIS_MPC_CONFIG_H
#define WHU_CHASSIS_MPC_CONFIG_H
/* Experimental fixed-leg model only. No task or motor integration. */
#define CHASSIS_MPC_KNOTS 8       /* 8 state nodes, 7 input intervals */
#define CHASSIS_MPC_TS_S 0.004f
#define CHASSIS_MPC_LEG_LENGTH_M 0.24f
#define CHASSIS_MPC_MAX_ITER 200
#define CHASSIS_MPC_TOL 0.001f
#define CHASSIS_MPC_RHO 1.0f
/* Model torque limits for numerical experiments, NOT actuator certification. */
#define CHASSIS_MPC_WHEEL_LIMIT_NM 6.0f
#define CHASSIS_MPC_HIP_LIMIT_NM 12.0f
#define CHASSIS_MPC_ACTUATOR_ENABLE 0
#endif
