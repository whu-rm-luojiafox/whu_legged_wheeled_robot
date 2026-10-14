#ifndef WHU_CHASSIS_MPC_H
#define WHU_CHASSIS_MPC_H
#include <stdint.h>
#include "chassis_control_types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    CHASSIS_MPC_OK = 0,
    CHASSIS_MPC_NOT_INITIALIZED = 1,
    CHASSIS_MPC_BAD_INPUT = 2,
    CHASSIS_MPC_NOT_CONVERGED = 3,
    CHASSIS_MPC_NUMERIC_ERROR = 4
} chassis_mpc_status_t;
typedef struct {
    chassis_model_input_t command; /* zero unless accepted */
    chassis_model_input_t candidate; /* diagnostic only; never send on failure */
    uint32_t iterations;
    uint32_t valid;
    float primal_residual;
    float dual_residual;
    float input_bound_violation;
    uint32_t matrix_payload_bytes; /* excludes allocator overhead/solve temporaries */
} chassis_mpc_result_t;
/* Single instance, NOT reentrant. Init once before solving; heap required.
 * x/reference are deviations from the exported zero equilibrium.
 * reference is a constant 10-state vector repeated over the horizon.
 * This function does not implement moving-reference generation or VMC.
 */
int chassis_mpc_init(void);
void chassis_mpc_reset(void);
int chassis_mpc_solve(const chassis_control_state_t *state,
                      const chassis_control_state_t *reference,
                      chassis_mpc_result_t *result);
#ifdef __cplusplus
}
#endif
#endif
