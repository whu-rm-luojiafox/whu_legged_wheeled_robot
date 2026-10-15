#include "chassis_mpc.h"
#include "chassis_mpc_config.h"
#include "chassis_tiny_data.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

static bool initialized = false;
extern "C" int chassis_mpc_init(void) {
    if (!initialized) { whu_tiny_init(); initialized = true; }
    chassis_mpc_reset();
    return CHASSIS_MPC_OK;
}
extern "C" void chassis_mpc_reset(void) {
    if (initialized) whu_tiny_reset();
}
extern "C" int chassis_mpc_solve(const chassis_control_state_t *state,
                                const chassis_control_state_t *reference,
                                chassis_mpc_result_t *result) {
    if (!result) return CHASSIS_MPC_BAD_INPUT;
    std::memset(result, 0, sizeof(*result));
    if (!initialized) return CHASSIS_MPC_NOT_INITIALIZED;
    if (!state || !reference) return CHASSIS_MPC_BAD_INPUT;
    for (int i=0; i<10; ++i) {
        if (!std::isfinite(state->x[i]) || !std::isfinite(reference->x[i]))
            return CHASSIS_MPC_BAD_INPUT;
    }
    TinySolver *s = whu_tiny_solver();
    for (int i=0; i<10; ++i) {
        s->work->x(i,0) = state->x[i];
        s->work->Xref.row(i).setConstant(reference->x[i]);
    }
    /* Reset slacks/duals to make baseline tests deterministic (cold start).
     * Warm-start policy is a later, explicitly validated optimization. */
    whu_tiny_reset();
    for (int i=0; i<10; ++i) s->work->x(i,0)=state->x[i];
    int rc = solve(s);
    result->iterations = static_cast<uint32_t>(s->work->iter);
    result->matrix_payload_bytes = whu_tiny_payload_bytes();
    result->primal_residual = static_cast<float>(std::max(s->work->primal_residual_state,
                                                       s->work->primal_residual_input));
    result->dual_residual = static_cast<float>(std::max(s->work->dual_residual_state,
                                                     s->work->dual_residual_input));
    if (!s->solution->u.allFinite() || !s->solution->x.allFinite() ||
        !std::isfinite(result->primal_residual) || !std::isfinite(result->dual_residual))
        return CHASSIS_MPC_NUMERIC_ERROR;
    tinytype violation=0;
    for (int k=0; s->settings->en_input_bound && k<s->work->N-1; ++k)
        for (int i=0; i<4; ++i) {
            violation=std::max(violation, s->solution->u(i,k)-s->work->u_max(i,k));
            violation=std::max(violation, s->work->u_min(i,k)-s->solution->u(i,k));
        }
    result->input_bound_violation=static_cast<float>(violation);
    for(int i=0;i<4;++i) result->candidate.u[i]=static_cast<float>(s->solution->u(i,0));
    if (rc || !s->solution->solved || violation > CHASSIS_MPC_TOL)
        return CHASSIS_MPC_NOT_CONVERGED;
    result->command=result->candidate;
    result->valid=1;
    return CHASSIS_MPC_OK;
}
