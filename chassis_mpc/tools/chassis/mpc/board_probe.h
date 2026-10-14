#ifndef WHU_MPC_BOARD_PROBE_H
#define WHU_MPC_BOARD_PROBE_H
#include "chassis_mpc.h"
typedef struct {
    uint32_t sequence, status, cycles, elapsed_us, core_clock_hz;
    chassis_mpc_result_t result;
} whu_mpc_probe_record_t;
extern volatile whu_mpc_probe_record_t whu_mpc_probe_record;
#ifdef __cplusplus
extern "C" {
#endif
int whu_mpc_board_probe_init(uint32_t core_clock_hz);
void whu_mpc_board_probe_step(void);
#ifdef __cplusplus
}
#endif
#endif
