/* Opt-in standalone probe. NOT registered in freertos.c or the Keil project.
 * Call init once after the clock and C++ runtime are ready, then step from
 * one task. Never call from an ISR; no motor/CAN functions are referenced.
 */
#include "board_probe.h"
#include "chassis_mpc_config.h"
#include <stdint.h>
#define WHU_DEMCR (*(volatile uint32_t *)0xE000EDFCu)
#define WHU_DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define WHU_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
volatile whu_mpc_probe_record_t whu_mpc_probe_record;
static uint32_t probe_clock;
int whu_mpc_board_probe_init(uint32_t core_clock_hz) {
    if(!core_clock_hz) return CHASSIS_MPC_BAD_INPUT;
    probe_clock=core_clock_hz;
    WHU_DEMCR |= (1u<<24);
    WHU_DWT_CYCCNT=0;
    WHU_DWT_CTRL |= 1u;
    return chassis_mpc_init();
}
void whu_mpc_board_probe_step(void) {
    chassis_control_state_t x={{0}}, ref={{0}};
    chassis_mpc_result_t r;
    uint32_t t0,t1;
    int status;
    if(!probe_clock) return;
    x.x[CHASSIS_STATE_PITCH]=0.01f; /* fixed test input, no live-state access */
    t0=WHU_DWT_CYCCNT;
    status=chassis_mpc_solve(&x,&ref,&r);
    t1=WHU_DWT_CYCCNT;
    whu_mpc_probe_record.status=(uint32_t)status;
    whu_mpc_probe_record.cycles=t1-t0;
    whu_mpc_probe_record.elapsed_us=(uint32_t)(((uint64_t)(t1-t0)*1000000u)/probe_clock);
    whu_mpc_probe_record.core_clock_hz=probe_clock;
    whu_mpc_probe_record.result=r;
    ++whu_mpc_probe_record.sequence; /* read a stable record while task paused */
}
