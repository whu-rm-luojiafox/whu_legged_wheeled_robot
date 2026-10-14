/* Standalone measurement image; no RTOS, peripherals or motor outputs.
 * Inspect whu_mpc_probe_record in the debugger. This is not a control loop.
 */
#include "stm32f4xx.h"
#include "board_probe.h"

int main(void)
{
    SystemCoreClockUpdate();
    if (whu_mpc_board_probe_init(SystemCoreClock) == CHASSIS_MPC_OK) {
        whu_mpc_board_probe_step();
    }
    for (;;) {
        __WFI();
    }
}
