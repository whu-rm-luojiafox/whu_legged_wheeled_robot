#ifndef CHASSIS_LQR_GAIN_H
#define CHASSIS_LQR_GAIN_H

/* Existing firmware diagnostics observe this matrix. The calculation itself
 * accepts an explicit output and has no application or hardware dependency. */
extern float LQR[4][10];
void chassis_lqr_gain_calculate(float left_m, float right_m, float gain[4][10]);

#endif
