#include "main.h"
#include "cmsis_os.h"
#include "shoot.h"
#include "pid.h"
#include "referee.h"
#include "detect_task.h"
#include "math.h"
#include "user_lib.h"
#include "uart_receive.h"

shoot_control_t shoot_control;
const chassis_data_t *shoot_enable;

uint8_t single_shoot_state = 0;         // 单发模式状态位
uint8_t single_shoot_cnt = 0;           // 1表示本次单发动作完成，防止连续单发

uint16_t shoot_state;
uint8_t process_state = 0;
int8_t shoot_mode;

fp32 ShootTime = 0.0f;
fp32 shoot_time = 0.0f;
fp32 shoot_speed = 0.0f;

extern robot_status_t robot_state;
extern power_heat_data_t power_heat_data;
float real_time_heat = 0;


/* ========================= 防堵转 / 退弹参数 =========================
 * 说明：
 * 1) 用 HAL_GetTick() 计时，所以不依赖任务周期。
 * 2) 判据：当前处于拨弹工作状态，且 PID 输出电流较大，但电机转速持续很低。
 * 3) 动作：反转固定时长 -> 恢复正转。
 * 4) 参数需要你上车后再微调。
 */
#define TRIGGER_JAM_MIN_CMD_CURRENT      4000      // 判定堵转所需的最小电流输出
#define TRIGGER_JAM_SPEED_RPM_THRES      400.0f     // 低速阈值，低于此值持续一段时间认为堵转
#define TRIGGER_JAM_DETECT_MS            300U      // 堵转持续判定时间
#define TRIGGER_REVERSE_SPEED_RPM        2200.0f   // 退弹反转速度
#define TRIGGER_REVERSE_MS               180U      // 退弹反转时长
#define TRIGGER_REVERSE_COOLDOWN_MS      100U      // 退弹完成后冷却时间，避免立刻再次误判

typedef enum
{
    TRIGGER_ANTI_JAM_IDLE = 0,
    TRIGGER_ANTI_JAM_REVERSE,
} trigger_anti_jam_state_e;

static trigger_anti_jam_state_e trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
static uint8_t trigger_jam_checking = 0;
static uint32_t trigger_jam_start_tick = 0;
static uint32_t trigger_reverse_start_tick = 0;
static uint32_t trigger_reverse_cooldown_tick = 0;

void real_heat_calc(void)
{
    if(shoot_control.shoot_heat_value != shoot_control.last_shoot_heat_value)	//认为更新了
	{
		real_time_heat = shoot_control.shoot_heat_value;
	}
	else 
	{
		real_time_heat = real_time_heat + shoot_speed*0.002f*10 - shoot_control.shoot_cooling_rate*0.002f;		//我的任务间隔是0.002s
		if(real_time_heat <= 0)		
        {real_time_heat = 0;}
	}
	//以上保证更新没问题
}

static void trigger_anti_jam_reset(void)
{
    trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
    trigger_jam_checking = 0;
    trigger_jam_start_tick = 0;
    trigger_reverse_start_tick = 0;
    trigger_reverse_cooldown_tick = 0;
}

static void trigger_anti_jam_start_reverse(void)
{
    trigger_anti_jam_state = TRIGGER_ANTI_JAM_REVERSE;
    trigger_reverse_start_tick = HAL_GetTick();
    trigger_jam_checking = 0;
    trigger_jam_start_tick = 0;
}

static uint8_t trigger_need_anti_jam(fp32 actual_speed_rpm, int16_t given_current, uint8_t motor_should_run)
{
    uint32_t now = HAL_GetTick();

    if (!motor_should_run)
    {
        trigger_jam_checking = 0;
        trigger_jam_start_tick = 0;
        return 0;
    }

    if (trigger_anti_jam_state != TRIGGER_ANTI_JAM_IDLE)
    {
        return 0;
    }

    if (now < trigger_reverse_cooldown_tick)
    {
        return 0;
    }

    if ((fabsf(actual_speed_rpm) < TRIGGER_JAM_SPEED_RPM_THRES) &&
        (abs(given_current) > TRIGGER_JAM_MIN_CMD_CURRENT))
    {
        if (!trigger_jam_checking)
        {
            trigger_jam_checking = 1;
            trigger_jam_start_tick = now;
        }
        else if ((now - trigger_jam_start_tick) >= TRIGGER_JAM_DETECT_MS)
        {
            return 1;
        }
    }
    else
    {
        trigger_jam_checking = 0;
        trigger_jam_start_tick = 0;
    }

    return 0;
}

/* 返回1表示本周期已被“退弹逻辑”接管，外部不应再覆盖 given_current */
static uint8_t trigger_anti_jam_control(void)
{
    uint32_t now = HAL_GetTick();

    if (trigger_anti_jam_state == TRIGGER_ANTI_JAM_REVERSE)
    {
        shoot_control.speed_set = -TRIGGER_REVERSE_SPEED_RPM;
        PID_calc(&shoot_control.trigger_speed_mode_speed_pid,
                 shoot_control.shoot_motor_measure->speed_rpm,
                 shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);

        if ((now - trigger_reverse_start_tick) >= TRIGGER_REVERSE_MS)
        {
            trigger_anti_jam_state = TRIGGER_ANTI_JAM_IDLE;
            trigger_reverse_cooldown_tick = now + TRIGGER_REVERSE_COOLDOWN_MS;
            trigger_jam_checking = 0;
            trigger_jam_start_tick = 0;
        }
        return 1;
    }

    return 0;
}

/* ========================= 原逻辑 ========================= */

// void shoot_speed_calc(void)
// {
//     float a = (float)(robot_state.shooter_barrel_cooling_value);
//     float m = (float)(robot_state.shooter_barrel_heat_limit - power_heat_data.shooter_17mm_1_barrel_heat);
//     float d = 10.0f;

//     if (shoot_time == 0)
//     {
//         ShootTime = (m + 2 * a) * 5;
//         ShootTime = fp32_constrain(ShootTime, 0, 5000);

//         if (m < 100)
//         {
//             shoot_speed = (10 * m - a - 5 * d) / (d * (ShootTime / 100.0f)) + a / d;
//         }
//         else
//         {
//             shoot_speed = (10 * m - a - 20 * d) / (d * (ShootTime / 100.0f)) + a / d;
//         }
//     }
//     else if (0 < shoot_time && shoot_time < ShootTime)
//     {
//     }
//     else
//     {
//         shoot_speed = (a / d);
//     }

//     if (shoot_time < ShootTime)
//     {
//         shoot_time++;
//     }

//     shoot_speed = fp32_constrain(shoot_speed, 0, 36);
// }
void shoot_speed_calc(void)
{
    shoot_speed = 32;
	if(real_time_heat >= (float)shoot_control.shoot_heat_limit /2.0f)		//热量值达到最大的1/3开始减速
	{
		shoot_speed = (float)(shoot_control.shoot_heat_limit - real_time_heat)/(float)shoot_control.shoot_heat_limit*20.0f;
	}
	if((float)shoot_control.shoot_heat_limit - real_time_heat <= 18.0f)	//直接停
	{
		shoot_speed = shoot_control.shoot_cooling_rate/10;
	}
	else 
	{
		shoot_speed = 32.0f;		//维持理想弹频
	}
}

void ballet_feq_calc(fp32 fequence)
{
    shoot_control.speed_set = (fequence / 8.0f) * 60 * 36;
}

uint16_t shoot_single_control(void)
{
    if (single_shoot_state == 0)
    {
        shoot_control.set_angle = shoot_control.angle + PI_TEN;
        single_shoot_state = 1;
        return 0;
    }
    else if (single_shoot_state == 1)
    {
        if (fabs(shoot_control.set_angle - shoot_control.angle) > 2.0f)
        {
            /* 位置环 PID */
            PID_calc(&shoot_control.trigger_motor_angle_pid,
                     shoot_control.angle,
                     shoot_control.set_angle);

            PID_calc(&shoot_control.trigger_position_mode_speed_pid,
                     shoot_control.shoot_motor_measure->speed_rpm,
                     shoot_control.trigger_motor_angle_pid.out);

            shoot_control.given_current = (int16_t)(shoot_control.trigger_position_mode_speed_pid.out);

            /* 单发过程中也允许触发退弹 */
            if (trigger_need_anti_jam(shoot_control.shoot_motor_measure->speed_rpm,
                                      shoot_control.given_current,
                                      1))
            {
                trigger_anti_jam_start_reverse();
            }

            if (trigger_anti_jam_control())
            {
                return 0;
            }

            return 0;
        }
        else
        {
            single_shoot_state = 0;
            single_shoot_cnt = 1;
            return 1;
        }
    }
    return 0;
}

static void shoot_feedback_update(void)
{
    shoot_control.shoot_heat_limit = robot_state.shooter_barrel_heat_limit;
    shoot_control.shoot_cooling_rate = robot_state.shooter_barrel_cooling_value;
    shoot_control.last_shoot_heat_value = shoot_control.shoot_heat_value;
    shoot_control.shoot_heat_value = power_heat_data.shooter_17mm_1_barrel_heat;
    shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm;
    shoot_control.angle = shoot_control.shoot_motor_measure->angle;
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = (shoot_control.shoot_control_data->shoot_mode_rc >> 4) & 0x01;
    shoot_mode = shoot_control.shoot_control_data->shoot_mode_rc & 0x0F;
}

void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};

    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.shoot_control_data = get_Uart_Chassisdata_point();

    PID_init(&shoot_control.trigger_position_mode_speed_pid,
             PID_POSITION,
             Trigger_speed_pid,
             TRIGGER_BULLET_PID_MAX_OUT,
             TRIGGER_BULLET_PID_MAX_IOUT);

    PID_init(&shoot_control.trigger_speed_mode_speed_pid,
             PID_POSITION,
             Trigger_speed_pid,
             TRIGGER_BULLET_PID_MAX_OUT,
             TRIGGER_BULLET_PID_MAX_IOUT);

    PID_init(&shoot_control.trigger_motor_angle_pid,
             PID_POSITION,
             Trigger_angle_pid,
             TRIGGER_ANGLE_PID_MAX_OUT,
             TRIGGER_ANGLE_PID_MAX_IOUT);

    shoot_control.angle = shoot_control.shoot_motor_measure->angle;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed_set = 0.0f;
    shoot_control.speed = 0.0f;
    shoot_control.given_current = 0;
    shoot_state = SHOOT_FINISH;

    trigger_anti_jam_reset();
}

static void shoot_set_mode(void)
{
    if (shoot_mode == SHOOT_SINGLE)
    {
        shoot_control.shoot_mode = SHOOT_SINGLE;
    }
    else if (shoot_mode == SHOOT_CONTINUE)
    {
        shoot_control.shoot_mode = SHOOT_CONTINUE;
    }
    else if (shoot_mode == SHOOT_STOP || shoot_mode == SHOOT_READY_FRIC)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
}

void shoot_control_set(void)
{
    if (shoot_control.shoot_mode == SHOOT_SINGLE && shoot_state == SHOOT_FINISH && single_shoot_cnt == 0)
    {
        shoot_state = SHOOT_START_SINGLE;
    }
    else if (shoot_state == SHOOT_START_SINGLE)
    {
        if((shoot_control.shoot_heat_limit - real_time_heat) > 18.0f)
        {
            process_state = shoot_single_control();
            if (process_state == 1)
            {
                process_state = 0;
                shoot_control.given_current = 0;
                shoot_state = SHOOT_FINISH;
            }
        }
        else{
                shoot_state = SHOOT_FINISH;            
        }
    }

    if (shoot_control.shoot_mode == SHOOT_CONTINUE && shoot_state == SHOOT_FINISH)
    {
        shoot_state = SHOOT_START_CONTINUE;
    }
    else if (shoot_state == SHOOT_START_CONTINUE)
    {

        if (trigger_anti_jam_control())
        {
            return;
        }
        shoot_speed_calc();
        ballet_feq_calc(shoot_speed);
        single_shoot_cnt = 0;

        PID_calc(&shoot_control.trigger_speed_mode_speed_pid,
                 shoot_control.shoot_motor_measure->speed_rpm,
                 shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);

        /* 连发防堵转判定 */
        if (trigger_need_anti_jam(shoot_control.shoot_motor_measure->speed_rpm,
                                  shoot_control.given_current,
                                  1))
        {
            trigger_anti_jam_start_reverse();
            trigger_anti_jam_control();
            return;
        }

        /* 若正在退弹，本周期由退弹逻辑接管 */
        if (trigger_anti_jam_control())
        {
            return;
        }

        if (shoot_control.shoot_mode != SHOOT_CONTINUE)
        {
            shoot_state = SHOOT_FINISH;
            shoot_time = 0;
            shoot_speed = 0;
            trigger_anti_jam_reset();
        }
    }

    if ((shoot_control.shoot_mode != SHOOT_CONTINUE) &&
        (shoot_control.shoot_mode != SHOOT_SINGLE) &&
        shoot_state == SHOOT_FINISH)
    {
        /* 停拨弹盘 */
        shoot_control.speed_set = 0;
        PID_calc(&shoot_control.trigger_speed_mode_speed_pid,
                 shoot_control.shoot_motor_measure->speed_rpm,
                 shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);
        single_shoot_cnt = 0;
        trigger_anti_jam_reset();
    }

    if (shoot_control.shoot_mode == SHOOT_SINGLE &&
        shoot_state == SHOOT_FINISH &&
        single_shoot_cnt == 1)
    {
        shoot_control.speed_set = 0;
        PID_calc(&shoot_control.trigger_speed_mode_speed_pid,
                 shoot_control.shoot_motor_measure->speed_rpm,
                 shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);
    }
}

shoot_control_t *shoot(void)
{
    real_heat_calc();
    shoot_feedback_update();
    shoot_set_mode();
    shoot_control_set();
    return &shoot_control;
}