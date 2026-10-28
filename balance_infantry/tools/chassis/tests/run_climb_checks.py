"""Exercise the production climb state machine and VMC combiner without hardware.

Only the RTOS clock and application storage are stubbed. The torque/mapping
types, combiner function and limit macros are extracted from current sources.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def typedef(source, name):
    end = source.index('} ' + name + ';') + len('} ' + name + ';')
    start = source.rfind('typedef struct', 0, end)
    return source[start:end]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc', type=Path)
    args = parser.parse_args()
    compiler = args.cc or shutil.which('gcc')
    if not compiler:
        compiler = next(Path('D:/clion-toolchain').glob('**/mingw64/bin/gcc.exe'), None)
    if not compiler:
        raise SystemExit('Pass a native GCC compiler with --cc')
    compiler = Path(compiler)
    out = ROOT / 'build/chassis-climb-checks'
    out.mkdir(parents=True, exist_ok=True)
    task_source = (ROOT / 'src/chassis/application/chassis_task.c').read_text(encoding='utf-8')
    task_header = (ROOT / 'src/chassis/application/chassis_task.h').read_text(encoding='utf-8')
    torque_type = typedef(task_header, 'torque_info_t')
    mapping_type = typedef(task_header, 'mapping_info_t')
    pid_type = typedef((ROOT / 'src/common/components/controller/pid.h').read_text(encoding='utf-8'), 'pid_type_def')
    stub = r'''
#ifndef CLIMB_CHECK_TASK_H
#define CLIMB_CHECK_TASK_H
#include <stdint.h>
#include <stddef.h>
#include "chassis_fwd.h"
#include "chassis_leg_math.h"
#include "chassis_balance.h"
typedef float fp32;
enum { ON_GROUND = 0, OFF_GROUND = 1 };
enum { ENABLE_CHASSIS = 0, DISABLE_CHASSIS = 1 };
enum { NORMAL_MOVING_MODE = 1, JUMPING_MODE = 2 };
enum { NO_FORCE = 0, BALANCING_READY = 3 };
typedef struct { int16_t given_current; } motor_measure_t;
typedef struct { motor_measure_t *motor_measure; float speed, torque_out; } motor_t;
typedef struct { uint8_t climb_flag, recovery_flag; } chassis_data_t;
@TORQUE_TYPE@
@MAPPING_TYPE@
@PID_TYPE@
struct chassis_move {
    chassis_data_t *chassis_data_;
    struct { int chassis_mode, chassis_balancing_mode, sport_mode; } mode;
    struct { int suspend_flag_L, suspend_flag_R; } flag_info;
    struct {
        chassis_leg_posture_t chassis_posture_L, chassis_posture_R;
        float ideal_high, foot_speed_set, pitch_angle, roll_angle, pitch_gyro;
        float foot_distance_set, foot_distance_K;
        uint8_t position_lock_flag, position_lock_state;
    } chassis_posture_info;
    motor_t foot_motor_L, foot_motor_R;
    motor_t joint_motor_1, joint_motor_2, joint_motor_3, joint_motor_4;
    torque_info_t torque_info;
    mapping_info_t mapping_info;
    pid_type_def leg_L_length_pid, leg_R_length_pid;
};
extern chassis_lqr_feedback_t chassis_lqr_feedback;
extern chassis_feedforward_t chassis_feedforward;
#endif
'''.replace('@TORQUE_TYPE@', torque_type).replace('@MAPPING_TYPE@', mapping_type).replace('@PID_TYPE@', pid_type)
    (out / 'chassis_task.h').write_text(stub, encoding='utf-8')
    (out / 'uart_receive.h').write_text('/* UART is unused by this module. */\n', encoding='utf-8')
    (out / 'task.h').write_text(
        '#include <stdint.h>\n#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))\n'
        'uint32_t xTaskGetTickCount(void);\n', encoding='utf-8')
    start = task_source.index('void Chassis_Torque_Combine(chassis_move_t *bl_ctrl)\n{')
    end = task_source.index('\nvoid Motor_CMD_Send', start)
    macros = task_source[task_source.index('#define LimitMax'):task_source.index('#define MODEL_PYH_WHEEL_DIR')]
    foot_limit = re.search(r'^#define MAX_FOOT_OUTPUT\s+.*$', task_header, re.M).group(0)
    protocol = (ROOT / 'src/chassis/protocols/motor_protocol.h').read_text(encoding='utf-8')
    joint_limit = re.search(r'^#define JOINT_TORQUE_LIMIT\s+.*$', protocol, re.M).group(0)
    (out / 'combine_under_test.c').write_text(
        macros + '\n' + foot_limit + '\n' + joint_limit + '\n' + task_source[start:end], encoding='utf-8')
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    executable = out / ('climb_checks.exe' if os.name == 'nt' else 'climb_checks')
    command = [str(compiler), '-std=c11', '-Wall', '-Wextra', '-Werror', '-O0', '-g',
               '-I' + str(out), '-I' + str(ROOT / 'src/chassis/module'),
               '-I' + str(ROOT / 'src/chassis/components/algorithm'),
               '-I' + str(ROOT / 'src/chassis/components/controller'),
               str(Path(__file__).with_name('climb_checks.c')),
               str(ROOT / 'src/chassis/components/algorithm/chassis_leg_math.c'),
               str(ROOT / 'src/chassis/module/chassis_motion.c'),
               '-lm', '-o', str(executable)]
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    subprocess.run([str(executable)], cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
