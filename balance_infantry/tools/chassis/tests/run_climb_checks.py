"""Run climb FSM and final VMC regression checks with a host RTOS/storage fixture."""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def typedef(source, name):
    end = source.index('} ' + name + ';') + len('} ' + name + ';')
    return source[source.rfind('typedef struct', 0, end):end]


def function(source, name):
    match = re.search(r'(?m)^(?:static\s+)?(?:void|float|uint8_t)\s+' + name + r'\([^;]*?\)\s*\{', source)
    start = match.start()
    lex = re.compile(r'/\*[\s\S]*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]')
    depth = 0
    for token in lex.finditer(source, source.index('{', start)):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                return source[start:token.end()]
    raise ValueError(name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc', type=Path)
    args = parser.parse_args()
    compiler = args.cc or shutil.which('gcc') or next(Path('D:/clion-toolchain').glob('**/mingw64/bin/gcc.exe'), None)
    if not compiler:
        raise SystemExit('Pass a native GCC compiler with --cc')
    compiler = Path(compiler)
    out = ROOT / 'build/chassis-climb-checks'
    out.mkdir(parents=True, exist_ok=True)
    types = (ROOT / 'src/chassis/module/chassis_types.h').read_text(encoding='utf-8')
    control = (ROOT / 'src/chassis/module/chassis_control.c').read_text(encoding='utf-8')
    config = (ROOT / 'src/chassis/application/chassis_config.h').read_text(encoding='utf-8')
    pid = (ROOT / 'src/common/components/controller/pid.h').read_text(encoding='utf-8')
    stub = r'''
#ifndef CLIMB_CHECK_TYPES_H
#define CLIMB_CHECK_TYPES_H
#include <stdint.h>
#include <stddef.h>
#include "chassis_fwd.h"
#include "chassis_leg_math.h"
#include "chassis_balance.h"
typedef float fp32;
enum { ON_GROUND=0, OFF_GROUND=1, ENABLE_CHASSIS=0, DISABLE_CHASSIS=1 };
enum { NORMAL_MOVING_MODE=1, JUMPING_MODE=2, NO_FORCE=0, BALANCING_READY=3 };
typedef struct { int16_t given_current; } motor_measure_t;
typedef struct { motor_measure_t *motor_measure; float speed, torque_out; } motor_t;
typedef struct { uint8_t climb_flag, recovery_flag; } chassis_data_t;
@POSTURE@
@TORQUE@
@MAPPING@
@PID@
struct chassis_move {
    chassis_data_t *chassis_data_;
    struct { int chassis_mode, chassis_balancing_mode, sport_mode; } mode;
    struct { int suspend_flag_L, suspend_flag_R; } flag_info;
    chassis_posture_info_t chassis_posture_info;
    motor_t foot_motor_L, foot_motor_R;
    motor_t joint_motor_1, joint_motor_2, joint_motor_3, joint_motor_4;
    torque_info_t torque_info;
    mapping_info_t mapping_info;
    pid_type_def leg_L_length_pid, leg_R_length_pid;
};
static inline float fp32_constrain(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}
extern chassis_lqr_feedback_t chassis_lqr_feedback;
extern chassis_feedforward_t chassis_feedforward;
#endif
'''
    for marker, source, name in [('@POSTURE@', types, 'chassis_posture_info_t'), ('@TORQUE@', types, 'torque_info_t'), ('@MAPPING@', types, 'mapping_info_t'), ('@PID@', pid, 'pid_type_def')]:
        stub = stub.replace(marker, typedef(source, name))
    (out / 'chassis_types.h').write_text(stub, encoding='utf-8')
    (out / 'FreeRTOS.h').write_text('/* Host clock fixture. */\n', encoding='utf-8')
    (out / 'task.h').write_text('#include <stdint.h>\n#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))\nuint32_t xTaskGetTickCount(void);\n', encoding='utf-8')
    (out / 'climb_under_test.c').write_text((ROOT / 'src/chassis/module/chassis_climb.c').read_text(encoding='utf-8'), encoding='utf-8')
    protocol = (ROOT / 'src/chassis/protocols/motor_protocol.h').read_text(encoding='utf-8')
    macros = '\n'.join(re.search(r'^#define ' + name + r'\s+.*$', source, re.M).group() for source, name in [(control, 'FORWARD_TORQUE'), (config, 'MAX_FOOT_OUTPUT'), (protocol, 'JOINT_TORQUE_LIMIT')])
    (out / 'combine_under_test.c').write_text(macros + '\n' + function(control, 'chassisControlMap'), encoding='utf-8')
    motion = (ROOT / 'src/chassis/module/chassis_motion.c').read_text(encoding='utf-8')
    (out / 'motion_under_test.c').write_text('#include "chassis_motion.h"\n#include <math.h>\n' + '\n'.join(function(motion, name) for name in ['chassis_position_speed_request', 'release', 'chassis_position_step']), encoding='utf-8')
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    executable = out / ('climb_checks.exe' if os.name == 'nt' else 'climb_checks')
    command = [str(compiler), '-std=c11', '-Wall', '-Wextra', '-Werror', '-O0', '-g',
               '-I' + str(out), '-I' + str(ROOT / 'src/chassis/module'),
               '-I' + str(ROOT / 'src/chassis/components/algorithm'),
               '-I' + str(ROOT / 'src/chassis/components/controller'),
               str(Path(__file__).with_name('climb_checks.c')),
               str(ROOT / 'src/chassis/components/algorithm/chassis_leg_math.c'),
               str(out / 'motion_under_test.c'), '-lm', '-o', str(executable)]
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    subprocess.run([str(executable)], cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
