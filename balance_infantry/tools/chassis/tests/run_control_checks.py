"""Compile the production chassis modules against host hardware/RTOS fixtures."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc', type=Path)
    args = parser.parse_args()
    compiler = args.cc or shutil.which('gcc') or next(Path('D:/clion-toolchain').glob('**/mingw64/bin/gcc.exe'), None)
    if not compiler:
        raise SystemExit('Pass a native GCC compiler with --cc')
    compiler = Path(compiler)
    out = ROOT / 'build/chassis-control-checks'
    out.mkdir(parents=True, exist_ok=True)
    def emit(name, text):
        (out / name).write_text(text, encoding='utf-8')
    # Keep actual state fields and protocol/PID definitions; RC hardware is unused.
    types = (ROOT / 'src/chassis/module/chassis_types.h').read_text(encoding='utf-8')
    body = types[types.index('typedef enum'):types.rfind('#endif')]
    emit('chassis_types.h', r'''
#ifndef CONTROL_HOST_TYPES_H
#define CONTROL_HOST_TYPES_H
#include "struct_typedef.h"
#include "chassis_fwd.h"
#include "can_service.h"
#include "gim_cha_protocol.h"
#include "super_power_protocol.h"
#include "pid.h"
#include "chassis_config.h"
#include "chassis_leg_math.h"
#include "gimbal_motor.h"
typedef struct { int unused; } RC_ctrl_t;
''' + body + '\n#endif\n')
    emit('main.h', '#include <string.h>\n')
    emit('FreeRTOS.h', '#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))\n')
    emit('task.h', '#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\n')
    emit('stm32f407xx.h', '#include <stddef.h>\nvoid *velocity_test_alloc(size_t size);\n')
    emit('arm_math.h', (HERE / 'host_matrix.h').read_text(encoding='utf-8') + '\nfloat arm_sin_f32(float);\nfloat arm_cos_f32(float);\n')
    emit('user_lib.h', r'''
#ifndef CONTROL_HOST_USER_LIB_H
#define CONTROL_HOST_USER_LIB_H
#include <math.h>
#include "struct_typedef.h"
typedef struct { float unused; } first_order_filter_type_t;
static inline float fp32_constrain(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static inline float loop_fp32_constrain(float x,float lo,float hi){while(x>hi)x-=hi-lo;while(x<lo)x+=hi-lo;return x;}
#define rad_format(x) loop_fp32_constrain((x),-3.14159265358979f,3.14159265358979f)
static inline float theta_format(float x){return loop_fp32_constrain(x,-180,180);}
#endif
''')
    emit('runtime_host.c', r'''
#include "chassis_types.h"
#include <math.h>
uint32_t host_tick;
unsigned host_enable_count;
motor_measure_t host_wheels[2],host_yaw;
static dm_motor_measure_t joints[4]={{0}};
float host_accel[3]={0,0,9.81f};
static float angles[3],gyro[3];
uint32_t xTaskGetTickCount(void){return host_tick;}
void CAN_clear_dm_error(void){}
void CAN_dm_enable(uint16_t id){(void)id;++host_enable_count;}
motor_measure_t *get_chassis_motor_measure_point(uint8_t i){return &host_wheels[i];}
dm_motor_measure_t *get_DM_motor_measure_point(uint8_t i){joints[0].pos=-2.70f;joints[1].pos=3.08f;joints[2].pos=3.08f;joints[3].pos=-2.70f;return &joints[i];}
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void){return &host_yaw;}
const float *get_INS_angle_point(void){return angles;}
const float *get_gyro_data_point(void){return gyro;}
const float *get_accel_data_point(void){return host_accel;}
float arm_sin_f32(float x){return sinf(x);}
float arm_cos_f32(float x){return cosf(x);}
''')
    modules = ['chassis_observer','chassis_mode','chassis_motion','chassis_control','chassis_contact','chassis_climb','chassis_recovery','chassis_power','chassis_follow']
    sources = []
    for name in modules:
        source = ROOT / 'src/chassis/module' / (name + '.c')
        emit(source.name, source.read_text(encoding='utf-8'))
        sources.append(str(out / source.name))
    for relative in ['src/chassis/service/kalman_service.c','src/chassis/components/algorithm/kalman_filter.c','src/chassis/components/algorithm/chassis_leg_math.c','src/chassis/components/algorithm/LQR.c','src/chassis/components/controller/chassis_balance.c','src/common/components/controller/pid.c']:
        source = ROOT / relative
        emit(source.name, source.read_text(encoding='utf-8'))
        sources.append(str(out / source.name))
    includes = ['src/chassis/module','src/chassis/application','src/chassis/service','src/chassis/service/CanService','src/chassis/protocols','src/common/include','src/common/protocols','src/common/components/controller','src/chassis/components/devices','src/chassis/components/algorithm','src/chassis/components/controller']
    executable = out / ('control_checks.exe' if os.name == 'nt' else 'control_checks')
    # The unchanged LQR coefficient table uses a legal flat 2-D initializer.
    command = [str(compiler),'-std=c11','-Wall','-Wextra','-Werror','-Wno-missing-braces','-O0','-g','-include','string.h','-Duser_malloc=velocity_test_alloc','-I'+str(out)]
    command += ['-I'+str(ROOT / path) for path in includes]
    command += sources + [str(HERE / 'host_matrix.c'),str(out / 'runtime_host.c'),str(HERE / 'control_checks.c'),'-lm','-o',str(executable)]
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH','')
    subprocess.run(command,cwd=ROOT,env=env,check=True)
    subprocess.run([str(executable)],cwd=ROOT,env=env,check=True)


if __name__ == '__main__':
    main()
