# include "chassis_contact.h"
# include "LQR.h"
# include "task.h"
//物理属性
const fp32 g = 9.81f;
const fp32 m_w = 0.51f;
const fp32 stablize_foot_speed_threshold = 1.2f, stablize_yaw_speed_threshold = 1.5f;

void calculate_wheel_vertical_acceleration(chassis_move_t * detect)
{
	detect->chassis_posture_info.chassis_accel = detect->chassis_posture_info.z_accel-g*cos(detect->chassis_posture_info.pitch_angle);
	
	detect->chassis_posture_info.foot_accel_L=
	+detect->chassis_posture_info.chassis_accel
	-detect->chassis_posture_info.chassis_posture_L.leg_ddlength*cos(detect->chassis_posture_info.chassis_posture_L.leg_angle);
	// +2*detect->chassis_posture_info.chassis_posture_L.leg_dlength_jacobian*detect->chassis_posture_info.chassis_posture_L.leg_gyro*sin(detect->chassis_posture_info.chassis_posture_L.leg_angle)
	// +detect->chassis_posture_info.chassis_posture_L.leg_length*detect->chassis_posture_info.chassis_posture_L.leg_accel*sin(detect->chassis_posture_info.chassis_posture_L.leg_angle)
	// +detect->chassis_posture_info.chassis_posture_L.leg_length*detect->chassis_posture_info.chassis_posture_L.leg_gyro*detect->chassis_posture_info.chassis_posture_L.leg_gyro*cos(detect->chassis_posture_info.chassis_posture_L.leg_angle);
	
	detect->chassis_posture_info.foot_accel_R=
	+detect->chassis_posture_info.chassis_accel
	-detect->chassis_posture_info.chassis_posture_R.leg_ddlength*cos(detect->chassis_posture_info.chassis_posture_R.leg_angle);
	// +2*detect->chassis_posture_info.chassis_posture_R.leg_dlength_jacobian*detect->chassis_posture_info.chassis_posture_R.leg_gyro*sin(detect->chassis_posture_info.chassis_posture_R.leg_angle)
	// +detect->chassis_posture_info.chassis_posture_R.leg_length*detect->chassis_posture_info.chassis_posture_R.leg_accel*sin(detect->chassis_posture_info.chassis_posture_R.leg_angle)
	// +detect->chassis_posture_info.chassis_posture_R.leg_length*detect->chassis_posture_info.chassis_posture_R.leg_gyro*detect->chassis_posture_info.chassis_posture_R.leg_gyro*cos(detect->chassis_posture_info.chassis_posture_R.leg_angle);
	
}

//F_N = P + M_w*a + M_w*g 
void Supportive_Force_Calculate(chassis_move_t * detect)
{
	//计算腿部支持力
	detect->torque_info.forque_L=
	detect->torque_info.joint_vertical_torque_L*cos(detect->chassis_posture_info.chassis_posture_L.leg_angle)
	-detect->torque_info.joint_horizontal_torque_L*sin(detect->chassis_posture_info.chassis_posture_L.leg_angle)/detect->chassis_posture_info.chassis_posture_L.leg_length;
	detect->torque_info.forque_R=
	detect->torque_info.joint_vertical_torque_R*cos(detect->chassis_posture_info.chassis_posture_R.leg_angle)	
	-detect->torque_info.joint_horizontal_torque_R*sin(detect->chassis_posture_info.chassis_posture_R.leg_angle)/detect->chassis_posture_info.chassis_posture_R.leg_length;
	fp32 temp_L = fp32_constrain(detect->torque_info.forque_L, -100.0f, 100.0f);
	fp32 temp_R = fp32_constrain(detect->torque_info.forque_R, -100.0f, 100.0f);
	//计算加速度环节
	calculate_wheel_vertical_acceleration(detect);
	//支持力计算环节
	detect->torque_info.supportive_force_L=temp_L+m_w*g+m_w*detect->chassis_posture_info.foot_accel_L;
	detect->torque_info.supportive_force_R=temp_R+m_w*g+m_w*detect->chassis_posture_info.foot_accel_R;
	detect->torque_info.supportive_force_L = 0.7f*detect->torque_info.supportive_force_L + 0.3f * detect->torque_info.last_supportive_force_L;
	detect->torque_info.supportive_force_R = 0.7f*detect->torque_info.supportive_force_R + 0.3f * detect->torque_info.last_supportive_force_R;
	detect->torque_info.last_supportive_force_L=detect->torque_info.supportive_force_L;
	detect->torque_info.last_supportive_force_R=detect->torque_info.supportive_force_R;

}

TickType_t offTickL = 0, onTickL = 0;
TickType_t offTickR = 0, onTickR = 0;

/*--------------------------- Off Ground Detect --------------------------*/
void Chassis_Status_Detect(chassis_move_t *detect)
{
	Supportive_Force_Calculate(detect);

	TickType_t now = xTaskGetTickCount();
	if (detect->mode.jumping_stage == EXTENDING_LEGS)
	{
		detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
		offTickL = onTickL = offTickR = onTickR = 0;	
	}
	else
	{
		if (detect->torque_info.supportive_force_L <= LOWER_SUPPORT_FORCE  )
			{
				if (offTickL == 0) offTickL = now;
				if (now - offTickL >= pdMS_TO_TICKS(20))
				detect->flag_info.suspend_flag_L = OFF_GROUND;			
			}
		else offTickL = 0;
		if (detect->torque_info.supportive_force_L > LOWER_SUPPORT_FORCE + 5.0f)
		{
			if (onTickL == 0) onTickL = now;
			if (now - onTickL >= pdMS_TO_TICKS(30))
				detect->flag_info.suspend_flag_L = ON_GROUND;
		}
		else onTickL = 0;

		if (detect->torque_info.supportive_force_R <= LOWER_SUPPORT_FORCE )
		{
			if (offTickR == 0) offTickR = now;
			if (now - offTickR >= pdMS_TO_TICKS(20))
				detect->flag_info.suspend_flag_R = OFF_GROUND;
		}
		else offTickR = 0;
		if (detect->torque_info.supportive_force_R > LOWER_SUPPORT_FORCE + 5.0f)
		{
			if (onTickR == 0) onTickR = now;
			if (now - onTickR >= pdMS_TO_TICKS(30))
				detect->flag_info.suspend_flag_R = ON_GROUND;
		}
		else onTickR = 0;
	}
}		


void Chassis_Contact_ApplyAirborne_Torque(chassis_move_t *bl_ctrl)
{
	// 处理 joint_balancing_torque（平衡力矩）
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
		bl_ctrl->torque_info.joint_balancing_torque_R = (
			+ LQR[3][6] * (bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle_set - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_angle)
            + LQR[3][7] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_R.leg_gyro) 
        );
        bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
    }
    
    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
		bl_ctrl->torque_info.joint_balancing_torque_L = (
			+ LQR[2][4] * (bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle_set - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_angle)
            + LQR[2][5] * (0.0f - bl_ctrl->chassis_posture_info.chassis_posture_L.leg_gyro) 
        );
        bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
    }
    
    // 处理 foot 相关力矩
    if (bl_ctrl->flag_info.suspend_flag_R == 1)
    {
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_R = 0;
    }
    
    if (bl_ctrl->flag_info.suspend_flag_L == 1)
    {
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
        bl_ctrl->torque_info.foot_moving_torque_L = 0;
    }
}
