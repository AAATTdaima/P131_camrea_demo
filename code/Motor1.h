/*
 * Motor1.h
 *
 *  Created on: 2026年10月20日
 *      Author: Lenovo
 */

#ifndef CODE_MOTOR1_H_
#define CODE_MOTOR1_H_
#include "zf_common_headfile.h"

extern volatile int16 motor_base_speed;
//extern volatile int16 motor_base_speed_low;
extern volatile float motor_position_kp;
//extern volatile float motor_position_kp_low;
extern volatile int16 motor_speed_kp_left;
extern volatile int16 motor_speed_ki_left;
extern volatile int16 motor_speed_kp_right;
extern volatile int16 motor_speed_ki_right;
extern volatile int16 motor_speed_ff;

extern volatile int16 left_count;//编码器读数
extern volatile int16 right_count;
extern volatile int32 left_pwm;//输出pwm
extern volatile int32 right_pwm;
extern volatile int32 left_target;//目标速度
extern volatile int32 right_target;

extern volatile int32 left_pwm_f;//输出pwm
extern volatile int32 right_pwm_f;//输出pwm
extern volatile int32 left_target_f;//目标速度
extern volatile int32 right_target_f;
extern volatile int left_count_f;
extern volatile int right_count_f;
extern volatile uint8 prt_flg;

void motor_init(void);
void motor_positions(int32 err,uint8 position_valid);
void Motor_SpeedLoop_5ms();
void Motor_Stop();

#endif /* CODE_MOTOR1_H_ */
