/*
 * Motor1.c
 *
 *  Created on: 2026年10月20日
 *      Author: Lenovo
 */


#include "Motor1.h"





//参数调节区
volatile int16 motor_base_speed=45;//50，70
//volatile int16 motor_base_speed_low=30;
volatile float motor_position_kp=1.2 ;//2.6,1.4
//volatile float motor_position_kp_low=15.0;
volatile int16 motor_speed_kp_left=120;
volatile int16 motor_speed_ki_left=380;
volatile int16 motor_speed_kp_right=115;
volatile int16 motor_speed_ki_right=280;
volatile int16 motor_speed_ff=35;
//显示区
volatile int16 left_count= 0;//编码器读数
volatile int16 right_count= 0;
volatile int32 left_pwm = 0;//输出pwm
volatile int32 right_pwm = 0;
volatile int32 left_target;//目标速度
volatile int32 right_target;

//积分存储全局变量
static int32 motor_speed_integral_left = 0;
static int32 motor_speed_integral_right = 0;

//影子变量
volatile int32 left_pwm_f = 0;//输出pwm
volatile int32 right_pwm_f = 0;//输出pwm
volatile int32 left_target_f = 0;//目标速度
volatile int32 right_target_f = 0;
volatile int left_count_f=0;
volatile int right_count_f=0;
volatile uint8 prt_flg=0;

//积分变量的累加限制
static int32 motor_limit_i(int32 value)
{
    if(value >10000) return 10000;
    if(value < -10000) return -10000;
    return value;
}
//

//电机初始化
void motor_init(void)
{
    gpio_init (P33_5,  GPO, 1,  GPO_PUSH_PULL);//失常
    pwm_init(ATOM1_CH1_P33_9, 20000, 0);
    gpio_init (P33_11,  GPO,0, GPO_PUSH_PULL);//正常
    pwm_init(ATOM3_CH5_P33_13, 20000, 0);
    motor_speed_integral_left=0;
    motor_speed_integral_right=0;
}
//

//pwm限制
static uint16 motor_limit_pwm(int32 value)
{
    if(value <= 0) return 0  ;
    if(value >= 7000) return 7000;
    return (uint16)value;
}
//

//速度环的pi调控，左轮的参数有待调整
static int32 motor_speed_left_pi(int16 target_speed, int16 measured_speed, int32 *integral)
{
    int32 error = (int32)target_speed - (int32)measured_speed;
    int32 correction;
    int32 output;

    *integral = motor_limit_i(*integral + error);
    correction = (int32)motor_speed_kp_left * error + (int32)motor_speed_ki_left * (*integral)/1000;
    output = (int32)motor_speed_ff * target_speed + correction;
    return output;
}

//速度环的pi调控，右轮的参数合理
static int32 motor_speed_right_pi(int16 target_speed, int16 measured_speed, int32 *integral)
{
    int32 error = (int32)target_speed - (int32)measured_speed;
    int32 correction;
    int32 output;

    *integral = motor_limit_i(*integral + error);
    correction = (int32)motor_speed_kp_right * error + (int32)motor_speed_ki_right * (*integral)/1000;
    output = (int32)motor_speed_ff * target_speed + correction;
    return output;
}
//



//丢线保护
void Motor_Stop(void)
{
    left_target=0;
    right_target=0;
    motor_speed_integral_left = 0;
    motor_speed_integral_right = 0;
    left_pwm = 0;
    right_pwm = 0;
    pwm_set_duty(ATOM1_CH1_P33_9, 0);
    pwm_set_duty(ATOM3_CH5_P33_13, 0);
}
//

//方向控制
void motor_positions(int32 err,uint8 position_valid)
{
    static int16 bssp=0;
    static float kp=0;
    if(zebra_stop_flag)
    {
        left_target = 0;
        right_target = 0;
        left_target_f = 0;
        right_target_f = 0;
        return;
    }
    if(!position_valid)
    {
        Motor_Stop();
        return;
    }
    if (left_count_f <= 100000)
    {
        bssp = motor_base_speed +25;
        kp = motor_position_kp + 0.1f;
    }
    else if (left_count_f > 270000)
    {
        bssp = motor_base_speed + 25;
        kp = motor_position_kp + 0.1f;
    }
    else
    {
        bssp = motor_base_speed;
        kp = motor_position_kp;
    }
    float correction=kp*err;
    left_target = bssp - correction;
    right_target = bssp + correction;
    left_target_f=left_target;
    right_target_f=right_target;
}
//

//为斑马线停车加的反向输出能力
static void motor_output_signed(pwm_channel_enum channel,gpio_pin_enum dir_pin,uint8 forward_level,int32 value)
{

    if(!zebra_stop_flag && value < 0)
    {
        value = 0;
    }

    if(value > 7000)  value = 7000;
    if(value < -7000) value = -7000;

    if(value < 0)
    {
        gpio_set_level(dir_pin, !forward_level);
        value = -value;
    }
    else
    {
        gpio_set_level(dir_pin, forward_level);
    }

    pwm_set_duty(channel, (uint32)value);
}
//

//5ms速度环
void Motor_SpeedLoop_5ms(void)
{
    left_count = encoder_get_count(TIM6_ENCODER);
    right_count = -encoder_get_count(TIM2_ENCODER);
    left_count_f += left_count;
    right_count_f += right_count;
    encoder_clear_count(TIM2_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
    if(zebra_stop_flag)
    {
        left_target = 0;
        right_target = 0;
    }
    left_pwm = motor_speed_left_pi(left_target, left_count, &motor_speed_integral_left);
    right_pwm = motor_speed_right_pi(right_target, right_count, &motor_speed_integral_right);
    left_pwm_f = left_pwm;
    right_pwm_f = right_pwm;
    motor_output_signed(ATOM3_CH5_P33_13, P33_11, 0, left_pwm);
    motor_output_signed(ATOM1_CH1_P33_9,  P33_5,  1, right_pwm);
}
//
