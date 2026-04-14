/*
 * pendulum.c
 *
 *  Created on: Mar 13, 2026
 *      Author: dalya
 */

#include "pendulum.h"
#include <stdlib.h>
#include "as5600.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
struct motor_s motor;
struct pendulum_s pendulum;
int16_t tim1_counter = 0;
uint8_t state = 0;
int16_t global_pwm;
float global_e, global_e_pot;
float GetPendulumPosition(void)
{
	int16_t encoder_val = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	float pendulum_position = (((float)(encoder_val)/P_CPR)*2*PI) + P_OFFSET;
	return pendulum_position;
}
//max 1800
//min 1200
int16_t motor_MapPwm(int16_t pwm){
    // Map from [-100, 100] → [1200, 1800]
    int16_t rev_duty_time = 1200 + ((pwm + 100) * (1800 - 1200)) / 200;
    //1800 1200
    return rev_duty_time;
}

void motor_setPwm(int16_t pwm){
	if(abs(pwm) > PWMmax){
		pwm = pwm > 0 ? PWMmax : -PWMmax;
	}
	global_pwm = pwm;
	int16_t cnt_val = motor_MapPwm(pwm);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, cnt_val);
}

void motor_variables_iter(){
	static int32_t cnt_1 = 0;
	static float real_count = 0.0f;
	static float d_cnt_1 = 0.0f;
	static uint32_t prev_tick = 0;
	static uint8_t initialized = 0;

	uint32_t now = HAL_GetTick();
	int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

	if(!initialized){
		cnt_1 = cnt;
		prev_tick = now;
		initialized = 1;
		motor.position = 0.0f;
		motor.velocity = 0.0f;
		return;
	}

	float dt = (float)(now - prev_tick) * 1e-3f;
	if(dt <= 0.0f){
		return;
	}

	tim1_counter = cnt;
	float d_cnt = cnt - cnt_1;

	// from [0,6000] to [-inf,inf]
	if (fabs(d_cnt) > M_CNT_MAX){
		if(cnt > cnt_1){
			d_cnt = -(M_CNT_OVERFLOW - cnt + cnt_1);
		}else if(cnt < cnt_1){
			d_cnt = M_CNT_OVERFLOW - cnt_1 + cnt;
		}
	}

	// filtering d_pos Low pass
	d_cnt = (dt / (dt + M_WTf))*(d_cnt) + (1 - (dt / (dt + M_WTf)))*d_cnt_1;

	d_cnt_1 = d_cnt;
	real_count += d_cnt;
	cnt_1 = cnt;
	motor.position = real_count*M_CPR2RAD;
	motor.velocity = d_cnt/dt*M_CPR2RAD;

	prev_tick = now; // saving timestamp for next iteration
}

void pendulum_variables_iter(){
	static uint16_t cnt_1 = 0;
	static int32_t real_count = 0;
	static uint32_t prev_tick = 0;
	static uint8_t initialized = 0;
	static float d_cnt_1 = 0.0f;

	uint32_t now = HAL_GetTick();
	uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

	if (!initialized) {
		cnt_1 = cnt;
		prev_tick = now;
		initialized = 1;
		pendulum.position = 0.0f;
		pendulum.velocity = 0.0f;
		return;
	}

	float dt = (float)(now - prev_tick) * 1e-3f;
	if (dt <= 0.0f) {
		return;
	}

	/* Signed delta with automatic 16-bit wrap handling */
	int16_t d_cnt = (int16_t)(cnt - cnt_1);
	real_count += d_cnt;
	cnt_1 = cnt;
	float d_cnt_f = (float)d_cnt;
	d_cnt_f = (dt / (dt + P_WTf))*d_cnt_f + (1.0f - (dt / (dt + P_WTf)))*d_cnt_1;
	d_cnt_1 = d_cnt_f;

	/* Absolute angle in radians, wrapped to [-PI, PI] */
	float theta = ((float)real_count * P_CPR2RAD) - P_OFFSET;
	pendulum.position = atan2f(sinf(theta), cosf(theta));

	/* Angular velocity in rad/s */
	pendulum.velocity = (d_cnt_f / dt) * P_CPR2RAD;

	prev_tick = now;
}

void pendulum_LQR(){

	/** LQR Matlab generated controller
	* Q = diag([1,1,1]) R = [10]
	*/
	//float Kr[3] = {798.8827,114.3271,0.27871};
	//float Kr[3] = {1121.963,160.4269,0.24802};
	// P equation
	float uk = 150*pendulum.position;
	// LQR equation
	//float uk = Kr[0]*pendulum.position+Kr[1]*pendulum.velocity+Kr[2]*motor.velocity;

	//uk = u2pwm_(uk);
	// anti-windup
	if(abs((int)uk) > M_Umax) 	uk = uk > 0 ? M_Umax : -M_Umax;
	if(abs((int)uk) > M_Umax){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	}
	// set pwm to motor
	motor_setPwm(-uk);
}

void pendulum_swingup(){
    // check if swingup needed
    if(fabs(pendulum.position) >= P_SWITCH_ANGLE){
        float E_kin, E_pot;
        static float dir = 1.0f;

        // kinetic energy
        E_kin = 0.5f * P_Jp * powf(pendulum.velocity, 2.0f);
        // potential energy
        E_pot = P_Mp * P_g * P_Lcm * (1.0f + cosf(pendulum.position));
        // target energy
        pendulum.E_tar = E_target;
        pendulum.E_tot = E_kin + E_pot;

        /* Phase-aware energy pumping: sign(theta_dot*cos(theta)) */
        float phase = pendulum.velocity * cosf(pendulum.position);
        if (phase > 0.1f) {      // lowered from 0.25
            dir = 1.0f;
        } else if (phase < -0.1f) {
            dir = -1.0f;
        }
        if (phase > -0.02f && phase < 0.02f){  // lowered from 0.05
            dir = (pendulum.position >= 0.0f) ? -1.0f : 1.0f;
        }
        float e = pendulum.E_tar - pendulum.E_tot;

        // Reduced gain for low inertia
        float u = (80.0f * e) * dir;

        /* Compensate friction/stiction when still below target */
        if ((fabsf(e) > (0.08f * pendulum.E_tar)) && (fabsf(u) < 20.0f)) {
            u = dir * 20.0f;
        }

        // Saturate PWM safely
        if (u > M_Umax) {
            u = M_Umax;
        } else if (u < -M_Umax) {
            u = -M_Umax;
        }

        motor_setPwm((int16_t)u);
    }
}

void Controller_SysTick(){
	static int n_m,n_p;

	if(n_m >= (M_Ts*1000)){
		// gather state variables
		motor_variables_iter();
		n_m=0;
	}else{
		n_m++;
	}
	if(n_p >= (P_Ts*1000)){
		pendulum_variables_iter();

		if(fabs(pendulum.position) < P_SWITCH_ANGLE || state == 1){

			pendulum_LQR();
			//pendulum_swingup();
			state = 1;
			//pendulum_P();
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
			pendulum_swingup();
			state = 2;
			//pendulum_disable();
		}
		n_p = 0;
	}else{
		n_p++;
	}
}
