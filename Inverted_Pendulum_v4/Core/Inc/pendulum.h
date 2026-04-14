/*
 * pendulum.h
 *
 *  Created on: Mar 13, 2026
 *      Author: dalya
 */

#ifndef INC_PENDULUM_H_
#define INC_PENDULUM_H_
#include "main.h"
#include <stdint.h>
#define AS5600_ADDR (0x36 << 1)
#define RAW_ANGLE_REG 0x0C
#define AS5600_RESOLUTION 28
#define ENC_RESOLUTION 65536   // if timer is 16-bit

// motor - hall effect encoder
#define M_CPR 28
#define M_CPR2RAD (2.0*PI/M_CPR)
// pendulum - quadrature encoder
#define P_CPR (1600)
#define P_CPR2RAD (2.0*PI/P_CPR)
#define P_OFFSET PI // encoder inital position offset //PI

// angle where linear controller switchen in
#define P_SWITCH_ANGLE PI/10

// Control time constants
#define M_Ts 0.01 	// motor sampling time - oversamplig
#define P_Ts (0.01) // pendulum sampling time
#define M_WTf 0.015  // motor velocity filtering time constant
#define P_WTf 0.02   // pendulum velocity filtering time constant

// Voltage and PWM constants
#define PWMmax 100
#define M_Umax (PWMmax)
#define u2pwm_(u) (u*M_Umax/U_PWR)
#define sign_(a) (((a)<0) ? -1 : ((a)>0))
#define U_PWR  12

// SWING-UP parameters
// Physical system parameters
// chnage once mechanical model changed
#define P_Mp    0.85       //0,664  //	0.236					// full pendulum mass  |new motor 0.425kg
#define P_Mmw   0.67  //0.484    		// motor+wheel mass                    |new motor 0.425kg
#define P_Lcm   0.20 //0.210   //0.11 			// center of mass distance
#define P_Jw   	1.0125e−4//101250e-9     // Wheel momentum of inertia 162231e-9
#define P_g  		9.81          // gravity acceleration
#define P_Jp   (P_Mp*pow(P_Lcm,2) + 1153811.05e-9)   // full pendulum momentum of inertia

//#define P_Jp   (P_Mp*pow(P_Lcm,2))   // full pendulum momentum of inertia

#define E_target  (2*P_Mp*P_g*P_Lcm) // Potential energy of upright pendulum


// encoder parameters
#define M_CNT_MAX 65535.0 //2048
#define M_CNT_OVERFLOW 65535.0
#define P_CNT_MAX 65535
#define P_CNT_OVERFLOW 65535
#define PI 3.14159265359

// struct for geting motor variables
struct motor_s{
	float position;
	float velocity;
};
extern  struct motor_s motor;
struct pendulum_s{
	float position;
	float velocity;
	float E_tot; // current Ekin+Epot
	float E_tar; // Potential energy of upright pendulum
};
extern  struct pendulum_s pendulum;


float GetPendulumPosition(void);
void motor_setPwm(int16_t pwm);
int16_t motor_MapPwm(int16_t pwm);
#endif /* INC_PENDULUM_H_ */
