/*
 * servo.cpp
 *
 *  Created on: Mar 21, 2022
 *      Author: Chris Hajduk
 */

#include "servo.h"

Servo::Servo(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t servo_max, uint16_t servo_min, float deg_range)
{
	TIMER = timer_handle;
	CHANNEL = channel_num;
	RANGE = deg_range;
	SERVO_MAX = servo_max;
	SERVO_MIN = servo_min;
	duty = duty_cycle;
}

Servo::~Servo()
{
	HAL_TIM_PWM_Stop(&TIMER, CHANNEL); // stop generation of pwm
}

void Servo::set_angle(float deg_angle)
{
	if (deg_angle > RANGE)
	{
		deg_angle = RANGE;
	}
	else if (deg_angle < 0)
	{
		deg_angle = 0;
	}

	uint16_t output_pulse = static_cast<uint16_t>((SERVO_MAX - SERVO_MIN)/RANGE * deg_angle + SERVO_MIN);

	set_pulse(output_pulse);
}

// set_pwm_period
void Servo::set_pulse(uint16_t pulse_duration)
{
	pulse = pulse_duration;
	update();
}

void Servo::set_duty_cycle(uint16_t duty_cycle)
{
	duty = duty_cycle;
	update();
}

void Servo::update()
{
	HAL_TIM_PWM_Stop(&TIMER, CHANNEL); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	TIMER.Init.Period = duty; // set the period duration
	HAL_TIM_PWM_Init(&TIMER); // reinititialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&TIMER, &sConfigOC, CHANNEL);
	HAL_TIM_PWM_Start(&TIMER, CHANNEL); // start pwm generation
}


float Servo::get_range() const
{
	return RANGE;
}

float Servo::get_angle() const
{
	return (pulse - SERVO_MIN) * RANGE/(SERVO_MAX - SERVO_MIN);
}



