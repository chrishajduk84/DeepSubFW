/*
 * servo.cpp
 *
 *  Created on: Mar 21, 2022
 *      Author: Chris Hajduk
 */

#include <motors.h>

Motor::Motor(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t servo_max, uint16_t servo_min)
{
	TIMER = timer_handle;
	CHANNEL = channel_num;
	PWM_MAX = servo_max;
	PWM_MIN = servo_min;
	duty = duty_cycle;
}

Motor::~Motor()
{
	HAL_TIM_PWM_Stop(&TIMER, CHANNEL); // stop generation of pwm
}


// set_pwm_period
void Motor::set_pulse(uint16_t pulse_duration)
{
	pulse = pulse_duration;
	update();
}

void Motor::set_duty_cycle(uint16_t duty_cycle)
{
	duty = duty_cycle;
	update();
}

void Motor::update()
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


/**************************************************************
 * Servo
 **************************************************************/

Servo::Servo(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t pwm_max, uint16_t pwm_min, float deg_range)
	: Motor::Motor(timer_handle, channel_num, duty_cycle, pwm_max, pwm_min)
{
	RANGE = deg_range;
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

	uint16_t output_pulse = static_cast<uint16_t>((PWM_MAX - PWM_MIN)/RANGE * deg_angle + PWM_MIN);

	set_pulse(output_pulse);
}

float Servo::get_angle() const
{
	return (pulse - PWM_MIN) * RANGE/(PWM_MAX - PWM_MIN);
}

float Servo::get_range() const
{
	return RANGE;
}

/**************************************************************
 * Thruster
 **************************************************************/

Thruster::Thruster(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t pwm_max, uint16_t pwm_min, uint16_t pwm_idle, float forward_start_threshold, float back_start_threshold)
	: Motor::Motor(timer_handle, channel_num, duty_cycle, pwm_max, pwm_min)
{
	PWM_IDLE = pwm_idle;

	// Forward and backward starting thresholds are specified in percentages (make sure the +/- sign is correctly set)
	if (forward_start_threshold < 0) forward_start_threshold *= -1;
	if (back_start_threshold > 0) back_start_threshold *= -1;
	FORWARD_THRESHOLD = forward_start_threshold;
	BACKWARD_THRESHOLD = back_start_threshold;

}

void Thruster::set_thrust(float percent_thrust)
{
	//Ensure low thrust is big enough

	// PWM_MAX - PWM_MIN is a 200% range (meaning valid values of percent_thrust are +100% -> -100%)
	// PWM_IDLE is not guaranteed to be (PWM_MAX - PWM_MIN)/2
	uint16_t output_pulse = PWM_IDLE;
	if (percent_thrust > 0)
	{
		// FORWARDS
		if (percent_thrust < FORWARD_THRESHOLD) percent_thrust = 0;
		output_pulse = static_cast<uint16_t>((PWM_MAX - PWM_IDLE)*(percent_thrust/100.0) + PWM_IDLE);
	}
	else
	{
		// BACKWARDS
		if (percent_thrust > BACKWARD_THRESHOLD) percent_thrust = 0;
		output_pulse = static_cast<uint16_t>((PWM_IDLE - PWM_MIN)*(percent_thrust/100.0) + PWM_IDLE);
	}

	set_pulse(output_pulse);
}

float Thruster::get_thrust()
{
	float thrust = 0;
	if (pulse >= PWM_IDLE)
	{
		thrust = pulse * 100.0/(PWM_MAX - PWM_IDLE);
	}
	else if (pulse < PWM_IDLE)
	{
		thrust = pulse * 100.0/(PWM_MIN - PWM_IDLE);
	}
	return thrust;
}

void Thruster::arm_sequence()
{

}
