/*
 * servo.h
 *
 *  Created on: Mar 21, 2022
 *      Author: Chris Hajduk
 */

#ifndef SRC_MOTORS_H_
#define SRC_MOTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


class Motor
{
public:
	Motor(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t pwm_max, uint16_t pwm_min);
	~Motor();

	void set_duty_cycle(uint16_t duty_cycle); // Do I really want this public?
	void set_pulse(uint16_t pulse_duration); // Do I really want this public?

protected:
	TIM_HandleTypeDef TIMER;
	uint32_t CHANNEL;
	uint16_t PWM_MAX;
	uint16_t PWM_MIN;
	uint16_t duty;
	uint16_t pulse;

	void update();

};

class Servo : public Motor
{
public:
	Servo(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t pwm_max, uint16_t pwm_min, float deg_range=270.0);
	void set_angle(float deg_angle);
	float get_angle() const;
	float get_range() const;
private:
	float RANGE;
};

class Thruster : public Motor
{
public:
	Thruster(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t pwm_max, uint16_t pwm_min, uint16_t pwm_idle, float forward_start_threshold=8.0, float back_start_threshold=8.0);
	void set_thrust(float percent_thrust);
	float get_thrust();
	void arm_sequence();
private:
	uint16_t PWM_IDLE;
	float FORWARD_THRESHOLD;
	float BACKWARD_THRESHOLD;
};

#ifdef __cplusplus
}
#endif

#endif /* SRC_MOTORS_H_ */
