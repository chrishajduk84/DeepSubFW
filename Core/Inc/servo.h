/*
 * servo.h
 *
 *  Created on: Mar 21, 2022
 *      Author: Chris Hajduk
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

class Servo
{
public:
	Servo(TIM_HandleTypeDef timer_handle, uint32_t channel_num, uint16_t duty_cycle, uint16_t servo_max, uint16_t servo_min, float deg_range=270.0);
	~Servo();
	void set_angle(float deg_angle);
	void set_duty_cycle(uint16_t duty_cycle); // Do I really want this public?
	void set_pulse(uint16_t pulse_duration); // Do I really want this public?
	float get_range() const;
	float get_angle() const;

private:
	TIM_HandleTypeDef TIMER;
	uint32_t CHANNEL;
	float RANGE;
	uint16_t SERVO_MAX;
	uint16_t SERVO_MIN;
	uint16_t duty;
	uint16_t pulse;


	void update();

};


#ifdef __cplusplus
}
#endif

#endif /* SRC_SERVO_H_ */
