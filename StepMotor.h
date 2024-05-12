/*
 * StepMotor.h
 *
 *  Created on: Nov 28, 2023
 *      Author: user
 */

#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "main.h"


class StepMotor{

public:
	StepMotor(GPIO_TypeDef *GPIOx_Enable, uint16_t pin_Enable,GPIO_TypeDef *GPIOx_Dir,
			uint16_t pin_Dir, TIM_HandleTypeDef *htim_PWM, uint32_t Channel);


	uint32_t m_counterSteps;

private:

	enum TYPE_MOTION{
		NO_MOTION = 0,
		ACCELERATION = 1,
		MOTION = 2,
		BRACKING = 3,
		ERROR = 4
	};

	enum TYPE_MOTOR_CONTROL{
		DC_MOTOR = 0,
		STEP_MOTOR = 1
	};

	private:
	GPIO_TypeDef *m_GPIOx_Enable;
	GPIO_TypeDef *m_GPIOx_Dir;
	GPIO_TypeDef *m_GPIOx_Step;
	uint16_t m_pin_enable;
	uint16_t m_pin_Dir;
	uint16_t m_pin_step;

	TIM_HandleTypeDef *p_htim_PWM;

	TYPE_MOTOR_CONTROL typeMotorControl;

	uint32_t m_Channel;
	bool m_direction;
	uint32_t m_nStepsForMotion;
	uint32_t m_MaxSpeed;
	uint32_t m_MinSpeed;
	bool m_Retention;
	uint32_t m_speed;
	uint32_t m_stepEndAcceleration;
	uint32_t m_stepStartBrake;
	uint32_t m_OneStepBrake;
	uint32_t m_OneStepAcceleration;
	TYPE_MOTION m_typeMotion;
	uint32_t m_stepFreqAcceleration;
	uint32_t m_stepFreqBrake;
	uint32_t m_stepsAcceleration;
	uint32_t m_stepsBrake;

	public:
	void setDirection(uint8_t direction);
	void checkMotorInCallback(TIM_HandleTypeDef *htim);
	inline uint8_t getDirection();
	void startMotion(uint32_t steps, uint32_t maxSpeed, uint8_t procentAccelBrake,  uint16_t nStepsAccelBrake);
	void setMaxSpeed(uint32_t speed);
	uint32_t getSpeed();
	void setAccelerationStep(uint32_t step,  uint32_t stepEndAccep);
	void setSpeed(uint32_t speed);
	uint32_t getMaxSpeed();
	void setMinSpeed(uint32_t speed);
	uint32_t getMinSpeed();
	void setRetention(bool);
	void setBrakeMotorStep(uint32_t stepsBrake, uint32_t pointStartBraking);
	void stopMotion();
	inline int getMotorState();
	void startDC_Motion(uint16_t nSteps, uint16_t stepsInOneAccelStep);

	private:
	void calculateFreqBrakeStep();
	void calculateFreqAccelerationStep();
	void accelerationService();
	void motorService();
	void brakeService();
	void accelerationDCService(uint16_t stepsAccelerate, uint32_t stepMotorInStepAccel);
};




