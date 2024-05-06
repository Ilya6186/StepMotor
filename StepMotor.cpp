#include "StepMotor.h"


	StepMotor::StepMotor(GPIO_TypeDef *GPIOx_Enable, uint16_t pin_Enable,GPIO_TypeDef *GPIOx_Dir,
			uint16_t pin_Dir, TIM_HandleTypeDef *htim_PWM, uint32_t Channel)
	{

		htim_PWM-> Instance->CR1 |=  TIM_CR1_ARPE;

		//---------------------------
		// pinout
		m_GPIOx_Enable = GPIOx_Enable;
		m_GPIOx_Dir = GPIOx_Dir;
		p_htim_PWM = htim_PWM;			// pointer to taimer
		m_Channel = Channel;			// chanel taimer
		m_pin_enable = pin_Enable;		// pin enable
		m_pin_Dir = pin_Dir;

		//---------------------------
		m_speed = 0;					// now speed
		m_MaxSpeed = 0;					// max speed
		m_MinSpeed = 0;					// min speed
		//---------------------------
		m_direction = 0;				// direction 1 - left, 0 - right
		m_counterSteps = 0;				// counter steps
		m_nStepsForMotion = 0;			// all steps
		//---------------------------
		m_Retention = 0;				// retention
		//---------------------------
		typeMotion = NO_MOTION;
		//---------------------------
		m_OneStepAcceleration = 0;			// значение на которое увеличивается скорость за один шаг
		m_stepEndAcceleration = 0;		// шаг конца ускорения
		m_OneStepBrake = 0;				// значение на которое уменьшается скорость за один шаг
		m_stepStartBrake = 0;			// шаг начала торможения

		HAL_TIM_PWM_Stop(p_htim_PWM, m_Channel);
		htim_PWM-> Instance->CR1 |=  TIM_CR1_ARPE;

		this->setMinSpeed(1);
	}

	void StepMotor::motorService()		//вызывается в коллбэке
	{
		HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_RESET);

		if(m_direction)
		{
			HAL_GPIO_WritePin(m_GPIOx_Dir, m_pin_Dir, GPIO_PIN_SET);
		}
		else if(!m_direction)
		{
			HAL_GPIO_WritePin(m_GPIOx_Dir, m_pin_Dir, GPIO_PIN_RESET);
		}

		if(m_counterSteps <= m_nStepsForMotion)
		{
			if(m_counterSteps <= m_stepEndAcceleration)
				accelerationService(m_counterSteps);

			else if(m_counterSteps >= m_stepStartBrake)
				brakeService(m_counterSteps);

			m_counterSteps++;
		}
		else
		{
			m_speed = 0;
			typeMotion = NO_MOTION;
			m_counterSteps = 0;
			stopMotion();
			setRetention(m_Retention);
		}
	}


	void StepMotor::checkMotorInCallback(TIM_HandleTypeDef *htim)
	{
		if(htim == this->p_htim_PWM)
			{
				motorService();
			}
	}

	void StepMotor::setDirection(uint8_t in_direction)
	{
		m_direction = in_direction;
	}

	uint8_t StepMotor::getDirection()
	{
		return m_direction;
	}

	void StepMotor::startMotion(uint32_t steps)
	{
		int pointStartBrake_Acceleration = steps * 0.15;			// 15% ШАГОВ ИСПОЛЬЗУЕМ ДЛЯ РАЗГОНА И ТОРМОЖЕНИЯ
		int stepsAcceleration_Brake = pointStartBrake_Acceleration / 100;		// КОЛ-ВО ШАГОВ УВЕЛИЧЕНИЯ ЧАСТОТЫ
		this->setAccelerationStep(stepsAcceleration_Brake, pointStartBrake_Acceleration);
		this->setBrakeMotorStep(stepsAcceleration_Brake, pointStartBrake_Acceleration);

		if(typeMotion == NO_MOTION)
		{
			calculateFreqBrakeStep();
			calculateFreqAccelerationStep();
			typeMotion = ACCELERATION;
			m_nStepsForMotion = steps;
			HAL_TIM_PWM_Start_IT(p_htim_PWM, TIM_CHANNEL_1);
		}
	}

	void StepMotor::stopMotion()
	{
		HAL_TIM_PWM_Stop(p_htim_PWM, m_Channel); 				// остановить шим
	//	HAL_TIM_PWM_Stop_IT(p_htim_PWM, TIM_CHANNEL_1);
	}

	void StepMotor::setMaxSpeed(uint32_t maxSpeed)
	{
		m_MaxSpeed = maxSpeed;
	}

	uint32_t StepMotor::getSpeed()
	{
		return m_MaxSpeed;
	}

	uint32_t StepMotor::getMaxSpeed()
	{
		return m_MaxSpeed;
	}


	void StepMotor::setMinSpeed(uint32_t speed)
	{
		m_MinSpeed = speed;
	}

	uint32_t StepMotor::getMinSpeed()
	{
		return m_MinSpeed;
	}

	void StepMotor::setRetention(bool Retention)
	{
		m_Retention = Retention;

		if(m_Retention)
			HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_RESET);
		else if(!m_Retention)
			HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_SET);
	}

	void StepMotor::setSpeed(uint32_t speed)
	{
		if(speed <= 0)
			return;

		if(speed < m_MinSpeed)
			speed = m_MinSpeed;

		m_speed = speed;
		int arr = (HAL_RCC_GetSysClockFreq() / p_htim_PWM->Instance->PSC) / speed ;
		p_htim_PWM -> Instance -> ARR = arr - 1;
		p_htim_PWM -> Instance -> CCR1 = arr / 2 - 1;


	}

	void StepMotor::setAccelerationStep(uint32_t steps, uint32_t stepEndAcceleration)		// до какого шага увеличение частоты
	{
		m_OneStepAcceleration = stepEndAcceleration / steps;
		m_stepEndAcceleration = stepEndAcceleration;
		m_stepsAcceleration = steps;
	}

	void StepMotor::calculateFreqAccelerationStep()		// до какого шага увеличение частоты
	{
		m_stepFreqAcceleration = m_MaxSpeed / m_stepsAcceleration;
	}

	void StepMotor::calculateFreqBrakeStep()		// до какого шага увеличение частоты
	{
		m_stepFreqBrake = m_MaxSpeed / m_stepsBrake;
	}


	void StepMotor::accelerationService(uint32_t i)
	{
		if (i == 0 || i > m_stepEndAcceleration)
			return;

		if (i == m_stepEndAcceleration)
		{
			setSpeed(m_MaxSpeed);
			return;
		}

		if(i % m_OneStepAcceleration == 0)	//
		{
			if(m_speed <= m_MaxSpeed)
				setSpeed(m_speed + m_stepFreqAcceleration);
		}

	}

	void StepMotor::setBrakeMotorStep(uint32_t stepsBrake, uint32_t stepsForEndBraking)
	{
		m_OneStepBrake = stepsForEndBraking / stepsBrake;
		m_stepStartBrake = m_nStepsForMotion - stepsForEndBraking;
		m_stepsBrake = stepsBrake;
	}

	void StepMotor::brakeService(uint32_t i)
	{
		if (i == 0 || i < m_stepStartBrake)
			return;

		if(i > m_nStepsForMotion)
		{
			stopMotion();
			return;
		}

		if(i % m_OneStepBrake == 0)	//
		{
			setSpeed(m_speed - m_stepFreqAcceleration);
		}
	}
