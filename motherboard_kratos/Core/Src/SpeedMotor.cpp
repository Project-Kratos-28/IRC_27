#include "SpeedMotor.h"

SpeedMotor::SpeedMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                        GPIO_TypeDef* dirPort, uint16_t dirPin1, uint16_t dirPin2,
                        float kp, float ki, float kd)
    : pwmTim_(pwmTim), pwmChannel_(pwmChannel), pwmPeriod_(pwmPeriod),
      dirPort_(dirPort), dirPin1_(dirPin1), dirPin2_(dirPin2),
      pid_(kp, ki, kd, -100.0f, 100.0f),
      lastOutput_(0.0f), maxDuty_(100.0f)
{
    // No wrap here - speed is not circular like an absolute position is.
}

void SpeedMotor::begin() {
    if (!pwmTim_ || !dirPort_) return;
    // pwmPeriod == 0 -> use the timer's real ARR as 100 % duty (set in CubeMX).
    if (pwmPeriod_ == 0) pwmPeriod_ = __HAL_TIM_GET_AUTORELOAD(pwmTim_);
    HAL_TIM_PWM_Start(pwmTim_, pwmChannel_);
    // Only advanced timers (TIM1/8/15/16/17) have a break unit / MOE bit.
    if (IS_TIM_BREAK_INSTANCE(pwmTim_->Instance)) __HAL_TIM_MOE_ENABLE(pwmTim_);
    stop();
}

void SpeedMotor::setTargetSpeed(float rpmSetpoint) {
    pid_.setSetpoint(rpmSetpoint);
}

void SpeedMotor::setGains(float kp, float ki, float kd) {
    pid_.setGains(kp, ki, kd);
}

void SpeedMotor::update(float currentSpeedRPM, float dt) {
    float output = pid_.compute(currentSpeedRPM, dt);
    lastOutput_ = output;
    driveOutput(output);
}

void SpeedMotor::stop() {
    if (dirPort_) {
        HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dirPort_, dirPin2_, GPIO_PIN_RESET);
    }
    if (pwmTim_) {
        __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, 0);
    }
    lastOutput_ = 0.0f;
    pid_.reset(); // drop stale integral so restarting doesn't lurch
}

void SpeedMotor::setMaxDuty(float percent) {
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    maxDuty_ = percent;
    pid_.setOutputLimits(-percent, percent);
}

float SpeedMotor::getMaxDuty() const {
    return maxDuty_;
}

float SpeedMotor::getOutput() const {
    return lastOutput_;
}

float SpeedMotor::getTargetSpeed() const {
    return pid_.getSetpoint();
}

void SpeedMotor::driveOutput(float pidOutput) {
    if (!pwmTim_ || !dirPort_) return;

    // 1. Single DIR Pin Logic (High = Forward, Low = Reverse)
    if (pidOutput >= 0.0f) {
        HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_RESET);
    }

    // 2. PWM Duty Cycle (0 to maxDuty_ %, voltage clamp)
    float duty = (pidOutput < 0.0f) ? -pidOutput : pidOutput;
    if (duty > maxDuty_) duty = maxDuty_;

    // Convert duty percentage to Timer Compare Register value (ARR)
    uint32_t compareValue = static_cast<uint32_t>((duty / 100.0f) * static_cast<float>(pwmPeriod_));
    __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, compareValue);
}
