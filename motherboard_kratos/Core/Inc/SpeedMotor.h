#ifndef SPEED_MOTOR_H_
#define SPEED_MOTOR_H_
#include "main.h"   // CubeMX-generated - pulls in the correct stm32xxxx_hal.h
#include "PID.h"

/*
 * Closed-loop speed control (drive wheels, anything where you regulate RPM
 * rather than a specific absolute position).
 *
 * You feed it an RPM reading every control cycle; it drives the PWM +
 * direction pins for you.
 */
class SpeedMotor {
public:
    // pwmTim / pwmChannel : timer + channel wired to the driver's PWM input
    // pwmPeriod           : ARR value of that timer (= 100% duty cycle).
    //                       0 = read the timer's ARR in begin().
    // dirPort/dirPin1/dirPin2 : two GPIOs for direction (H-bridge IN1/IN2 style)
    // kp, ki, kd           : starting PID gains (tune on the bench first)
    SpeedMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
               GPIO_TypeDef* dirPort, uint16_t dirPin1, uint16_t dirPin2,
               float kp, float ki, float kd);

    void begin();
    void setTargetSpeed(float rpmSetpoint);    // signed: negative = reverse
    void setGains(float kp, float ki, float kd);

    // Call this every control cycle (e.g. every 10 ms from a timer interrupt).
    // currentSpeedRPM: your latest speed measurement, signed if your sensor
    //                  can tell direction, otherwise unsigned (single-direction only).
    // dt: seconds since the previous call.
    void update(float currentSpeedRPM, float dt);

    void stop();                               // PWM off + PID state reset

    // Upper limit on PWM duty, 0..100 %. Use it when the motor's rated voltage is
    // below the supply: maxDuty = 100 * V_rated / V_supply. Also becomes the PID
    // output limit, so the integral can't wind up against the clamp. Default 100.
    void setMaxDuty(float percent);
    float getMaxDuty() const;
    float getOutput() const;                   // last PID output, -100..100 (%)
    float getTargetSpeed() const;

private:
    TIM_HandleTypeDef* pwmTim_;
    uint32_t pwmChannel_;
    uint32_t pwmPeriod_;

    GPIO_TypeDef* dirPort_;
    uint16_t dirPin1_;
    uint16_t dirPin2_;

    PID pid_;
    float lastOutput_;
    float maxDuty_;     // % (voltage clamp)

    void driveOutput(float pidOutput);
};

#endif /* SPEED_MOTOR_H_ */
