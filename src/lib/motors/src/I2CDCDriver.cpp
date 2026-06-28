//
// Created by Romain Pouvreau on 23/04/2026.
//

#include "I2CDCDriver.h"
#ifndef ARDUINO
#include <cmath>
#include "AbstractMotors.h"

I2CDCDriver::I2CDCDriver(int m1_fwd_port, int m1_bck_port, int m1_pwm, int m2_fwd_port, int m2_bck_port, int m2_pwm, int stdby_port) : m_currentVelocity(0.0f)
{
    // Initialisation du tableau de moteurs hérité de AbstractDriver
    for (auto & m_motor : m_motors)
    {
        m_motor = nullptr;
    }

    m_m1_fwd_port = m1_fwd_port;
    m_m1_bck_port = m1_bck_port;
    m_m2_fwd_port = m2_fwd_port;
    m_m2_bck_port = m2_bck_port;
    m_stdby_port = stdby_port;
    m_m1_pwm = m1_pwm;
    m_m2_pwm = m2_pwm;
}

void I2CDCDriver::init(i2c_dev_t* dev)
{
    m_dev = dev;
#ifdef ARDUINO
#elif ESP_PLATFORM
    ESP_ERROR_CHECK(pca9685_init(m_dev));
    ESP_ERROR_CHECK(pca9685_restart(m_dev));

    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(m_dev, 1526));

    ESP_ERROR_CHECK(pca9685_set_pwm_value(m_dev, m_stdby_port, PCA9685_HIGH));
#endif
}

int I2CDCDriver::attach(AbstractMotor* motor)
{
    if(m_motors[0] == nullptr)
        return attach(motor, 0);
    else if(m_motors[1] == nullptr)
        return attach(motor, 1);
    return -1;
}

int I2CDCDriver::attach(AbstractMotor* motor, uint8_t id)
{
    if (id >= 2 || id >= MAX_MOTOR_IN_DRIVER)
    {
        return -1;
    }
    m_motors[id] = motor;

    m_motors[id]->m_driver = this;
    m_motors[id]->m_driverId = id;
    return id;
}

void I2CDCDriver::setVelocity(float velocity, uint8_t id)
{
    if (m_motors[id] == nullptr) return;

    float absSpeed = fabsf(velocity);
    int pwmValue = saturate(absSpeed * PCA9685_HIGH, PCA9685_LOW, PCA9685_HIGH);
    bool forward = velocity >= 0;
    bool dirChanged = (forward != m_lastForward[id]);  // add bool m_lastForward[2] to header

    uint8_t pwm_port = (id == 0) ? m_m1_pwm     : m_m2_pwm;
    uint8_t fwd_port = (id == 0) ? m_m1_fwd_port : m_m2_fwd_port;
    uint8_t bck_port = (id == 0) ? m_m1_bck_port : m_m2_bck_port;

    // Only update direction pins when direction actually changes (saves 2 I2C writes)
    if (dirChanged)
    {
        pca9685_set_pwm_value(m_dev, fwd_port, forward ? PCA9685_HIGH : PCA9685_LOW);
        pca9685_set_pwm_value(m_dev, bck_port, forward ? PCA9685_LOW  : PCA9685_HIGH);
        m_lastForward[id] = forward;
    }

    // Always update PWM speed (1 I2C write per motor instead of 3)
    pca9685_set_pwm_value(m_dev, pwm_port, pwmValue);

    if (id == 0)
    {
        printf("%f\n", velocity);
    }
}

float I2CDCDriver::getVelocity(uint8_t id) const
{
    return (id == 0) ? m_currentVelocity : 0.0f;
}

bool I2CDCDriver::isFaulty() const
{
    return false;
}

void I2CDCDriver::setSleep(bool active)
{
#ifdef ARDUINO

#elif ESP_PLATFORM
        ESP_ERROR_CHECK(pca9685_set_pwm_value(m_dev, m_stdby_port, active ? PCA9685_HIGH : PCA9685_LOW));
#endif

}
#endif