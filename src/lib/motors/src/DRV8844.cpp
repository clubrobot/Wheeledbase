//
// Created by Romain Pouvreau on 29/04/2026.
//

#include "DRV8844.h"

#include "AbstractMotors.h"

DRV8844::DRV8844(int m1_en, int m1_dir, int m1_pwm, int m2_en, int m2_dir, int m2_pwm, int fault, int rst)
{
    this->m_m1_en = m1_en;
    this->m_m1_dir = m1_dir;
    this->m_m1_pwm = m1_pwm;
    this->m_m2_en = m2_en;
    this->m_m2_dir = m2_dir;
    this->m_m2_pwm = m2_pwm;
    this->m_fault = fault;
    this->m_rst = rst;
}

void DRV8844::init()
{
    analogWriteFrequency(20000);
#ifdef ARDUINO
    pinMode(this->m_m1_en, OUTPUT);
    pinMode(this->m_m1_dir, OUTPUT);
    pinMode(this->m_m1_pwm, OUTPUT);
    pinMode(this->m_m2_en, OUTPUT);
    pinMode(this->m_m2_dir, OUTPUT);
    pinMode(this->m_m2_pwm, OUTPUT);
    pinMode(this->m_fault, INPUT);
    pinMode(this->m_rst, OUTPUT);

    digitalWrite(this->m_rst, HIGH);

#else
    printf("DRV8844::init() is not implemented \n");
#endif
}

int DRV8844::attach(AbstractMotor* motor)
{
    if(m_motors[0] == nullptr)
        return attach(motor, 0);
    else if(m_motors[1] == nullptr)
        return attach(motor, 1);
    return -1;
}

int DRV8844::attach(AbstractMotor* motor, uint8_t id)
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

void DRV8844::setVelocity(float velocity, uint8_t id)
{
    if (m_motors[id] == nullptr)
    {
        return;
    };

    m_currentVelocity[id] = velocity;

    float absSpeed = fabsf(velocity);
    absSpeed = saturate(absSpeed, 0 ,1);

    float pwm = absSpeed * 255;

#ifdef ARDUINO

    if (id == 0)
    {
        digitalWrite(m_m1_en, HIGH);
        analogWrite(m_m1_pwm, pwm);
        digitalWrite(m_m1_dir, (velocity > 0) ? 1 : 0);

    }else if(id == 1){

        digitalWrite(m_m2_en, HIGH);
        analogWrite(m_m2_pwm, pwm);
        digitalWrite(m_m2_dir, (velocity > 0) ? 1 : 0);
    }

#elif
    printf("DRV8844::setVelocity() is not implemented \n");
#endif
}

float DRV8844::getVelocity(uint8_t id) const
{
    return (id == 0) ? m_currentVelocity[id] : 0.0f;
}

bool DRV8844::isFaulty() const
{
    return false;
}