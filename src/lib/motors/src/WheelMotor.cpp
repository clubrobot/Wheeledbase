//
// Created by Romain Pouvreau on 23/04/2026.
//

#include "WheelMotor.h"

#include <math.h>

WheelMotor::WheelMotor(float maxVelocity)
    : m_maxVelocity(maxVelocity)
{
    // Initialisation héritée de AbstractMotor
    m_driver = nullptr;
    m_driverId = 255;
}

float WheelMotor::getMaxVelocity() const
{
    return m_maxVelocity;
}

void WheelMotor::setMaxVelocity(float maxVelocity)
{
    m_maxVelocity = maxVelocity;
}

void WheelMotor::setWheelRadius(float wheelRadius)   {
    m_wheelRadius = wheelRadius;
}

void WheelMotor::setVelocity(float velocity)
{
    this->m_driver->setVelocity(velocity / (2 * M_PI * m_wheelRadius) * m_constant, this->m_driverId);
}

void WheelMotor::setConstant(float constant)   {
    m_constant = constant;
}