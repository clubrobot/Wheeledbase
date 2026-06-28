//
// Created by Romain Pouvreau on 15/04/2026.
//

#include "DCMotor.h"

DCMotor::DCMotor(float maxVelocity)
    : m_maxVelocity(maxVelocity)
{
    // Initialisation héritée de AbstractMotor
    m_driver = nullptr;
    m_driverId = 255;
}

float DCMotor::getMaxVelocity() const
{
    return m_maxVelocity;
}

void DCMotor::setMaxVelocity(float maxVelocity)
{
    m_maxVelocity = maxVelocity;
}