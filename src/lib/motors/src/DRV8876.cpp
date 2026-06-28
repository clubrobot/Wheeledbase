//
// Created by Romain Pouvreau on 15/04/2026.
//

#include "DRV8876.h"

#include "AbstractMotors.h"

DRV8876::DRV8876(int enPin, int phPin, int sleepPin, int faultPin)
        : m_enPin(enPin),
          m_phPin(phPin),
          m_sleepPin(sleepPin),
          m_faultPin(faultPin),
          m_currentVelocity(0.0f)
    {
        // Initialisation du tableau de moteurs hérité de AbstractDriver
        for (auto & m_motor : m_motors) {
            m_motor = nullptr;
        }
    }

    void DRV8876::init() {

#ifdef ARDUINO
    // Configuration des directions des broches
    pinMode(m_enPin, OUTPUT);
    pinMode(m_phPin, OUTPUT);

    analogWriteFrequency(100000);

    if (m_sleepPin != -1) {
        pinMode(m_sleepPin, OUTPUT);
        setSleep(false); // Réveil par défaut
    }

    if (m_faultPin != -1) {
        pinMode(m_faultPin, INPUT_PULLUP);
    }

    // Initialisation à l'arrêt
    analogWrite(m_enPin, 0);
#elif ESP_IDF
    //To be implemented
#endif
    }

    int DRV8876::attach(AbstractMotor* motor) {
        return attach(motor, 0);
    }

    int DRV8876::attach(AbstractMotor* motor, uint8_t id) {
        // Le DRV8876 est un driver monocanal.
        // On vérifie que l'ID demandé est 0 et qu'on ne dépasse pas la capacité du tableau.
        if (id >= 1 || id >= MAX_MOTOR_IN_DRIVER) {
            return -1;
        }
        m_motors[id] = motor;

        m_motors[id]->m_driver = this;
        m_motors[id]->m_driverId = id;
        return id;
    }

    void DRV8876::setVelocity(float velocity, uint8_t id) {
        if (id != 0 || m_motors[id] == nullptr)
        {
            return;
        };

        m_currentVelocity = velocity;

        float absSpeed = fabsf(velocity);
        if (absSpeed > m_motors[id]->getMaxVelocity()) absSpeed = m_motors[id]->getMaxVelocity();

        int pwmValue = static_cast<int>(absSpeed * 255.0f);

#ifdef ARDUINO
        digitalWrite(m_phPin, (velocity >= 0) ? HIGH : LOW);
        analogWrite(m_enPin, pwmValue);
#elif ESP_IDF
    //To be implemented
#endif
    }

    float DRV8876::getVelocity(uint8_t id) const {
        return (id == 0) ? m_currentVelocity : 0.0f;
    }

    bool DRV8876::isFaulty() const {
        if (m_faultPin != -1) {
            // nFAULT est "Active Low" : un état bas indique un défaut
#ifdef ARDUINO
            return (digitalRead(m_faultPin) == LOW);
#elif ESP_IDF
            //To be implemented
#endif
        }
        return false;
    }

    void DRV8876::setSleep(bool active) {
        if (m_sleepPin != -1) {
            // nSLEEP est "Active Low" : HIGH pour réveiller, LOW pour éteindre
#ifdef ARDUINO
            digitalWrite(m_sleepPin, active ? LOW : HIGH);
#elif ESP_IDF
            //To be implemented
#endif
        }
    }