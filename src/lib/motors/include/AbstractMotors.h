/**
 * @file AbstractMotors.h
 * @ingroup motors
 * @brief Interface abstraite des moteurs.
 */

#ifndef MOTORS_ABSTRACTMOTORS_H
#define MOTORS_ABSTRACTMOTORS_H

#include <cstdint>
#include "AbstractDriver.h"

class AbstractMotor
{
public:
    /**
     * @brief Destructeur virtuel.
     */
    virtual ~AbstractMotor() = default;
    /**
     * @brief Applique une nouvelle vitesse.
     *
     * Met la vitesse du moteur à la valeur passée en paramètre.
     *
     * @param velocity Nouvelle vitesse.
     */
    virtual void setVelocity(float velocity) {
        this->m_driver->setVelocity(velocity, this->m_driverId);
    };

    /**
     * @brief Retourne la vitesse maximale.
     *
     * Retourne la vitesse maximale du moteur pour son intégration dans les
     * calculs du contrôleur différentiel.
     *
     * @return Vitesse maximale.
     */
    [[nodiscard]] virtual float getMaxVelocity() const = 0;

    AbstractDriver* m_driver = nullptr;
    uint8_t m_driverId = 255;

};
#endif //MOTORS_ABSTRACTMOTORS_H