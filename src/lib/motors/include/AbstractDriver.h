/**
 * @defgroup motors Moteurs
 * @ingroup libs
 * @ingroup wheeledbase
 * @brief Différentes interface permettant de manipuler des moteurs
 */

/**
 * @file AbstractDriver.h
 * @ingroup motors
 * @brief Interface abstraite des drivers moteurs.
 */

#ifndef MOTORS_ABSTRACTDRIVER_H
#define MOTORS_ABSTRACTDRIVER_H

#define MAX_MOTOR_IN_DRIVER 2

#include <cstdint>

class AbstractMotor;


/**
 * @brief Représente un driver moteur abstrait.
 */
class AbstractDriver {
public:
    virtual ~AbstractDriver() = default;

    /**
     * @brief Attache un moteur au premier canal disponible.
     * @param motor Moteur à attacher.
     * @return Identifiant du moteur.
     */
    virtual int attach(AbstractMotor* motor) = 0;

    /**
     * @brief Attache un moteur à l'identifiant spécifié.
     * @param motor Moteur à attacher.
     * @param id Identifiant du canal.
     * @return -1 en cas d'échec, sinon l'identifiant du moteur.
     */
    virtual int attach(AbstractMotor* motor, uint8_t id) = 0;

    /**
     * @brief Détache le moteur à cet identifiant.
     * @param id Identifiant du moteur.
     */
    virtual void detach(const int id) {
        m_motors[id] = nullptr;
    }

    /**
     * @brief Fixe la vitesse du moteur à l'identifiant donné.
     * @param velocity Vitesse à appliquer.
     * @param id Identifiant du moteur.
     */
    virtual void setVelocity(float velocity, uint8_t id) = 0;

    /**
     * @brief Retourne la vitesse du moteur à l'identifiant donné.
     * @param id Identifiant du moteur.
     * @return Vitesse courante.
     */
    virtual float getVelocity(uint8_t id) const = 0;

    /**
     * @brief Retourne un pointeur vers le moteur à l'identifiant donné.
     * @param id Identifiant du moteur.
     * @return Pointeur vers le moteur, ou nullptr s'il n'existe pas.
     */
    AbstractMotor* getMotor(const int id) const {
        if (id < 0 || id >= MAX_MOTOR_IN_DRIVER) return nullptr;
        return m_motors[id];
    }
protected:
    AbstractMotor* m_motors[MAX_MOTOR_IN_DRIVER] = {};
};

#endif //MOTORS_ABSTRACTDRIVER_H
