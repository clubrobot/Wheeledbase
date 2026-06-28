/**
 * @file WheelMotor.h
 * @ingroup motors
 * @brief Implémentation d'un moteur de roue.
 */

#ifndef MOTORS_WHEELMOTOR_H
#define MOTORS_WHEELMOTOR_H

#include "AbstractMotors.h"

/**
 * @brief Implémentation concrète d'un moteur de roue.
 *
 * Cette classe permet de définir les limites physiques d'un moteur
 * et de le lier à un canal spécifique d'un AbstractDriver.
 */
class WheelMotor : public AbstractMotor
{
public:
    /**
     * @brief Construit un moteur de roue.
     * @param maxVelocity Vitesse maximale autorisée (généralement 1.0f pour 100%).
     */
    explicit WheelMotor(float maxVelocity = 1.0f);

    /**
     * @brief Retourne la vitesse maximale configurée.
     * @return Vitesse maximale.
     */
    [[nodiscard]] float getMaxVelocity() const override;

    /**
     * @brief Modifie la vitesse maximale (limitation logicielle).
     * @param maxVelocity Nouvelle vitesse maximale.
     */
    void setMaxVelocity(float maxVelocity);

    //! Renvoie la constante du moteur paramétrée.
    /*!
     * \return Constante : (60 * reduction_ratio / velocity_constant_in_RPM) / supplied_voltage_in_V
     */
    float getConstant() const { return m_constant; }

    //! Renvoie le rayon de la roue du moteur.
    /*!
     * \return Rayon en mm.
     */
    float getWheelRadius() const { return m_wheelRadius; }

    //! Paramètre le rayon de la roue liée au moteur.
    /*!
     * \param wheelRadius Rayon en mm.
     */
    void setWheelRadius(float wheelRadius);

    //! Envoie une commande de vitesse au moteur.
    /*!
     * \param velocity Vitesse de commande en mm/s.
     */
    void setVelocity(float velocity) override;


    //! Paramètre la constante du moteur.
    /*!
     * \param constant Constante en rad/s/Volt.
     */
    void setConstant(float constant);

private:
    float m_maxVelocity; ///< Limite de vitesse pour ce moteur.
    float m_wheelRadius; //!<  in mm
    float m_constant; //!<  (60 * reduction_ratio / velocity_constant_in_RPM) / supplied_voltage_in_V
};


#endif //MOTORS_WHEELMOTOR_H
