//
// Created by Romain Pouvreau on 15/04/2026.
//
/**
 * @file DRV8844.h
 * @ingroup motors
 * @brief Driver pour le composant DRV8844.
 */

#ifndef MOTORS_DRV8844_H
#define MOTORS_DRV8844_H

#include "mathutils.h"
#include "AbstractDriver.h"
#include <iostream>

#ifdef ARDUINO
#include <Arduino.h>
#elif ESP_PLATFORM

#endif


class DRV8844 : public AbstractDriver
{
public:
    /**
     * @brief Construit le driver DRV8844.
     */
    DRV8844(int m1_en, int m1_dir, int m1_pwm, int m2_en, int m2_dir, int m2_pwm, int fault, int rst);

    /**
     * @brief Initialise les broches matérielles.
     */
    void init();

    /**
     * @brief Attache un moteur au canal disponible.
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @return ID affecté (0) ou -1 en cas d'erreur.
     */
    int attach(AbstractMotor* motor) override;

    /**
     * @brief Attache un moteur à un identifiant spécifique.
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @param id Identifiant du canal.
     * @return ID affecté ou -1 si l'identifiant est invalide (> 0).
     */
    int attach(AbstractMotor* motor, uint8_t id) override;

    /**
     * @brief Définit la vitesse de sortie pour le moteur spécifié.
     * @param velocity Vitesse normalisée entre -1.0 et 1.0.
     * Le signe détermine l'état de la broche DIR.
     * @param id Identifiant du moteur (0).
     */
    void setVelocity(float velocity, uint8_t id) override;

    /**
     * @brief Récupère la dernière vitesse appliquée.
     * @param id Identifiant du moteur (0).
     * @return Vitesse entre -1.0 et 1.0.
     */
    float getVelocity(uint8_t id) const override;

    /**
     * @brief Vérifie l'état de santé du driver.
     * @return true si la broche nFAULT est à l'état BAS (erreur).
     */
    bool isFaulty() const;

    /**
     * @brief Contrôle le mode basse consommation.
     * @param active true pour mettre le driver en sommeil, false pour le réveiller.
     */
    void setSleep(bool active);

private:
    int m_m1_en;
    int m_m1_dir;
    int m_m1_pwm;
    int m_m2_en;
    int m_m2_dir;
    int m_m2_pwm;
    int m_fault;
    int m_rst;

    float m_currentVelocity[2] {}; ///< Stockage interne de la vitesse
};



#endif //MOTORS_DRV8844_H
