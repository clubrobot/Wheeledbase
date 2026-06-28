//
// Created by Romain Pouvreau on 15/04/2026.
//
/**
 * @file DRV8876.h
 * @ingroup motors
 * @brief Driver pour le composant DRV8876.
 */

#ifndef MOTORS_DRV8876_H
#define MOTORS_DRV8876_H

#ifdef ARDUINO
#include <Arduino.h>
#elif ESP_PLATFORM
//#error "Not implemented"
#endif

#include <cmath>

#include "AbstractDriver.h"
/**
     * @class DRV8876
     * @brief Implémentation du driver pour le circuit intégré DRV8876.
     * * Le DRV8876 est un pont en H simple canal (H-Bridge). Cette classe gère
     * l'interface matérielle directe : PWM pour la vitesse, DIR pour le sens,
     * nSLEEP pour l'économie d'énergie et nFAULT pour le diagnostic.
     */
class DRV8876 : public AbstractDriver
{
public:
    /**
     * @brief Construit le driver DRV8876.
     * @param enPin Broche supportant le PWM (Contrôle de vitesse).
     * @param phPin Broche numérique (Contrôle de direction).
     * @param sleepPin Broche nSLEEP (HIGH: actif, LOW: sommeil). Par défaut -1 (non utilisé).
     * @param faultPin Broche nFAULT (LOW: erreur détectée). Par défaut -1 (non utilisé).
     */
    DRV8876(int enPin, int phPin, int sleepPin = -1, int faultPin = -1);

    /**
     * @brief Initialise les broches matérielles.
     * Configure les entrées/sorties et réveille le driver si sleepPin est défini.
     */
    void init();

    /**
     * @brief Attache un moteur au canal disponible (ID 0).
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @return ID affecté (0) ou -1 en cas d'erreur.
     */
    int attach(AbstractMotor* motor) override;

    /**
     * @brief Attache un moteur à un identifiant spécifique.
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @param id Identifiant du canal (doit être 0 pour le DRV8876).
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
    int m_enPin; ///< Pin PWM (Vitesse)
    int m_phPin; ///< Pin DIR (Direction)
    int m_sleepPin; ///< Pin nSLEEP
    int m_faultPin; ///< Pin nFAULT
    float m_currentVelocity; ///< Stockage interne de la vitesse
};
#endif //MOTORS_DRV8876_H
