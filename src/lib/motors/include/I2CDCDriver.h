//
// Created by Romain Pouvreau on 23/04/2026.
//

#ifndef PAMIS_I2CDCDRIVER_H
#define PAMIS_I2CDCDRIVER_H


#ifdef ARDUINO
//#error "Not implemented"
#elif ESP_PLATFORM
#include "i2cdev.h"
#include "pca9685.h"
#include "mathutils.h"
#endif
#ifndef ARDUINO
#include "mathutils.h"
#include "AbstractDriver.h"

#define PCA9685_HIGH 4096
#define PCA9685_LOW 0

/**
     * @class DRV8876
     * @brief Implémentation du driver pour le circuit intégré DRV8876.
     * * Le DRV8876 est un pont en H simple canal (H-Bridge). Cette classe gère
     * l'interface matérielle directe : PWM pour la vitesse, DIR pour le sens,
     * nSLEEP pour l'économie d'énergie et nFAULT pour le diagnostic.
     */
class I2CDCDriver : public AbstractDriver
{
public:
    /**
     * @brief Constructeur du driver DRV8876.
     * @param scl Pin SCL.
     * @param sda Pin SDA.
     */
    I2CDCDriver(int m1_fwd_port, int m1_bck_port, int m1_pwm, int m2_fwd_port, int m2_bck_port, int m2_pwm, int stdby_port);

    /**
     * @brief Initialise les broches matérielles.
     * Configure les entrées/sorties et réveille le driver si sleepPin est défini.
     */
    void init(i2c_dev_t* dev);

    /**
     * @brief Attache un moteur au canal disponible (ID 0).
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @return L'ID affecté (0) ou -1 en cas d'erreur.
     */
    int attach(AbstractMotor* motor) override;

    /**
     * @brief Attache un moteur à un ID spécifique.
     * @param motor Pointeur vers l'objet AbstractMotor.
     * @param id Identifiant du canal (doit être 0 pour le DRV8876).
     * @return L'ID affecté ou -1 si l'ID est invalide (> 0).
     */
    int attach(AbstractMotor* motor, uint8_t id) override;

    /**
     * @brief Définit la vitesse de sortie pour le moteur spécifié.
     * * @param velocity Vitesse normalisée entre -1.0 et 1.0.
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
     * @return true si la broche nFAULT est à l'état BAS (Erreur).
     */
    bool isFaulty() const;

    /**
     * @brief Contrôle le mode basse consommation.
     * @param active true pour mettre le driver en sommeil, false pour le réveiller.
     */
    void setSleep(bool active);

private:
    int m_m1_fwd_port;
    int m_m1_bck_port;
    int m_m1_pwm;
    int m_m2_fwd_port;
    int m_m2_bck_port;
    int m_m2_pwm;
    int m_stdby_port;
    i2c_dev_t* m_dev = {};
    float m_currentVelocity; ///< Stockage interne de la vitesse
    bool m_lastForward[2];
};

#endif
#endif //PAMIS_I2CDCDRIVER_H