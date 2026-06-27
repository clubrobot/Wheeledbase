//
// Created by boris on 28/10/2024.
//
/**
 * @file Wheeledbase.h
 * @ingroup wheeledbase
 * @ingroup lib
 * @brief Fonctions de convénience pour faire les déplacements facilement
 */
#ifndef WHEELEDBASE_H
#define WHEELEDBASE_H

#include <functional>

#include "WheelMotor.h"
#include "Codewheel.h"
#include "Odometry.h"
#include "PID.h"
#include "VelocityController.h"
#include "PositionController.h"
#include "PurePursuit.h"
#include "TurnOnTheSpot.h"
#include "mathutils.h"
#include <math.h>
#include <memory>

#include "DRV8844.h"
#include "Logger.h"

// Parameters identifiers
/**
 * @defgroup wb_slowdown_define Paramètres de ralentissement du robot
 * @ingroup wheeledbase
 * @ingroup globals
 * @brief Les paramètres de ralentissement du robot à la fin de goto
 * @{
 */
#define SLOWDOWN_FACTOR                 0.5
#define SLOWDOWN_DISTANCE               0
#define ALIGN_DISTANCE                  130
///@}

/**
 * @namespace Wheeledbase
 * @ingroup wheeledbase
 * @ingroup libs
 * @brief Fonctions de convénience pour faire les déplacements facilement
 */
namespace  Wheeledbase {

    /**
     * @struct WBConstants
     *
     * @brief Défini un certain nombre de constantes et paramètres pour la wheeledbase
     */
    struct WBConstants
    {
        const float LEFTWHEEL_RADIUS;
        const float LEFTWHEEL_CONSTANT;
        const float LEFTWHEEL_MAXPWM;
        const float RIGHTWHEEL_RADIUS;
        const float RIGHTWHEEL_CONSTANT;
        const float RIGHTWHEEL_MAXPWM;
        const long LEFTCODEWHEEL_COUNTSPERREV;
        const long RIGHTCODEWHEEL_COUNTSPERREV;
        const float LEFTCODEWHEEL_RADIUS;
        const float RIGHTCODEWHEEL_RADIUS;

        const float ODOMETRY_AXLETRACK;
        const float ODOMETRY_SLIPPAGE;

        const float VELOCITYCONTROL_AXLETRACK;
        const float VELOCITYCONTROL_MAXLINACC;
        const float VELOCITYCONTROL_MAXLINDEC;
        const float VELOCITYCONTROL_MAXANGACC;
        const float VELOCITYCONTROL_MAXANGDEC;
        const bool VELOCITYCONTROL_SPINSHUTDOWN;

        /*---- LINEAR VELOCITIES PID  ----*/
        const float LINVELPID_KP;
        const float LINVELPID_KI;
        const float LINVELPID_KD;

        const float LINVELPID_MINOUTPUT;
        const float LINVELPID_MAXOUTPUT;

        /*---- ANGULAR VELOCITIES PID ----*/
        const float ANGVELPID_KP;
        const float ANGVELPID_KI;
        const float ANGVELPID_KD;

        const float ANGVELPID_MINOUTPUT;
        const float ANGVELPID_MAXOUTPUT;

        /*---- POSITION PID ----*/
        const float POSITIONCONTROL_LINVELKP;
        const float POSITIONCONTROL_ANGVELKP;

        const float POSITIONCONTROL_LINVELMAX;
        const float POSITIONCONTROL_ANGVELMAX;
        const float POSITIONCONTROL_LINPOSTHRESHOLD;
        const float POSITIONCONTROL_ANGPOSTHRESHOLD;
        const float PUREPURSUIT_LOOKAHEAD;
        const float PUREPURSUIT_LOOKAHEADBIS;

        // Timesteps
        const float ODOMETRY_TIMESTEP;
        const float PID_CONTROLLERS_TIMESTEP;
        const float POSITIONCONTROL_TIMESTEP;

        // DC motors driver
        const int LEFT_MOTOR_EN;
        const int LEFT_MOTOR_PWM;
        const int LEFT_MOTOR_DIR;
        const int RIGHT_MOTOR_EN;
        const int RIGHT_MOTOR_PWM;
        const int RIGHT_MOTOR_DIR;
        const int DRIVER_RESET;
        const int DRIVER_FAULT;
    };
    /**
    * @struct WheeledBase
    *
    * @brief Représente les divers composants de la base roulante
    *
    */
    struct WheeledBase {
        std::unique_ptr<DRV8844> driver;
        std::unique_ptr<WheelMotor> leftWheel;
        std::unique_ptr<WheelMotor> rightWheel;

        std::unique_ptr<Codewheel> leftCodewheel;
        std::unique_ptr<Codewheel> rightCodewheel;

        std::unique_ptr<Odometry> odometry;

        std::unique_ptr<VelocityController> velocityControl;
        std::unique_ptr<PositionController> positionControl;

        std::unique_ptr<PID> linVelPID;
        std::unique_ptr<PID> angVelPID;

        /**
         * Initialise la base roulante
         * @param driver Un pointeur vers le driver
         * @param leftWheel Un pointeur la roue gauche
         * @param rightWheel Un pointeur vers la roue droite
         * @param leftCodewheel Un pointeur vers la roue codeuse gauche
         * @param rightCodewheel Un pointeur vers la roue codeuse droite
         * @param odometry Un pointeur vers l'odometrie
         * @param velocityControl Un pointeur vers le controleur de vitesse
         * @param positionControl Un pointeur vers le controleur de position
         * @param linVelPID Un pointeur vers le PID de vitesse linéaire
         * @param angVelPID Un pointeur vers le PID de vitesse angulaire
         * @param param Un pointeur vers la structure de configuration
         * @param halSetup Un pointeur vers une fonction qui initialise la hal
         */
        WheeledBase(
            std::unique_ptr<DRV8844> driver,
            std::unique_ptr<WheelMotor> leftWheel,
            std::unique_ptr<WheelMotor> rightWheel,
            std::unique_ptr<Codewheel> leftCodewheel,
            std::unique_ptr<Codewheel> rightCodewheel,
            std::unique_ptr<Odometry> odometry,
            std::unique_ptr<VelocityController> velocityControl,
            std::unique_ptr<PositionController> positionControl,
            std::unique_ptr<PID> linVelPID,
            std::unique_ptr<PID> angVelPID,
            WBConstants& param,
            const std::function<void()>& halSetup
            );
    };

    [[noreturn]] void run(WheeledBase* wheeledBase, const std::function<void()>& delayFunc);

    /**
     * @enum WBParameterID
     * @brief Des identifiants permettant d'accéder à la plupart des paramètres de la base roulante.
     */
    enum WBParameterID : uint8_t {
        WBPID_LEFTWHEEL_RADIUS_ID,
        WBPID_LEFTWHEEL_CONSTANT_ID,
        WBPID_LEFTWHEEL_MAXPWM_ID,
        WBPID_RIGHTWHEEL_RADIUS_ID,
        WBPID_RIGHTWHEEL_CONSTANT_ID,
        WBPID_RIGHTWHEEL_MAXPWM_ID,
        WBPID_LEFTCODEWHEEL_RADIUS_ID,
        WBPID_LEFTCODEWHEEL_COUNTSPERREV_ID,
        WBPID_RIGHTCODEWHEEL_RADIUS_ID,
        WBPID_RIGHTCODEWHEEL_COUNTSPERREV_ID,
        WBPID_ODOMETRY_AXLETRACK_ID,
        WBPID_ODOMETRY_SLIPPAGE_ID,
        WBPID_VELOCITYCONTROL_AXLETRACK_ID,
        WBPID_VELOCITYCONTROL_MAXLINACC_ID,
        WBPID_VELOCITYCONTROL_MAXLINDEC_ID,
        WBPID_VELOCITYCONTROL_MAXANGACC_ID,
        WBPID_VELOCITYCONTROL_MAXANGDEC_ID,
        WBPID_VELOCITYCONTROL_SPINSHUTDOWN_ID,
        WBPID_LINVELPID_KP_ID,
        WBPID_LINVELPID_KI_ID,
        WBPID_LINVELPID_KD_ID,
        WBPID_LINVELPID_MINOUTPUT_ID,
        WBPID_LINVELPID_MAXOUTPUT_ID,
        WBPID_ANGVELPID_KP_ID,
        WBPID_ANGVELPID_KI_ID,
        WBPID_ANGVELPID_KD_ID,
        WBPID_ANGVELPID_MINOUTPUT_ID,
        WBPID_ANGVELPID_MAXOUTPUT_ID,
        WBPID_POSITIONCONTROL_LINVELKP_ID,
        WBPID_POSITIONCONTROL_ANGVELKP_ID,
        WBPID_POSITIONCONTROL_LINVELMAX_ID,
        WBPID_POSITIONCONTROL_ANGVELMAX_ID,
        WBPID_POSITIONCONTROL_LINPOSTHRESHOLD_ID,
        WBPID_POSITIONCONTROL_ANGPOSTHRESHOLD_ID,
        WBPID_PUREPURSUIT_LOOKAHED_ID,
        WBPID_PUREPURSUIT_LOOKAHEADBIS_ID
    };
    /// Le logger pour la base roulante
    inline auto wb_logger = Logger("WHEELEDBASE");

    // Instructions prototypes
    /**
     * @brief Désactive la base roulante
     * @param wheeledBase La base roulante
     */
    void disable(const WheeledBase* wheeledBase);

    /**
     * @brief Déplace la base roulante d'une certaine distance dans le référentiel du robot
     * @param wheeledBase La base roulante
     * @param purePursuit Le contrôleur de poursuite pure
     * @param dx La distance à parcourir sur l'axe des x
     * @param dy La distance à parcourir sur l'axe des y
     * @param bloquant Indique si l'opération doit être bloquante
     */
    void gotoDelta(WheeledBase* wheeledBase, PurePursuit* purePursuit, float dx, float dy, bool bloquant=true); //bloquant: do we pass at the next step after ?

    //void TURNTO_DELTA(WheeledBase* wheeledBase, float dtheta, bool bloquant=true);

    //void TENTATIVE_POUR_PLUSTARD();

    /**
     * @brief Définit les vitesses en boucle ouverte pour les roues
     * @param wheeledBase La base roulante
     * @param leftWheelVel La vitesse de la roue gauche
     * @param rightWheelVel La vitesse de la roue droite
     */
    void setOpenloopVelocities(WheeledBase* wheeledBase, float leftWheelVel, float rightWheelVel);

    /**
     * @brief Récupère les compteurs des roues codeuses
     * @param wheeledBase La base roulante
     * @param leftCodewheelCounter Le pointeur vers le compteur de la roue codeuse gauche
     * @param rightCodewheelCounter Le pointeur vers le compteur de la roue codeuse droite
     */
    void getCodewheelsCounters(WheeledBase* wheeledBase, float *leftCodewheelCounter, float *rightCodewheelCounter);

    /**
     * @brief Fixe une vitesse linéaire et angulaire pour la base roulante
     * @param wheeledBase La base roulante
     * @param linVelSetpoint La consigne de vitesse linéaire
     * @param angVelSetpoint La consigne de vitesse angulaire
     */
    void setVelocities(WheeledBase* wheeledBase, float linVelSetpoint, float angVelSetpoint);

    /**
     * @brief Réinitialise le purepursuit
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit à réinitialiser
     */
    void resetPurepursuit(WheeledBase* wheeledBase, PurePursuit* purePursuit);

    /**
     * @brief Ajoute un waypoint au purepursuit
     * @param purePursuit Le purpursuit
     * @param x la position x du waypoint
     * @param y la position y du waypoint
     */
    void addPurepursuitWaypoint(PurePursuit* purePursuit, float x, float y);

    /**
     * @brief Démarre le purepursuit
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param direction La direction du mouvement
     * @param finalAngle L'angle final
     */
    void startPurepursuit(WheeledBase* wheeledBase, PurePursuit* purePursuit, PurePursuit::Direction direction, float finalAngle);

    /**
     * @brief démarre turn on the spot
     * @param wheeledBase La base roulante
     * @param turnOnTheSpot Le contrôleur turn on the spot
     * @param dir La direction du tour
     * @param theta L'angle du tour
     */
    void startTurnonthespot(WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, bool dir, float theta);

    /**
     * @brief démarre turn on the spot mais avec moins de magie dans la direction de rotation
     * @param wheeledBase La base roulante
     * @param turnOnTheSpot Le contrôleur turn on the spot
     * @param dir La direction du tour
     * @param theta L'angle du tour
     */
    void startTurnonthespotDir(const WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, TurnOnTheSpot::Direction dir, float theta);

    /**
     * @brief Démarre purepursuit ? honnêtement jsp trop
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param waypoints Les points de passage
     * @param nb_waypoints Le nombre de points de passage
     * @param dir La direction du mouvement
     * @param finalAngle L'angle final
     */
    void purepursuit(WheeledBase* wheeledBase, PurePursuit* purePursuit, const Position** waypoints, uint16_t nb_waypoints, PurePursuit::Direction dir, float finalAngle);

    /**
     * @brief Va à un point en utilisant purepursuit
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param pos La destination
     * @param alignFirst On s'aligne d'abord ?
     * @param dir La direction
     * @param bloquant Est-ce que la fonction doit bloquer jusqu'à la fin du mouvement
     */
    void wbGoto(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* pos, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE,bool bloquant=true);

    /**
     * @brief Suit une liste de waypoint en utilisant purepursuit
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param alignFirst On s'aligne d'abord ?
     * @param dir La direction
     * @param nb_waypoints Le nombre de points de passage
     * @param ... Les points de passage
     */
    void gotoWaypoints(WheeledBase* wheeledBase, PurePursuit* purePursuit, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, int nb_waypoints=1, ...);

    /**
     * @brief Suit une liste de waypoint en utilisant purepursuit mais sans les varargs
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param positions Les waypoints
     * @param alignFirst On s'aligne d'abord ?
     * @param dir La direction
     * @param nb_waypoints Le nombre de points de passage
     */
    void gotoWaypointsArray(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* positions[], bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, int nb_waypoints=1);

    /// Honnêtement aucune idée de ce que fait cette fonction
    void gotoFunct(WheeledBase* wheeledBase, PurePursuit* purePursuit, TurnOnTheSpot* turnOnTheSpot, Position* pos, void* duringMovingFunct, void* approachFunct, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, float finalAngle=MAXFLOAT);

    /**
     * @brief Est-ce que la position est atteinte ?
     * @param wheeledBase La base roulante
     * @return 0b01 si la position est atteinte, 0b10 si les roues patinent (en théorie, sauf que le code de détection de patinage est désactivé...)
     */
    uint8_t positionReached(const WheeledBase* wheeledBase);

    /**
     * @brief Récupère les vitesses linéaire et angulaire souhaitées
     * @param wheeledBase La base roulante
     * @param linOutput La variable dans laquelle la vitesse linéaire sera retournée
     * @param angOutput La variable dans laquelle la vitesse angulaire sera retournée
     * @param spin Est-ce que les roues patinent ?
     */
    void getVelocitiesWanted(const WheeledBase* wheeledBase, float *linOutput, float *angOutput, bool spin=true);

    /**
     * @brief Met à jour la position de l'odométrie
     * @param wheeledBase La base roulante
     * @param pos La nouvelle position
     */
    void setPosition(WheeledBase* wheeledBase, const Position* pos);

    /**
     * @brief Récupère la position de l'odométrie
     * @param wheeledBase La base roulante
     * @return La position de l'odométrie
     */
    const Position* getPosition(const WheeledBase* wheeledBase);

    /**
     * @brief Récupère les vitesses linéaire et angulaire actuelles
     * @param wheeledBase La base roulante
     * @param linVel La variable dans laquelle la vitesse linéaire sera retournée
     * @param angVel La variable dans laquelle la vitesse angulaire sera retournée
     */
    void getVelocities(const WheeledBase* wheeledBase, float *linVel, float *angVel);

    /**
     * @brief Met un paramètre de la base roulante à jour
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param paramID L'ID du paramètre
     * @param value La nouvelle valeur
     */
    void setParameterValue(WheeledBase* wheeledBase, PurePursuit* purePursuit, WBParameterID paramID, float value);

    /**
     * @brief Récupère la valeur d'un paramètre de la base roulante
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     * @param paramID L'ID du paramètre
     * @return La valeur du paramètre
     */
    float getParameterValue(const WheeledBase* wheeledBase, const PurePursuit* purePursuit, WBParameterID paramID);

    /**
     * @brief Affiche les paramètres de la base roulante
     * @param wheeledBase La base roulante
     * @param purePursuit Le purepursuit
     */
    void printParams(const WheeledBase* wheeledBase, const PurePursuit* purePursuit);
};



#endif //WHEELEDBASE_H
