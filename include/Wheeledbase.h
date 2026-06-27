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

#include "DRV8844.h"
#include "Logger.h"

// Parameters identifiers
/**
 * @defgroup wb_global_define Des identifiants
 * @ingroup wheeledbase
 * @ingroup globals
 * @brief Les valeurs des identifiants des constantes
 * @{
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
///@}
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
 * @ingroup wheeledbase
 * @ingroup lib
 *
 * @brief Représente les divers composants de la base roulante
 *
 */
struct WheeledBase {
    DRV8844 driver;
    WheelMotor leftWheel;
    WheelMotor rightWheel;

    Codewheel leftCodewheel;
    Codewheel rightCodewheel;

    Odometry odometry;

    VelocityController velocityControl;
    PositionController positionControl;

    PID linVelPID;
    PID angVelPID;
};

/**
 * @namespace Wheeledbase
 * @ingroup wheeledbase
 * @ingroup libs
 * @brief Fonctions de convénience pour faire les déplacements facilement
 */
namespace  Wheeledbase {
    /// Le logger pour la base roulante
    inline auto wb_logger = Logger("WHEELEDBASE");

    // Instructions prototypes
    void DISABLE(WheeledBase* wheeledBase);

    void GOTO_DELTA(WheeledBase* wheeledBase, PurePursuit* purePursuit, float dx, float dy, bool bloquant=true); //bloquant: do we pass at the next step after ?

    //void TURNTO_DELTA(WheeledBase* wheeledBase, float dtheta, bool bloquant=true);

    //void TENTATIVE_POUR_PLUSTARD();

    void SET_OPENLOOP_VELOCITIES(WheeledBase* wheeledBase, float leftWheelVel, float rightWheelVel);

    void GET_CODEWHEELS_COUNTERS(WheeledBase* wheeledBase, float *leftCodewheelCounter, float *rightCodewheelCounter);

    void SET_VELOCITIES(WheeledBase* wheeledBase, float linVelSetpoint, float angVelSetpoint);

    void RESET_PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit);

    void ADD_PUREPURSUIT_WAYPOINT(PurePursuit* purePursuit, float x, float y);

    void START_PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit, PurePursuit::Direction direction, float finalAngle);

    void START_TURNONTHESPOT(WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, bool dir, float theta);

    void START_TURNONTHESPOT_DIR(WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, TurnOnTheSpot::Direction dir, float theta);

    void PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit, const Position** waypoints, uint16_t nb_waypoints, PurePursuit::Direction dir, float finalAngle);

    void GOTO(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* pos, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE,bool bloquant=true);

    void GOTO_WAYPOINTS(WheeledBase* wheeledBase, PurePursuit* purePursuit, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, int nb_waypoints=1, ...);

    void GOTO_WAYPOINTS_ARRAY(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* positions[], bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, int nb_waypoints=1);

    void GOTO_FUNCT(WheeledBase* wheeledBase, PurePursuit* purePursuit, TurnOnTheSpot* turnOnTheSpot, Position* pos, void* duringMovingFunct, void* approachFunct, bool alignFirst=true, PurePursuit::Direction dir=PurePursuit::NONE, float finalAngle=MAXFLOAT);

    uint8_t POSITION_REACHED(WheeledBase* wheeledBase);

    void GET_VELOCITIES_WANTED(const WheeledBase* wheeledBase, float *linOutput, float *angOutput, bool spin=true);

    void SET_POSITION(WheeledBase* wheeledBase, const Position* pos);

    const Position* GET_POSITION(const WheeledBase* wheeledBase);

    void GET_VELOCITIES(const WheeledBase* wheeledBase, float *linVel, float *angVel);

    void SET_PARAMETER_VALUE(WheeledBase* wheeledBase, PurePursuit* purePursuit, WBParameterID paramID, float value);

    float GET_PARAMETER_VALUE(WheeledBase* wheeledBase, const PurePursuit* purePursuit, WBParameterID paramID);

    void PRINT_PARAMS(WheeledBase* wheeledBase, const PurePursuit* purePursuit);
};



#endif //WHEELEDBASE_H
