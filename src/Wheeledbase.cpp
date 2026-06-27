// Created by boris on 28/10/2024.
//

#include "Wheeledbase.h"
#include "BasicTurnStrategy.h"
#include "BasicMoveStrategy.h"
// Instructions


void Wheeledbase::DISABLE(WheeledBase* wheeledBase) {
    wheeledBase->velocityControl.disable();
    wheeledBase->positionControl.disable();
    wheeledBase->leftWheel.setVelocity(0);
    wheeledBase->rightWheel.setVelocity(0);
}

void Wheeledbase::GOTO_DELTA(WheeledBase* wheeledBase, PurePursuit* purePursuit, float dx, float dy, bool bloquant) {
    purePursuit->reset();
    wheeledBase->positionControl.disable();

    Position initial_pos = *wheeledBase->odometry.getPosition();

    Position target_pos;
    target_pos.x = initial_pos.x + dx * cos(initial_pos.theta) + dy * sin(initial_pos.theta);
    target_pos.y = initial_pos.y + dx * sin(initial_pos.theta) + dy * cos(initial_pos.theta);

    target_pos.theta = atan2(target_pos.y - initial_pos.y, target_pos.x - initial_pos.x);
    int direction;

    initial_pos.theta = inrange(initial_pos.theta, -M_PI,M_PI);

    if (fabs(inrange(target_pos.theta - initial_pos.theta, -M_PI,M_PI)) < (M_PI / 2)) {
        direction = PurePursuit::FORWARD;
    }
    else {
        direction = PurePursuit::BACKWARD;
    }

    purePursuit->setDirection(static_cast<PurePursuit::Direction>(direction));
    purePursuit->addWaypoint(PurePursuit::Waypoint(initial_pos.x, initial_pos.y));
    purePursuit->addWaypoint(PurePursuit::Waypoint(target_pos.x, target_pos.y));

    purePursuit->setFinalAngle(target_pos.theta);

    wheeledBase->positionControl.setPosSetpoint(Position(target_pos.x, target_pos.y, target_pos.theta + direction * M_PI));

    // Enable PurePursuit controller
    wheeledBase->velocityControl.enable();
    wheeledBase->positionControl.setMoveStrategy(*purePursuit);
    wheeledBase->positionControl.enable();

    while(!(POSITION_REACHED(wheeledBase) & 0b01) && bloquant) {
        //Wait I guess
    }
}
/*
void Wheeledbase::TURNTO_DELTA(float dtheta, bool bloquant){
    velocityControl.disable();
    positionControl.setPosThresholds(3, 1e-3); //> 1°
    basicTurn.ang_pid.setTunings(.45, 0.0, 0.2); // 0.45 0 0.5
    basicTurn.lin_pid.setTunings(.0, 0, 0);
    positionControl.setVelLimits(40, PI/2);

    Position initial_pos = *odometry.getPosition();

    Position target_pos;
    target_pos.x = initial_pos.x;
    target_pos.y = initial_pos.y;
    target_pos.theta = initial_pos.theta + dtheta;

    positionControl.setMoveStrategy(basicTurn);

    positionControl.setPosSetpoint(target_pos);
    velocityControl.enable();
    positionControl.enable();

    while(!(Wheeledbase::POSITION_REACHED() & 0b01) && bloquant) {
        //Wait I guess
    }
    //printf("initial_pos: %f %f %f\n", initial_pos.x, initial_pos.y, initial_pos.theta);
    //printf("target_pos: %f %f %f\n", target_pos.x, target_pos.y, target_pos.theta);
    //printf("L'objectif a été atteint, WheeledBase::TURNTO_DELTA est ok\n");
}*/

/*void Wheeledbase::TENTATIVE_POUR_PLUSTARD() {
    velocityControl.disable();
    positionControl.disable();

    Position posSetpoint = *odometry.getPosition();

    Position target_pos;
    target_pos.x = posSetpoint.x;
    target_pos.y = posSetpoint.y;
    target_pos.theta = posSetpoint.theta + dtheta;

    ang_precision = 0.0;
    ang_max_speed = 2;
    ang_slowing_distance = 0.5;

    float dang = getPosSetpoint().theta - getPosInput().theta;
    float obj;
    if (dang > 0 && dang > ang_slowing_distance) {
        obj = ang_max_speed;
        printf("++++\n");
    } else if (dang < 0 && fabs(dang) > ang_slowing_distance) {
        obj = -ang_max_speed;
        printf("----\n");
    } else {
        obj = ang_max_speed * fabs(dang) / ang_slowing_distance;
        obj = (dang > 0) ? obj : -obj;
        printf("OKKKKKKKKKKKKEY\n");
    }
    leftWheel.setVelocity(obj/2);
    rightWheel.setVelocity(-obj/2);
}*/


void Wheeledbase::SET_OPENLOOP_VELOCITIES(WheeledBase* wheeledBase, float leftWheelVel, float rightWheelVel) {
    wheeledBase->velocityControl.disable();
    wheeledBase->positionControl.disable();
    wheeledBase->leftWheel.setVelocity(leftWheelVel);
    wheeledBase->rightWheel.setVelocity(rightWheelVel);
}

void Wheeledbase::GET_CODEWHEELS_COUNTERS(WheeledBase* wheeledBase, float* leftCodewheelCounter, float* rightCodewheelCounter) {
    *leftCodewheelCounter = wheeledBase->leftCodewheel.getCounter();
    *rightCodewheelCounter = wheeledBase->rightCodewheel.getCounter();
}

void Wheeledbase::SET_VELOCITIES(WheeledBase* wheeledBase, float linVelSetpoint, float angVelSetpoint) {
    wheeledBase->positionControl.disable();
    wheeledBase->velocityControl.enable();
    wheeledBase->velocityControl.setSetpoints(linVelSetpoint, angVelSetpoint);
}

void Wheeledbase::RESET_PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit) {
    purePursuit->reset();
    wheeledBase->positionControl.disable();
}

void Wheeledbase::START_PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit, PurePursuit::Direction direction, float finalAngle) {
    // Setup PurePursuit
    purePursuit->setFinalAngle(finalAngle);
    auto angle = static_cast<double>(direction);
    switch (direction) {
        default:
        case PurePursuit::FORWARD:
            purePursuit->setDirection(PurePursuit::FORWARD);
            break;
        case PurePursuit::BACKWARD:
            purePursuit->setDirection(PurePursuit::BACKWARD);
            angle = angle - M_PI;
            break;
    }

    // Compute final setpoint
    const PurePursuit::Waypoint wp0 = purePursuit->getWaypoint(purePursuit->getNumWaypoints() - 2);
    const PurePursuit::Waypoint wp1 = purePursuit->getWaypoint(purePursuit->getNumWaypoints() - 1);
    wheeledBase->positionControl.setPosSetpoint(Position(wp1.x, wp1.y, atan2(wp1.y - wp0.y, wp1.x - wp0.x) + angle * M_PI));

    // Enable PurePursuit controller
    wheeledBase->velocityControl.enable();
    wheeledBase->positionControl.setMoveStrategy(*purePursuit);
    wheeledBase->positionControl.enable();
}

void Wheeledbase::ADD_PUREPURSUIT_WAYPOINT(PurePursuit* purePursuit, float x, float y) {
    // Queue waypoint
    purePursuit->addWaypoint(PurePursuit::Waypoint(x, y));
}

void Wheeledbase::START_TURNONTHESPOT(WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, bool dir, float theta) {
    Position posSetpoint = *wheeledBase->odometry.getPosition();
    float initTheta = posSetpoint.theta;
    posSetpoint.theta = theta;
    float angPosSetpoint = inrange((posSetpoint.theta - initTheta), -M_PI, M_PI);
    wheeledBase->velocityControl.enable();
    wheeledBase->positionControl.setPosSetpoint(posSetpoint);
    if (dir) {
        if (angPosSetpoint > 0) turnOnTheSpot->setDirection(TurnOnTheSpot::CLOCK);
        else turnOnTheSpot->setDirection(TurnOnTheSpot::TRIG);
    }
    else {
        if (angPosSetpoint > 0) turnOnTheSpot->setDirection(TurnOnTheSpot::TRIG);
        else turnOnTheSpot->setDirection(TurnOnTheSpot::CLOCK);
    }
    wheeledBase->positionControl.setMoveStrategy(*turnOnTheSpot);
    wheeledBase->positionControl.enable();
}

void Wheeledbase::START_TURNONTHESPOT_DIR(WheeledBase* wheeledBase, TurnOnTheSpot* turnOnTheSpot, TurnOnTheSpot::Direction dir, float theta) {
    Position posSetpoint = *wheeledBase->odometry.getPosition();
    posSetpoint.theta = theta;
    wheeledBase->velocityControl.enable();
    wheeledBase->positionControl.setPosSetpoint(posSetpoint);
    turnOnTheSpot->setDirection(dir);
    wheeledBase->positionControl.setMoveStrategy(*turnOnTheSpot);
    wheeledBase->positionControl.enable();
}

/**
* ret:
0b00 si pas reached pas spin urgency (0)
0b01 si reached (1)
0b10 si spin urgency (2)
0b11 si les deux (3)
*/
uint8_t Wheeledbase::POSITION_REACHED(WheeledBase* wheeledBase) {
    bool positionReached = wheeledBase->positionControl.getPositionReached() && wheeledBase->positionControl.isEnabled();
    bool spinUrgency = false;//!wheeledBase->velocityControl.isEnabled();
    uint8_t ret = 0;
    ret = ret | positionReached;
    ret = ret | (spinUrgency << 1);
    return ret;
}

void Wheeledbase::PUREPURSUIT(WheeledBase* wheeledBase, PurePursuit* purePursuit, const Position** waypoints, uint16_t nb_waypoints, PurePursuit::Direction dir, float finalAngle) {
    if(nb_waypoints<2) {
        wb_logger.log(WARNING_LEVEL, "not enough waypoints\n");
        return;
    }
    RESET_PUREPURSUIT(wheeledBase, purePursuit);
    for(int i = 0; i < nb_waypoints; i++) {
        ADD_PUREPURSUIT_WAYPOINT(purePursuit, waypoints[i]->x, waypoints[i]->y);
    }
    START_PUREPURSUIT(wheeledBase, purePursuit, dir, finalAngle);
}

void Wheeledbase::GOTO(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* pos, const bool alignFirst, PurePursuit::Direction dir, const bool bloquant) {
    const float defaultMaxSpeed = GET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID);
    wb_logger.log(INFO_LEVEL, "GOTO called with pos: %f %f %f, alignFirst: %d, dir: %d\n", pos->x, pos->y, pos->theta, alignFirst, dir);

    const Position *myPos = GET_POSITION(wheeledBase);
    if (dir==PurePursuit::NONE) {
        if(cos(atan2(pos->y-myPos->y, pos->x-myPos->x)-myPos->theta)>=0) {
            dir=PurePursuit::FORWARD;
        }else {
            dir=PurePursuit::BACKWARD;

        }
    }
    const Position *posTab[2]={myPos, pos};

    double maxSpeed = defaultMaxSpeed;

    if (alignFirst){
        //On avance vers un point avant le point d'arrivé pour s'aligner et après on avance tout droit
        const float theta = pos->theta;
        constexpr float radius = ALIGN_DISTANCE;

        const float x = pos->x - radius*cos(theta);
        const float y = pos->y - radius*sin(theta);

        wb_logger.log(INFO_LEVEL, "align point : %f %f\n", x, y);

        const Position appr_pos = Position(x,y,pos->theta);
        const Position *posApprTab[2]={myPos, &appr_pos};

        PUREPURSUIT(wheeledBase, purePursuit, posApprTab, 2, dir, pos->theta);

        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", appr_pos.x, appr_pos.y, appr_pos.theta, posi->x, posi->y, posi->theta);
        }

        maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);

        PUREPURSUIT(wheeledBase, purePursuit, posTab, 2, dir, pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01) && bloquant) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);
    }else{
        //On avance dans une ligne droite et on ralenti à la fin
        PUREPURSUIT(wheeledBase, purePursuit, posTab, 2, dir, pos->theta);

        while(!(POSITION_REACHED(wheeledBase) & 0b01) && bloquant) {

             const Position *posi = GET_POSITION(wheeledBase);
            int distance = sqrt(pow(posi->x-pos->x, 2)+pow(posi->y-pos->y, 2));
            if (distance<SLOWDOWN_DISTANCE){
                maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
                SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);
            }

            //printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);
            //Do nothing
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);
    }

}

void Wheeledbase::GOTO_WAYPOINTS(WheeledBase* wheeledBase, PurePursuit* purePursuit, bool alignFirst, PurePursuit::Direction dir, int nb_waypoints, ...){
    /////INIT VALEURS

    const auto **posTab = static_cast<const Position**>(malloc(sizeof(Position*) * nb_waypoints));
    const Position* myPos = GET_POSITION(wheeledBase);

    int way_index = 0;
    posTab[0] = myPos;

    Position* last_pos = new Position(0,0,0);
    Position* appr_pos = new Position(0,0,0);

    float defaultMaxSpeed = GET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID);
    double maxSpeed = defaultMaxSpeed;

    va_list args;
    va_start(args, nb_waypoints);
    nb_waypoints++;
    /////POUR TOUT LES WAYPOINTS
    for (way_index = 1; way_index < nb_waypoints; way_index++){
        Position* curr_position = va_arg(args, Position*);
        /////ON REGARDE SI ON EST PAS A LA FIN
        if (way_index==nb_waypoints-1){
            ///ON ENREGISTRE LA DERNIERE POS
            last_pos=curr_position;
            //CALC APPROACH
            if (alignFirst){
                float theta = PI-last_pos->theta;
                float radius = ALIGN_DISTANCE;

                float x = last_pos->x + radius*cos(theta);
                float y = last_pos->y - radius*sin(theta);
                appr_pos->x = x;
                appr_pos->y = y;
                appr_pos->theta = last_pos->theta;
            }
        }

        if (way_index==nb_waypoints-1 && alignFirst){posTab[way_index] = appr_pos;}
        else{posTab[way_index] = curr_position;}
    }

    if (alignFirst){
        PUREPURSUIT(wheeledBase, purePursuit, posTab, nb_waypoints, dir, appr_pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", appr_pos.x, appr_pos.y, appr_pos.theta, posi->x, posi->y, posi->theta);
        }

        maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);

        const Position *finalPosTab[2]={myPos, last_pos};
        PUREPURSUIT(wheeledBase, purePursuit, finalPosTab, 2, dir, last_pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);

    }else{
        PUREPURSUIT(wheeledBase, purePursuit, posTab, nb_waypoints, dir, last_pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", appr_pos.x, appr_pos.y, appr_pos.theta, posi->x, posi->y, posi->theta);
        }
    }

    free(posTab);
}

void Wheeledbase::GOTO_WAYPOINTS_ARRAY(WheeledBase* wheeledBase, PurePursuit* purePursuit, Position* positions[], bool alignFirst, PurePursuit::Direction dir, int nb_waypoints){
    /////INIT VALEURS

    const auto **posTab = static_cast<const Position**>(malloc(sizeof(Position*) * nb_waypoints));
    const Position* myPos = GET_POSITION(wheeledBase);

    int way_index = 0;
    posTab[0] = myPos;

    Position* last_pos = new Position(0,0,0);
    Position* appr_pos = new Position(0,0,0);

    float defaultMaxSpeed = GET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID);
    double maxSpeed = defaultMaxSpeed;

    nb_waypoints++;
    /////POUR TOUT LES WAYPOINTS
    for (way_index = 1; way_index < nb_waypoints; way_index++){
        Position* curr_position = positions[way_index-1];
        /////ON REGARDE SI ON EST PAS A LA FIN
        if (way_index==nb_waypoints-1){
            ///ON ENREGISTRE LA DERNIERE POS
            last_pos=curr_position;
            //CALC APPROACH
            if (alignFirst){
                float theta = PI-last_pos->theta;
                float radius = ALIGN_DISTANCE;

                float x = last_pos->x + radius*cos(theta);
                float y = last_pos->y - radius*sin(theta);
                appr_pos->x = x;
                appr_pos->y = y;
                appr_pos->theta = last_pos->theta;
            }
        }

        if (way_index==nb_waypoints-1 && alignFirst){posTab[way_index] = appr_pos;}
        else{posTab[way_index] = curr_position;}
    }

    if (alignFirst){
        PUREPURSUIT(wheeledBase, purePursuit, posTab, nb_waypoints, dir, appr_pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", appr_pos.x, appr_pos.y, appr_pos.theta, posi->x, posi->y, posi->theta);
        }

        maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);

        const Position *finalPosTab[2]={myPos, last_pos};
        PUREPURSUIT(wheeledBase, purePursuit, finalPosTab, 2, dir, last_pos->theta);
        while(!(Wheeledbase::POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = Wheeledbase::GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);

    }else{
        PUREPURSUIT(wheeledBase, purePursuit, posTab, nb_waypoints, dir, last_pos->theta);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            const Position *posi = GET_POSITION(wheeledBase);
            //printf("%f %f %f %f, %f, %f\n", appr_pos.x, appr_pos.y, appr_pos.theta, posi->x, posi->y, posi->theta);
        }
    }

    free(posTab);
}

void Wheeledbase::GOTO_FUNCT(WheeledBase* wheeledBase, PurePursuit* purePursuit, TurnOnTheSpot* turnOnTheSpot, Position* pos, void* duringMovingFunct, void* approachFunct, bool alignFirst, PurePursuit::Direction dir, float finalAngle){
    auto *first_funct = (void (*)()) duringMovingFunct;
    auto *second_funct = (void (*)()) approachFunct;

    float defaultMaxSpeed = GET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID);

    const Position *myPos = GET_POSITION(wheeledBase);
    if (dir==PurePursuit::NONE) {
        if(cos(atan2(pos->y-myPos->y, pos->x-myPos->x)-myPos->theta)>=0) {
            dir=PurePursuit::FORWARD;
        }else {
            dir=PurePursuit::BACKWARD;

        }
    }
    const Position *posTab[2]={myPos, pos};

    double maxSpeed = defaultMaxSpeed;

    if (alignFirst){
        //On avance vers un point avant le point d'arrivé pour s'aligner et après on avance tout droit
        float theta = PI-pos->theta;
        float radius = ALIGN_DISTANCE;

        float x = pos->x - radius*cos(theta);
        float y = pos->y - radius*sin(theta);
        Position appr_pos = Position(x,y,pos->theta);
        const Position *posApprTab[2]={myPos, &appr_pos};

        PUREPURSUIT(wheeledBase, purePursuit, posApprTab, 2, dir, pos->theta);
        first_funct();
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            /*const Position *posi = GET_POSITION(wheeledBase);
            printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);*/
        }

        maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);

        PUREPURSUIT(wheeledBase, purePursuit, posTab, 2, dir, pos->theta);
        second_funct();
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            /*const Position *posi = GET_POSITION(wheeledBase);
            printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);*/
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);
    }else{
        //On avance dans une ligne droite et on ralenti à la fin
        PUREPURSUIT(wheeledBase, purePursuit, posTab, 2, dir, pos->theta);

        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            /*
            const Position *posi = GET_POSITION(wheeledBase);
            int distance = sqrt(pow(posi->x-pos->x, 2)+pow(posi->y-pos->y, 2));
            if (distance<SLOWDOWN_DISTANCE){
                maxSpeed = maxSpeed > defaultMaxSpeed*0.1 ? maxSpeed * SLOWDOWN_FACTOR : maxSpeed;
                SET_PARAMETER_VALUE(wheeledBase, WBPID_POSITIONCONTROL_LINVELMAX_ID, maxSpeed);
            }
            */
            //printf("%f %f %f %f, %f, %f\n", pos->x, pos->y, pos->theta, posi->x, posi->y, posi->theta);
            //Do nothing
        }

        SET_PARAMETER_VALUE(wheeledBase, purePursuit, WBPID_POSITIONCONTROL_LINVELMAX_ID, defaultMaxSpeed);
    }


    if(finalAngle!=MAXFLOAT) {

        START_TURNONTHESPOT(wheeledBase, turnOnTheSpot, false, finalAngle);
        while(!(POSITION_REACHED(wheeledBase) & 0b01)) {
            //Todo: TimeOUT
        }
    }
}

void Wheeledbase::GET_VELOCITIES_WANTED(const WheeledBase* wheeledBase, float* linOutput, float* angOutput, bool spin) {
    if (spin) {
        *linOutput = wheeledBase->velocityControl.getLinOutput();
        *angOutput = wheeledBase->velocityControl.getAngOutput();
    }
    else {
        *linOutput = wheeledBase->velocityControl.getLinSpinGoal();
        *angOutput = wheeledBase->velocityControl.getAngSpinGoal();
    }
}


void Wheeledbase::SET_POSITION(WheeledBase* wheeledBase, const Position* pos) {
    wheeledBase->odometry.setPosition(pos->x, pos->y, pos->theta);
}

const Position* Wheeledbase::GET_POSITION(const WheeledBase* wheeledBase) {
    const Position* pos = wheeledBase->odometry.getPosition();
    return pos;
}

void Wheeledbase::GET_VELOCITIES(const WheeledBase* wheeledBase, float* linVel, float* angVel) {
    *linVel = wheeledBase->odometry.getLinVel();
    *angVel = wheeledBase->odometry.getAngVel();
}

void Wheeledbase::SET_PARAMETER_VALUE(WheeledBase* wheeledBase, PurePursuit* purePursuit, WBParameterID paramID, float value) {
    switch (paramID) {
    case WBPID_LEFTWHEEL_RADIUS_ID:
        wheeledBase->leftWheel.setWheelRadius(value);
        break;
    case WBPID_LEFTWHEEL_CONSTANT_ID:
        wheeledBase->leftWheel.setConstant(value);
        break;
    case WBPID_RIGHTWHEEL_RADIUS_ID:
        wheeledBase->rightWheel.setWheelRadius(value);
        break;
    case WBPID_RIGHTWHEEL_CONSTANT_ID:
        wheeledBase->rightWheel.setConstant(value);
        break;
    case WBPID_LEFTCODEWHEEL_RADIUS_ID:
        wheeledBase->leftCodewheel.setWheelRadius(value);
        break;
    case WBPID_LEFTCODEWHEEL_COUNTSPERREV_ID:
        wheeledBase->leftCodewheel.setCountsPerRev(static_cast<long>(value));
        break;
    case WBPID_RIGHTCODEWHEEL_RADIUS_ID:
        wheeledBase->rightCodewheel.setWheelRadius(value);
        break;
    case WBPID_RIGHTCODEWHEEL_COUNTSPERREV_ID:
        wheeledBase->rightCodewheel.setCountsPerRev(static_cast<long>(value));
        break;
    case WBPID_ODOMETRY_AXLETRACK_ID:
        wheeledBase->odometry.setAxleTrack(value);
        break;
    case WBPID_ODOMETRY_SLIPPAGE_ID:
        wheeledBase->odometry.setSlippage(value);
        break;
    case WBPID_VELOCITYCONTROL_AXLETRACK_ID:
        wheeledBase->velocityControl.setAxleTrack(value);
        break;
    case WBPID_VELOCITYCONTROL_MAXLINACC_ID:
        wheeledBase->velocityControl.setMaxLinAcc(value);
        break;
    case WBPID_VELOCITYCONTROL_MAXLINDEC_ID:
        wheeledBase->velocityControl.setMaxLinDec(value);
        break;
    case WBPID_VELOCITYCONTROL_MAXANGACC_ID:
        wheeledBase->velocityControl.setMaxAngAcc(value);
        break;
    case WBPID_VELOCITYCONTROL_MAXANGDEC_ID:
        wheeledBase->velocityControl.setMaxAngDec(value);
        break;
    case WBPID_VELOCITYCONTROL_SPINSHUTDOWN_ID:
        wheeledBase->velocityControl.setSpinShutdown(static_cast<bool>(value));
        break;
    case WBPID_LINVELPID_KP_ID:
        wheeledBase->linVelPID.setTunings(value, wheeledBase->linVelPID.getKi(), wheeledBase->linVelPID.getKd());
        break;
    case WBPID_LINVELPID_KI_ID:
        wheeledBase->linVelPID.setTunings(wheeledBase->linVelPID.getKp(), value, wheeledBase->linVelPID.getKd());
        break;
    case WBPID_LINVELPID_KD_ID:
        wheeledBase->linVelPID.setTunings(wheeledBase->linVelPID.getKp(), wheeledBase->linVelPID.getKi(), value);
        break;
    case WBPID_LINVELPID_MINOUTPUT_ID:
        wheeledBase->linVelPID.setOutputLimits(value, wheeledBase->linVelPID.getMaxOutput());
        break;
    case WBPID_LINVELPID_MAXOUTPUT_ID:
        wheeledBase->linVelPID.setOutputLimits(wheeledBase->linVelPID.getMinOutput(), value);
        break;
    case WBPID_ANGVELPID_KP_ID:
        wheeledBase->angVelPID.setTunings(value, wheeledBase->angVelPID.getKi(), wheeledBase->angVelPID.getKd());
        break;
    case WBPID_ANGVELPID_KI_ID:
        wheeledBase->angVelPID.setTunings(wheeledBase->angVelPID.getKp(), value, wheeledBase->angVelPID.getKd());
        break;
    case WBPID_ANGVELPID_KD_ID:
        wheeledBase->angVelPID.setTunings(wheeledBase->angVelPID.getKp(), wheeledBase->angVelPID.getKi(), value);
        break;
    case WBPID_ANGVELPID_MINOUTPUT_ID:
        wheeledBase->angVelPID.setOutputLimits(value, wheeledBase->angVelPID.getMaxOutput());
        break;
    case WBPID_ANGVELPID_MAXOUTPUT_ID:
        wheeledBase->angVelPID.setOutputLimits(wheeledBase->angVelPID.getMinOutput(), value);
        break;
    case WBPID_POSITIONCONTROL_LINVELKP_ID:
        wheeledBase->positionControl.setVelTunings(value, wheeledBase->positionControl.getAngVelKp());
        break;
    case WBPID_POSITIONCONTROL_ANGVELKP_ID:
        wheeledBase->positionControl.setVelTunings(wheeledBase->positionControl.getLinVelKp(), value);
        break;
    case WBPID_POSITIONCONTROL_LINVELMAX_ID:
        wheeledBase->positionControl.setVelLimits(value, wheeledBase->positionControl.getAngVelMax());
        break;
    case WBPID_POSITIONCONTROL_ANGVELMAX_ID:
        wheeledBase->positionControl.setVelLimits(wheeledBase->positionControl.getLinVelMax(), value);
        break;
    case WBPID_POSITIONCONTROL_LINPOSTHRESHOLD_ID:
        wheeledBase->positionControl.setPosThresholds(value, wheeledBase->positionControl.getAngPosThreshold());
        break;
    case WBPID_POSITIONCONTROL_ANGPOSTHRESHOLD_ID:
        wheeledBase->positionControl.setPosThresholds(wheeledBase->positionControl.getLinPosThreshold(), value);
        break;
    case WBPID_PUREPURSUIT_LOOKAHED_ID:
        purePursuit->setLookAhead(value);
        break;
    case WBPID_PUREPURSUIT_LOOKAHEADBIS_ID:
        purePursuit->setLookAheadBis(value);
        break;
    default:
        break;
    }
}



float Wheeledbase::GET_PARAMETER_VALUE(WheeledBase* wheeledBase, const PurePursuit* purePursuit, WBParameterID paramID) {
    if (paramID == WBPID_LEFTWHEEL_RADIUS_ID) {
        return wheeledBase->leftWheel.getWheelRadius();
    }
    if (paramID == WBPID_LEFTWHEEL_CONSTANT_ID) {
        return wheeledBase->leftWheel.getConstant();
    }
    if (paramID == WBPID_RIGHTWHEEL_RADIUS_ID) {
        return wheeledBase->rightWheel.getWheelRadius();
    }
    if (paramID == WBPID_RIGHTWHEEL_CONSTANT_ID) {
        return wheeledBase->rightWheel.getConstant();
    }
    if (paramID == WBPID_LEFTCODEWHEEL_RADIUS_ID) {
        return wheeledBase->leftCodewheel.getWheelRadius();
    }
    if (paramID == WBPID_LEFTCODEWHEEL_COUNTSPERREV_ID) {
        return wheeledBase->leftCodewheel.getCountsPerRev();
    }

    if (paramID == WBPID_RIGHTCODEWHEEL_RADIUS_ID) {
        return wheeledBase->rightCodewheel.getWheelRadius();
    }
    if (paramID == WBPID_RIGHTCODEWHEEL_COUNTSPERREV_ID) {
        return wheeledBase->rightCodewheel.getCountsPerRev();
    }

    if (paramID == WBPID_ODOMETRY_AXLETRACK_ID) {
        return wheeledBase->odometry.getAxleTrack();
    }
    if (paramID == WBPID_ODOMETRY_SLIPPAGE_ID) {
        return wheeledBase->odometry.getSlippage();
    }

    if (paramID == WBPID_VELOCITYCONTROL_AXLETRACK_ID) {
        return wheeledBase->velocityControl.getAxleTrack();
    }
    if (paramID == WBPID_VELOCITYCONTROL_MAXLINACC_ID) {
        return wheeledBase->velocityControl.getMaxLinAcc();
    }
    if (paramID == WBPID_VELOCITYCONTROL_MAXLINDEC_ID) {
        return wheeledBase->velocityControl.getMaxLinDec();
    }
    if (paramID == WBPID_VELOCITYCONTROL_MAXANGACC_ID) {
        return wheeledBase->velocityControl.getMaxAngAcc();
    }
    if (paramID == WBPID_VELOCITYCONTROL_MAXANGDEC_ID) {
        return wheeledBase->velocityControl.getMaxAngDec();
    }
    if (paramID == WBPID_VELOCITYCONTROL_SPINSHUTDOWN_ID) {
        return (float)wheeledBase->velocityControl.getSpinShutdown();
    }

    if (paramID == WBPID_LINVELPID_KP_ID) {
        return wheeledBase->linVelPID.getKp();
    }
    if (paramID == WBPID_LINVELPID_KI_ID) {
        return wheeledBase->linVelPID.getKi();
    }
    if (paramID == WBPID_LINVELPID_KD_ID) {
        return wheeledBase->linVelPID.getKd();
    }
    if (paramID == WBPID_LINVELPID_MINOUTPUT_ID) {
        return wheeledBase->linVelPID.getMinOutput();
    }
    if (paramID == WBPID_LINVELPID_MAXOUTPUT_ID) {
        return wheeledBase->linVelPID.getMaxOutput();
    }

    if (paramID == WBPID_ANGVELPID_KP_ID) {
        return wheeledBase->angVelPID.getKp();
    }
    if (paramID == WBPID_ANGVELPID_KI_ID) {
        return wheeledBase->angVelPID.getKi();
    }
    if (paramID == WBPID_ANGVELPID_KD_ID) {
        return wheeledBase->angVelPID.getKd();
    }
    if (paramID == WBPID_ANGVELPID_MINOUTPUT_ID) {
        return wheeledBase->angVelPID.getMinOutput();
    }
    if (paramID == WBPID_ANGVELPID_MAXOUTPUT_ID) {
        return wheeledBase->angVelPID.getMaxOutput();
    }

    if (paramID == WBPID_POSITIONCONTROL_LINVELKP_ID) {
        return wheeledBase->positionControl.getLinVelKp();
    }
    if (paramID == WBPID_POSITIONCONTROL_ANGVELKP_ID) {
        return wheeledBase->positionControl.getAngVelKp();
    }
    if (paramID == WBPID_POSITIONCONTROL_LINVELMAX_ID) {
        return wheeledBase->positionControl.getLinVelMax();
    }
    if (paramID == WBPID_POSITIONCONTROL_ANGVELMAX_ID) {
        return wheeledBase->positionControl.getAngVelMax();
    }
    if (paramID == WBPID_POSITIONCONTROL_LINPOSTHRESHOLD_ID) {
        return wheeledBase->positionControl.getLinPosThreshold();
    }
    if (paramID == WBPID_POSITIONCONTROL_ANGPOSTHRESHOLD_ID) {
        return wheeledBase->positionControl.getAngPosThreshold();
    }
    if (paramID == WBPID_PUREPURSUIT_LOOKAHED_ID) {
        return purePursuit->getLookAhead();
    }
    if (paramID == WBPID_PUREPURSUIT_LOOKAHEADBIS_ID) {
        return purePursuit->getLookAheadBis();
    }
    return 0;
}

void Wheeledbase::PRINT_PARAMS(WheeledBase* wheeledBase, const PurePursuit* purePursuit) {
    wb_logger.log(INFO_LEVEL, "WheeledBase parameters:\n");
    wb_logger.log(INFO_LEVEL, "LEFTWHEEL_RADIUS_ID: %f\n", wheeledBase->leftWheel.getWheelRadius());
    wb_logger.log(INFO_LEVEL, "LEFTWHEEL_CONSTANT_ID: %f\n", wheeledBase->leftWheel.getConstant());
    wb_logger.log(INFO_LEVEL, "RIGHTWHEEL_RADIUS_ID: %f\n", wheeledBase->rightWheel.getWheelRadius());
    wb_logger.log(INFO_LEVEL, "RIGHTWHEEL_CONSTANT_ID: %f\n", wheeledBase->rightWheel.getConstant());
    wb_logger.log(INFO_LEVEL, "LEFTCODEWHEEL_RADIUS_ID: %f\n", wheeledBase->leftCodewheel.getWheelRadius());
    wb_logger.log(INFO_LEVEL, "LEFTCODEWHEEL_COUNTSPERREV_ID: %ld\n", wheeledBase->leftCodewheel.getCountsPerRev());
    wb_logger.log(INFO_LEVEL, "RIGHTCODEWHEEL_RADIUS_ID: %f\n", wheeledBase->rightCodewheel.getWheelRadius());
    wb_logger.log(INFO_LEVEL, "RIGHTCODEWHEEL_COUNTSPERREV_ID: %ld\n", wheeledBase->rightCodewheel.getCountsPerRev());
    wb_logger.log(INFO_LEVEL, "ODOMETRY_AXLETRACK_ID: %f\n", wheeledBase->odometry.getAxleTrack());
    wb_logger.log(INFO_LEVEL, "ODOMETRY_SLIPPAGE_ID: %f\n", wheeledBase->odometry.getSlippage());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_AXLETRACK_ID: %f\n", wheeledBase->velocityControl.getAxleTrack());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_MAXLINACC_ID: %f\n", wheeledBase->velocityControl.getMaxLinAcc());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_MAXLINDEC_ID: %f\n", wheeledBase->velocityControl.getMaxLinDec());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_MAXANGACC_ID: %f\n", wheeledBase->velocityControl.getMaxAngAcc());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_MAXANGDEC_ID: %f\n", wheeledBase->velocityControl.getMaxAngDec());
    wb_logger.log(INFO_LEVEL, "VELOCITYCONTROL_SPINSHUTDOWN_ID: %d\n", wheeledBase->velocityControl.getSpinShutdown());
    wb_logger.log(INFO_LEVEL, "LINVELPID_KP_ID: %f\n", wheeledBase->linVelPID.getKp());
    wb_logger.log(INFO_LEVEL, "LINVELPID_KI_ID: %f\n", wheeledBase->linVelPID.getKi());
    wb_logger.log(INFO_LEVEL, "LINVELPID_KD_ID: %f\n", wheeledBase->linVelPID.getKd());
    wb_logger.log(INFO_LEVEL, "LINVELPID_MINOUTPUT_ID: %f\n", wheeledBase->linVelPID.getMinOutput());
    wb_logger.log(INFO_LEVEL, "LINVELPID_MAXOUTPUT_ID: %f\n", wheeledBase->linVelPID.getMaxOutput());
    wb_logger.log(INFO_LEVEL, "ANGVELPID_KP_ID: %f\n", wheeledBase->angVelPID.getKp());
    wb_logger.log(INFO_LEVEL, "ANGVELPID_KI_ID: %f\n", wheeledBase->angVelPID.getKi());
    wb_logger.log(INFO_LEVEL, "ANGVELPID_KD_ID: %f\n", wheeledBase->angVelPID.getKd());
    wb_logger.log(INFO_LEVEL, "ANGVELPID_MINOUTPUT_ID: %f\n", wheeledBase->angVelPID.getMinOutput());
    wb_logger.log(INFO_LEVEL, "ANGVELPID_MAXOUTPUT_ID: %f\n", wheeledBase->angVelPID.getMaxOutput());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_LINVELKP_ID: %f\n", wheeledBase->positionControl.getLinVelKp());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_ANGVELKP_ID: %f\n", wheeledBase->positionControl.getAngVelKp());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_LINVELMAX_ID: %f\n", wheeledBase->positionControl.getLinVelMax());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_ANGVELMAX_ID: %f\n", wheeledBase->positionControl.getAngVelMax());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_LINPOSTHRESHOLD_ID: %f\n", wheeledBase->positionControl.getLinPosThreshold());
    wb_logger.log(INFO_LEVEL, "POSITIONCONTROL_ANGPOSTHRESHOLD_ID: %f\n", wheeledBase->positionControl.getAngPosThreshold());
    wb_logger.log(INFO_LEVEL, "PUREPURSUIT_LOOKAHED_ID: %f\n", purePursuit->getLookAhead());
    wb_logger.log(INFO_LEVEL, "PUREPURSUIT_LOOKAHEADBIS_ID: %f\n", purePursuit->getLookAheadBis());
}
