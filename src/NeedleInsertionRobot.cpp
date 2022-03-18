//
//  NeedleInsertionRobot.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/4/22.
//

#include "NeedleInsertionRobot.h"
#include <iostream>

/* Static Value References */
const float NeedleInsertionRobot::s_default_speed[ROBOT_NUM_AXES];
const float NeedleInsertionRobot::s_default_acceleration[ROBOT_NUM_AXES];
const float NeedleInsertionRobot::s_default_deceleration[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kP[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kI[ROBOT_NUM_AXES];
const long NeedleInsertionRobot::s_default_kD[ROBOT_NUM_AXES];
const bool NeedleInsertionRobot::s_axes[GALIL_NUM_AXES];
const float NeedleInsertionRobot::s_countsPerDistance[ROBOT_NUM_AXES];

NeedleInsertionRobot::NeedleInsertionRobot()
{
    NeedleInsertionRobot(DEFAULT_GALIL_IP);
    
} // default constructor

NeedleInsertionRobot::NeedleInsertionRobot(GCStringIn ipAddress) : m_galilController(std::make_shared<GalilController>(ipAddress))
{
    // Set speed controls
    setSpeed(s_default_speed);
    setAcceleration(s_default_acceleration);
    setDeceleration(s_default_deceleration);
    
    // set PID controls
    setPID_P(s_default_kP);
    setPID_I(s_default_kI);
    setPID_D(s_default_kD);
    
} // constructor with IP address

NeedleInsertionRobot::~NeedleInsertionRobot()
{
    m_galilController.reset(); // delete pointer
    
} // destructor

float* NeedleInsertionRobot::getPosition(const bool axes[ROBOT_NUM_AXES], const bool absolute) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    long* gc_counts = m_galilController->getPosition(gc_axes); // get encoder counts

    long* counts = galilToRobotAxes(gc_counts); // convert to Robot axes mappings
    float* positions = countsToDistance(counts); // convert encoder counts to robot positions
    
    return positions;
    
    
} // NeedleInsertionRobot::getPosition

void NeedleInsertionRobot::motorsOn(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->motorsOn(gc_axes);
    
} // NeedleInsertionRobot::motorsOn

void NeedleInsertionRobot::motorsOff(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->motorsOff(gc_axes);
    
} // NeedleInsertionRobot::motorsOff


/* movement commands */
void NeedleInsertionRobot::moveAxesAbsolute(const float axes[ROBOT_NUM_AXES]) const
{
    // convert distance measurements to counts
    long* counts_axes = distanceToCounts(axes);
    
    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_galilController->moveAxesAbsolute(gc_axes);
    
    
} // NeedleInsertionRobot::moveAxesAbsolute

void NeedleInsertionRobot::moveAxesRelative(const float axes[ROBOT_NUM_AXES]) const
{
    // convert distance measurements to counts
    long* counts_axes = distanceToCounts(axes);
    
    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_galilController->moveAxesRelative(gc_axes);
    
} // NeedleInsertionRobot::moveAxesRelative

/* set PID commands */
void NeedleInsertionRobot::setPID_P(const long kp_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(kp_axes);
    
    m_galilController->setPID_P(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_P

void NeedleInsertionRobot::setPID_I(const long ki_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(ki_axes);
    
    m_galilController->setPID_I(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_I

void NeedleInsertionRobot::setPID_D(const long kd_axes[ROBOT_NUM_AXES]) const
{
    long* gc_axes = robotToGalilAxes(kd_axes);
    
    m_galilController->setPID_D(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_D


/* set speed commands */
void NeedleInsertionRobot::setAcceleration(const float ac_axes[ROBOT_NUM_AXES]) const
{
    long* l_ac_axes = distanceToCounts(ac_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_ac_axes); // convert to galil axes format
    
    m_galilController->setAcceleration(gc_axes);
    
} //NeedleInsertionRobot::setAcceleration

void NeedleInsertionRobot::setDeceleration(const float dc_axes[ROBOT_NUM_AXES]) const
{
    long* l_dc_axes = distanceToCounts(dc_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_dc_axes); // convert to galil axes format
    
    m_galilController->setDeceleration(gc_axes);
    
} //NeedleInsertionRobot::setDeceleration

void NeedleInsertionRobot::setSpeed(const float sp_axes[ROBOT_NUM_AXES]) const
{
    long* l_sp_axes = distanceToCounts(sp_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_sp_axes); // convert to galil axes format
    
    m_galilController->setSpeed(gc_axes);
    
} //NeedleInsertionRobot::setSpeed

/* Axes commands */
void NeedleInsertionRobot::stopAxes(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->stopAxes(gc_axes);
    
} // NeedleInsertionRobot::stopAxes


void NeedleInsertionRobot::zeroAxes(const bool axes[ROBOT_NUM_AXES]) const
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_galilController->zeroAxes(gc_axes);
    
} // NeedleInsertionRobot::zeroAxes




