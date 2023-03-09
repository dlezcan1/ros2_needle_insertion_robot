#include "galil/NeedleInsertionRobot.hpp"

NeedleInsertionRobot::NeedleInsertionRobot(const std::string& ip_address) : NeedleInsertionRobot(ip_address, RobotArray<size_t>{1, 2, 3, 0})
{

} // default constructor

NeedleInsertionRobot::NeedleInsertionRobot(const std::string& ip_address, const RobotArray<size_t>& axisMapping) :
    m_robot(std::make_shared<Galil::Robot>(ip_address)), m_axisMappings(axisMapping)
{
    // attach the specified axes
    m_robot->attachAxes({
        {GalilIndex(0), {s_countsPerDistance[0], s_default_speed[0], s_default_acceleration[0], s_default_deceleration[0], s_default_kP[0], s_default_kI[0], s_default_kD[0]}},
        {GalilIndex(1), {s_countsPerDistance[1], s_default_speed[1], s_default_acceleration[1], s_default_deceleration[1], s_default_kP[1], s_default_kI[1], s_default_kD[1]}},
        {GalilIndex(2), {s_countsPerDistance[2], s_default_speed[2], s_default_acceleration[2], s_default_deceleration[2], s_default_kP[2], s_default_kI[2], s_default_kD[2]}},
        {GalilIndex(3), {s_countsPerDistance[3], s_default_speed[3], s_default_acceleration[3], s_default_deceleration[3], s_default_kP[3], s_default_kI[3], s_default_kD[3]}},
    });

    allMotorsOff(); // ensure all motors start off

} // constructor

NeedleInsertionRobot::~NeedleInsertionRobot()
{
    allMotorsOff(); // turn off all motors upon destruction

} // destructor

/** Status Checks */
RobotArray<float> NeedleInsertionRobot::getPosition(const RobotArray<bool>& axes, bool absolute = true) const
{
    std::vector<size_t> galil_axes;

    for (size_t robot_idx = 0; robot_idx < axes.size(); robot_idx++)
    {
        if (axes[robot_idx])
            galil_axes.push_back( GalilIndex(robot_idx) );
    } // for
    
    return GalilToRobotArray( m_robot->getPosition( galil_axes, absolute ) );

} // NeedleInsertionRobot::getPosition

RobotArray<bool> NeedleInsertionRobot::getAxesMoving() const
{
    return GalilToRobotArray( m_robot->getAxesMoving() );
    
} // NeedleInsertionRobot::getAxesMoving

/** Commanding the Robot */
// motion
void NeedleInsertionRobot::moveAxesAbsolute(const RobotArray<float>& command)
{
    m_robot->moveAxesAbsolute( RobotArrayToGalilRobotMap(command) );

} // NeedleInsertionRobot::moveAxesAbsolute

void NeedleInsertionRobot::moveAxesRelative(const RobotArray<float>& command)
{
    m_robot->moveAxesRelative( RobotArrayToGalilRobotMap(command) );

} // NeedleInsertionRobot::moveAxesRelative

// Motor control
void NeedleInsertionRobot::motorsOn(const RobotArray<bool>& axes)
{
    m_robot->motorsOn(RobotToggleToGalilIndices(axes));
} // NeedleInsertionRobot::motorsOn

void NeedleInsertionRobot::motorsOff(const RobotArray<bool>& axes)
{
    m_robot->motorsOff(RobotToggleToGalilIndices(axes));

} // NeedleInsertionRobot::motorsOff

// set Axis Parameters
void NeedleInsertionRobot::setAxisProperties(const std::map<size_t, Galil::AxisProperties>& props_robot)
{

    std::map<size_t, Galil::AxisProperties> props_galil;

    for (const auto& kv : props_robot)
        props_galil[kv.first] = kv.second;

    m_robot->setAxesProperties(props_galil);

} // NeedleInsertionRobot::setAxisProperties

void NeedleInsertionRobot::setSpeed(const RobotArray<float>& speed)
{
    m_robot->setSpeed(RobotArrayToGalilRobotMap(speed));

} // NeedleInsertionRobot::setSpeed

void NeedleInsertionRobot::setAcceleration(const RobotArray<float>& acceleration)
{
    m_robot->setAcceleration(RobotArrayToGalilRobotMap(acceleration));

} // NeedleInsertionRobot::setAcceleration

void NeedleInsertionRobot::setDeceleration(const RobotArray<float>& deceleration)
{
    m_robot->setDeceleration(RobotArrayToGalilRobotMap(deceleration));

} // NeedleInsertionRobot::setDeceleration

void NeedleInsertionRobot::setPID_P(const RobotArray<long>& kps)
{
    m_robot->setPID_P(RobotArrayToGalilRobotMap(kps));

} // NeedleInsertionRobot::setPID_P

void NeedleInsertionRobot::setPID_I(const RobotArray<long>& kis)
{
    m_robot->setPID_I(RobotArrayToGalilRobotMap(kis));

} // NeedleInsertionRobot::setPID_I

void NeedleInsertionRobot::setPID_D(const RobotArray<long>& kds)
{
    m_robot->setPID_D(RobotArrayToGalilRobotMap(kds));

} // NeedleInsertionRobot::setPID_D

// axis limits
void NeedleInsertionRobot::setAxesLimits(const std::map<size_t, Galil::AxisLimits<float>>& limits)
{
    m_robot->setAxisLimits(RobotMapToGalilRobotMap(limits));

} // NeedleInsertionRobot::setAxesLimits

void NeedleInsertionRobot::stopAxes(const RobotArray<bool>& axes) const
{
    m_robot->stopAxes(RobotToggleToGalilIndices(axes));

} // NeedleInsertionRobot::stopAxes

void NeedleInsertionRobot::zeroAxes(const RobotArray<bool>& axes) const
{
    m_robot->stopAxes(RobotToggleToGalilIndices(axes));

} // NeedleInsertionRobot::zeroAxes
