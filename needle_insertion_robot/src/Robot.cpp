//
//  Robot.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#include "galil/Robot.hpp"

namespace Galil
{
    Robot::Robot(const std::string& ip_address) : Robot(std::make_shared<Controller>(ip_address))
    {
        
    } // constructor(string)
    
    Robot::Robot(const std::shared_ptr<Controller>& controller) : m_controller(controller)
    {
        
    } // constructor(shared_ptr Controller)

    Robot::Robot(const std::string& ip_address, std::initializer_list<Axis> axes) :
        Robot(ip_address)
    {
        attachAxes(axes);

    } // constructor(string, Axes)
    Robot::Robot(const std::shared_ptr<Controller>& controller, std::initializer_list<Axis> axes) :
        Robot(controller)
    {
        attachAxes(axes);

    } // constructor(shared_ptr Controller, Axes)
    
    bool Robot::attachAxis(const Axis& axis, const bool setprops)
    {
        if (isAxisAttached(axis) || axis.getAxisIndex() >= GALIL_NUM_AXES) // check if already have an axis attached
            return false;
        
        
        m_axes.insert(std::pair<size_t, Axis>(axis.getAxisIndex(), axis));

        // set the current axis property commands upon attachment
        if (setprops)
            setAxisProperties( axis );
        
        return true;
        
    } // Robot::attachAxis(Axis)
    
    std::vector<bool> Robot::attachAxes(std::initializer_list<Axis> axes)
    {
        std::vector<bool> retval;
        retval.reserve(axes.size());
        
        // attach each axis
        for (auto ax: axes)
            retval.push_back( attachAxis(ax, false) );

        setAxesProperties(); // set the axis properties
        
        return retval;
        
    } // Robot::attachAxis(Axis)

    Array<bool> Robot::galilAxesAttached() const 
    {   
        Array<bool> attached = newArray<bool>();

        for (const auto& kv : m_axes)
            attached[kv.first] = true;

        return attached;

    } // Robot::galilAxesAttached

    std::vector<size_t> Robot::galilAxisIndicesAttached() const
    {
        std::vector<size_t> axis_indices;

        for (size_t ax_idx = 0; ax_idx < GALIL_NUM_AXES; ax_idx++)
        {
            if (isAxisAttached(ax_idx))
                axis_indices.push_back(ax_idx);

        } // for

        return axis_indices;

    } // Robot::galilAxisIndicesAttached
    
    Axis& Robot::removeAxis(size_t axisIndex)
    {
        
        Axis& removeAxis = m_axes.at(axisIndex); // get the axis before removing
            
        m_axes.erase(axisIndex); // remove the axis
        
        return removeAxis;
        
    } // Robot::removeAxis
    
    bool Robot::swapAxis(size_t axis1, size_t axis2)
    {
        if (axis1 == axis2) // Do nothing
            return false;
        
        else if ( axis1 < 0 || axis1 >= GALIL_NUM_AXES ) // check for valid axes
            return false;
        
        else if ( axis2 < 0 || axis2 >= GALIL_NUM_AXES ) // check for valid axes
            return false;

        else if (!isAxisAttached(axis1) && !isAxisAttached(axis2)) // no axes attached here to swap
            return false;
        
        // 2 valid axes. Check if one exists
        Axis* ax1 = isAxisAttached(axis1) ? &m_axes.at(axis1) : nullptr;
        Axis* ax2 = isAxisAttached(axis2) ? &m_axes.at(axis2) : nullptr;
        
        if (ax1 != nullptr && ax2 == nullptr) // axis 2 open
        {
            ax1->setAxisIndex(axis2);     // change the axis index
            Axis tmp = removeAxis(axis1); // remove the current axis and hold it
            attachAxis(*ax1);             // reinsert the new, updated axis
            
        } // if
        
        else if (ax1 == nullptr && ax2 != nullptr) // axis 1 open
        {
            ax2->setAxisIndex(axis1);     // change the axis index
            Axis tmp = removeAxis(axis2); // remove the current axis and hold it
            attachAxis(*ax2);             // reinsert the new, updated axis
        } // else if
        
        else if (ax1 != nullptr && ax2 != nullptr) // need to swap
        {
            ax2->setAxisIndex( axis1 );      // swap the axis index
            ax1->setAxisIndex( axis2 );      // swap the axis index
            
            Axis tmp1 = removeAxis( axis1 ); // remove the axes
            Axis tmp2 = removeAxis( axis2 ); // remove the axes
            
            attachAxis( *ax1 );              // reattach the axes
            attachAxis( *ax2 );              // reattach the axes
            
        } // else if
        else // shouldn't happen, but just in case neither is attached
            return false;
        
        
        return true;
            
    } // Robot::swapAxis
    
    /** Galil commands */
    Array<bool> Robot::getAxesMoving()
    {
        Array<bool> moving = m_controller->getAxesMoving();
        
        for (size_t idx = 0; idx < moving.size(); idx++)
            moving[idx] = moving[idx] && isAxisAttached(idx); 

        return moving;

    } // Robot::getAxesMoving

    Array<bool> Robot::getAxesMoving(const std::vector<size_t>& axisIndices)
    {
        Array<bool> moving = getAxesMoving();
        Array<bool> query = newArray<bool>();

        for (size_t ax_idx : axisIndices)
        {
            if (isAxisAttached(ax_idx))
                query[ax_idx] = true;

        } // for

        // null-out any of the non-queried axes
        for (size_t idx = 0; idx < moving.size(); idx++)
            moving[idx] = moving[idx] && query[idx];

        return moving;

    } // Robot::getAxesMoving

    Array<float> Robot::getPosition(const std::vector<size_t>& axisIndices, bool absolute) const
    {
        // setup command
        Array<bool> query_axes = newArray<bool>();
        
        // what are the query axes
        for (auto ax_idx : axisIndices) // toggle the axes that are attached
        {
            if (isAxisAttached(ax_idx))
                query_axes [ax_idx] = true;
                
        } // for
        
        // get and parse the response
        Array<long> positions_l = m_controller->getPosition(query_axes, absolute);
        Array<float> positions_f = newArray<float>();
        
        // set the positions output
        for (size_t idx = 0; idx < GALIL_NUM_AXES; idx++)
        {
            if (!isAxisAttached(idx) || !query_axes[idx] || isNullAxis( positions_l[idx] ))
                positions_f[idx] = NULL_FLOAT_AXIS;
            
            else
                positions_f[idx] = getAxis(idx).countsToDistance( positions_l[idx] );
            
        } // for
        
        return positions_f;
        
    } // Robot::getPosition

    Array<bool> Robot::getMotorsOn ()
    {
        Array<bool> on = m_controller->getMotorsOn();

        for (size_t idx = 0; idx < on.size(); idx++)
            on[idx] = on[idx] && isAxisAttached(idx);

        return on;

    } // Robot::getMotorsOn
    
    Array<bool> Robot::getMotorsOff()
    {
        Array<bool> off = m_controller->getMotorsOff();

        for (size_t idx = 0; idx < off.size(); idx++)
            off[idx] = off[idx] && isAxisAttached(idx);

        return off;

    } // Robot::getMotorsOff

    Array<bool> Robot::motorsOn (const std::vector<size_t>& axes)
    {
        // setup query and return value
        Array<bool> query = newArray<bool>();
        
        for (size_t ax_idx : axes)
        {
            if (isAxisAttached(ax_idx))
                query[ax_idx] = true;

        } // for

        // send the command
        m_controller->motorsOn(query);

        return query;

    } // Robot::motorsOn

    Array<bool> Robot::motorsOff(const std::vector<size_t>& axes)
    {
        // setup query and return value
        Array<bool> query = newArray<bool>();
        
        for (size_t ax_idx : axes)
        {
            if (isAxisAttached(ax_idx))
                query[ax_idx] = true;

        } // for

        // send the command
        m_controller->motorsOff(query);
        
        return query;

    } // Robot::motorsOff
    
    Array<bool> Robot::moveAxesAbsolute(const std::map<size_t, float>& positions)
    {
        // setup the command and return value
        Array<long> command = newArray<long>();
        Array<bool> moved   = newArray<bool>();
        
        // iterate through the commands and set it
        for (const auto& kv: positions)
        {
            // unpack the axis
            size_t ax_idx = kv.first;
            float position = kv.second;
           
            if ( !isAxisAttached(ax_idx) && !isNullAxis(position) )
                continue;

            // limit the motion
            const Axis& axis = getAxis( ax_idx );
            float position_ltd = axis.limitMotion( position );

            // set the position command
            moved[ax_idx]   = true;
            command[ax_idx] = axis.distanceToCounts( position_ltd );
                
        } // for
        
        // send the command
        m_controller->moveAxesAbsolute( command );
        
        return moved;
        
    } // Robot::moveAxesAbsolute
    
    Array<bool> Robot::moveAxesRelative(const std::map<size_t, float>& positions)
    {
        // setup the command and return value
        Array<long> command = newArray<long>();
        Array<bool> moved   = newArray<bool>();

        Array<float> current_positions = getPosition(galilAxisIndicesAttached());
        
        // iterate through the commands and set it
        for (const auto& kv: positions)
        {
            // unpack the axis
            size_t ax_idx = kv.first;
            float position = kv.second;
           
            if ( !isAxisAttached(ax_idx) && !isNullAxis(position))
                continue;

            // limit the motion
            const Axis& axis = getAxis(ax_idx);
            float position_ltd = axis.limitMotion(position + current_positions[ax_idx]) - current_positions[ax_idx];
            
            // set the position command
            moved[ax_idx]   = true;
            command[ax_idx] = axis.distanceToCounts(position_ltd);
            
        } // for
        
        // send the command
        m_controller->moveAxesRelative( command );
        
        return moved;
    
    } // Robot::moveAxesRelative

    Array<bool> Robot::setAxesProperties(const std::vector<size_t>& axes)
    {
        std::map<size_t, AxisProperties> axisProps;

        for (const size_t ax_idx : axes)
        {
            if (isAxisAttached(ax_idx))
                axisProps[ax_idx] = getAxis(ax_idx).getAxisProperties();

        } // for

        return setAxesProperties(axisProps);

    } // Robot::setAxesProperties

    Array<bool> Robot::setAxesProperties(const std::vector<Axis>& axes)
    {
        // format into a map
        std::map<size_t, AxisProperties> axisProps;

        for (const Axis& axis : axes)
        {
            if (isAxisAttached(axis))
                axisProps[axis.getAxisIndex()] = axis.getAxisProperties();

        } // for

        return setAxesProperties(axisProps);

    } // Robot::setAxesProperties

    Array<bool> Robot::setAxesProperties(const std::map<size_t, Axis>& axes)
    {
        std::map<size_t, AxisProperties> axisProps;

        for (const auto& kv : axes)
            axisProps[kv.first] = kv.second.getAxisProperties();

        return setAxesProperties(axisProps);

    } // Robot::setAxesProperties

    Array<bool> Robot::setAxesProperties(const std::map<size_t, AxisProperties>& axisProps)
    {
        // build the set Properties
        Array<bool> success = newArray<bool>();
        
        std::map<size_t, float> accelerations;
        std::map<size_t, float> decelerations;
        std::map<size_t, float> speeds;

        std::map<size_t, long> kps;
        std::map<size_t, long> kis;
        std::map<size_t, long> kds;

        for (const auto& kv : axisProps)
        {
            // unpack tuple
            const size_t ax_idx         = kv.first;
            const AxisProperties& props = kv.second;

            if (!isAxisAttached(ax_idx)) // check if attached
                continue; 
                
            success[ax_idx] = true;

            // set the property maps
            accelerations[ax_idx] = props.acceleration;
            decelerations[ax_idx] = props.deceleration;
            speeds[ax_idx]        = props.speed;

            kps[ax_idx] = props.pid_kP;
            kis[ax_idx] = props.pid_kI;
            kds[ax_idx] = props.pid_kD;

        } // for

        // set all of the properties
        setAcceleration(accelerations);
        setDeceleration(decelerations);
        setSpeed       (speeds);

        setPID_P(kps);
        setPID_I(kis);
        setPID_D(kds);

        return success;

    } // Robot::setAxesProperties

    Array<bool> Robot::setAxisLimits(const std::map<size_t, AxisLimits<float>>& limits)
    {
        Array<bool> success = newArray<bool>();

        for (const auto& kv : limits)
        {
            // unpack the key-value pair
            const size_t axis = kv.first;
            const AxisLimits<float>& limit = kv.second;

            // check if the axis is attached
            if (!isAxisAttached(axis))
                continue;

            // set the axis
            getAxisRef(axis).setLimits(limit);
            success[axis] = true;

        } // for

        return success;

    } // Robot::setAxisLimits
    
    Array<bool> Robot::setAcceleration(const std::map<size_t, float>& accelerations)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: accelerations )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            float value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setAcceleration( value );
            command[ ax_idx ] = getAxis(ax_idx).distanceToCounts(value);
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setAcceleration( command );
        
        return success;
        
    } // Robot::setAcceleration
    
    Array<bool> Robot::setDeceleration(const std::map<size_t, float>& decelerations)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: decelerations )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            float value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setDeceleration( value );
            command[ ax_idx ] = getAxis(ax_idx).distanceToCounts(value);
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setDeceleration( command );
        
        return success;
        
    } // Robot::setDeceleration
    
    Array<bool> Robot::setSpeed(const std::map<size_t, float>& speeds)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: speeds )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            float value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setSpeed( value );
            command[ ax_idx ] = getAxis(ax_idx).distanceToCounts(value);
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setSpeed( command );
        
        return success;
        
    } // Robot::setSpeed
    
    Array<bool> Robot::setPID_P(const std::map<size_t, long>& kps)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: kps )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            long value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setPID_P( value );
            command[ ax_idx ] = value;
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setPID_P( command );
        
        return success;
        
    } // Robot::setPID_P
    
    Array<bool> Robot::setPID_I(const std::map<size_t, long>& kis)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: kis )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            long value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setPID_I( value );
            command[ ax_idx ] = value;
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setPID_I(command );
        
        return success;
        
    } // Robot::setPID_I
    
    Array<bool> Robot::setPID_D(const std::map<size_t, long>& kds)
    {
        // setup the command and return value
        Array<bool> success = newArray<bool>();
        Array<long> command = newArray<long>();

        // iterate through the commands
        for ( const auto& kv: kds )
        {
            // unpack the tuple
            size_t ax_idx = kv.first;
            long value = kv.second;
            
            if ( !isAxisAttached(ax_idx) )
                continue;
            
            // update the acceleration, command, and return value
            getAxis( ax_idx ).setPID_D( value );
            command[ ax_idx ] = value;
            success[ ax_idx ] = true;
            
        } // for
        
        // send the command
        m_controller->setPID_D( command );
        
        return success;
        
    } // Robot::setPID_D

    
    Array<bool> Robot::stopAxes(const std::vector<size_t>& axes)
    {
        Array<bool> stopped = newArray<bool>();
        
        for (size_t ax_idx : axes)
        {
            if (!isAxisAttached(ax_idx))
                continue;

            stopped[ax_idx] = true;

        } // for

        m_controller->stopAxes(stopped);

        return stopped;

    } // Robot::stopAxes
    
    
    Array<bool> Robot::zeroAxes(const std::vector<size_t>& axes)
    {
        Array<bool> zeroed = newArray<bool>();
        
        for (size_t ax_idx : axes)
        {
            if (!isAxisAttached(ax_idx))
                continue;

            zeroed[ax_idx] = true;

        } // for

        m_controller->stopAxes(zeroed);

        return zeroed;

    } // Robot::zeroAxes
    
}; // namespace: Galil
