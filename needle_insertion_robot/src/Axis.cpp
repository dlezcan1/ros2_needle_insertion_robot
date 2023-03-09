//
//  Axis.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#include "galil/Axis.hpp"

#include <math.h>
#include <stdexcept>

namespace Galil
{
    Axis::Axis(size_t axisIndex, const AxisProperties& axisProperties)
    {
        setAxisIndex(axisIndex);
        setAxisProperties(axisProperties);

    } // easy constructor
    
    Axis::Axis(size_t axisIndex, const AxisProperties& axisProperties, const AxisLimits<float>& axisLimits) 
    {
        setAxisIndex(axisIndex);
        setAxisProperties(axisProperties);
        setLimits(axisLimits);

    } // constructor
    
    float Axis::countsToDistance(long counts) const
    {
        return isNullAxis(counts) ? NULL_FLOAT_AXIS : (float) counts / m_axisProps.countsPerDistance;
        
    } // Axis::countsToDistance
    
    long  Axis::distanceToCounts(float distance) const
    {
        return isNullAxis(distance) ? NULL_LONG_AXIS : std::round(distance * m_axisProps.countsPerDistance);
        
    } // Axis::distanceToCounts
    
    float Axis::limitMotion(const float& command) const
    {
        return m_limits.limitMotion(command);
        
    } // Axis::limitMotion
    
    void Axis::setAxisIndex(size_t axisIndex)
    {
        if (axisIndex < GALIL_NUM_AXES)
            m_axisIndex = axisIndex;
        else
            throw std::domain_error("Set axis index is out of bounds!");
        
    } // Axis::setAxisIndex

    bool Axis::setLimits(const AxisLimits<float>& limits)
    { 
        if (limits.min > limits.max) 
            return false;
        
        m_limits = limits; 
        return true;

    } // Axis::setLimits
} // namespace: Galil
