//
//  Axis.hpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#pragma once

#include "Galil.hpp"
#include "Controller.hpp"

#include <limits>
#include <algorithm>

namespace Galil
{
    template <typename T>
    struct AxisLimits
    {
        // fields
        bool active = false;

        T min = std::numeric_limits<T>::lowest(),
          max = std::numeric_limits<T>::max();
        
        // constructors
        AxisLimits() {}
        AxisLimits(bool active) : active(active) {}
        AxisLimits(T min, T max) : AxisLimits(min, max, false) {}
        AxisLimits(T min, T max, bool active) : min(min), max(max), active(active) {}
        AxisLimits(const AxisLimits& limits) = default;
                
        // functions/methods
        inline T limitMotion(const T& command) const
        {
            return active ? std::min( std::max( min, command ), max) : command;
            
        } // limitMotion

        void turnOffMin()
        {
            min = std::numeric_limits<T>::lowest();

        } // turnOffMin

        void turnOffMax()
        {
            max = std::numeric_limits<T>::max();

        } // turnOffMax
        
    }; // struct: AxisLimits
    
    // Axis Properties
    struct AxisProperties
    {
        float countsPerDistance; // units: mm
        
        float speed;        // units mm/s
        float acceleration; // units mm/s
        float deceleration; // units mm/s
        
        long pid_kP; // units: counts
        long pid_kI; // units: counts
        long pid_kD; // units: counts
        
    }; // struct: AxisProperties
    
    class Axis
    {
    public:
        // constructors and destructors
        Axis(size_t axisIndex, const AxisProperties& axisProperties);
        Axis(size_t axisIndex, const AxisProperties& axisProperties, const AxisLimits<float>& axisLimits);
        Axis(const Axis& axis) = default;
        virtual ~Axis() = default;
        
        // getters and setters for axis index
        const char getAxisName()    const { return Controller::axisName( getAxisIndex() ); }
        const size_t getAxisIndex() const { return m_axisIndex; }
        void setAxisIndex(size_t axisIndex);
                
        /** conversion functions */
        float countsToDistance(long counts)    const;
        long  distanceToCounts(float distance) const;

        /** Axis Limit commands */
        AxisLimits<float>  getLimits()    const                       { return m_limits; }
        AxisLimits<float>& getLimitsRef()                             { return m_limits; }
        bool               setLimits(const AxisLimits<float>& limits); 
        
        // Get/Set Limit commands
        float getLimitMax() const    { return m_limits.max; }
        bool  setLimitMax(float max) { m_limits.max = max; return true; }

        float getLimitMin() const    { return m_limits.min; }
        bool  setLimitMin(float min) { m_limits.min = min; return true; }
        
        bool  getLimitActive() const      { return m_limits.active; }
        bool  setLimitActive(bool active) { m_limits.active = active; return true; }

        // limit motion commands (only takes absolute positioning, relative left to robot class)
        float limitMotion(const float& command) const;
                
        /** Axis properties commands */
        AxisProperties  getAxisProperties()    const                   { return m_axisProps; }
        AxisProperties& getAxisPropertiesRef()                         { return m_axisProps; }
        bool            setAxisProperties(const AxisProperties& props) { m_axisProps = props; return true; }
        
        // Get/Set counts per distance. Returns true if set properly
        float getCountsPerDistance()    const          { return getAxisProperties().countsPerDistance; }
        bool  setCountsPerDistance(const float counts) { m_axisProps.countsPerDistance = counts; return true; }
        
        // Get/Set speed commands. Returns true if set properly
        float getAcceleration()         const { return getAxisProperties().acceleration; }
        bool  setAcceleration(const float ac) { m_axisProps.acceleration = ac; return true; }
        
        float getDeceleration()         const { return getAxisProperties().deceleration; }
        bool  setDeceleration(const float dc) { m_axisProps.deceleration = dc; return true; }
        
        float getSpeed()                const { return getAxisProperties().speed; }
        bool  setSpeed(const float sp)        { m_axisProps.speed = sp; return true; }
        
        // Get/Set PID commands. Returns true if set properly
        long getPID_P()        const { return getAxisProperties().pid_kP; }
        bool setPID_P(const long kp) { m_axisProps.pid_kP = kp; return true; }
        
        long getPID_I()        const { return getAxisProperties().pid_kI; }
        bool setPID_I(const long ki) { m_axisProps.pid_kI = ki; return true; }
        
        long getPID_D()        const { return getAxisProperties().pid_kD; }
        bool setPID_D(const long kd) { m_axisProps.pid_kD = kd; return true; }
        
        
    private:
        size_t m_axisIndex;
        AxisProperties m_axisProps;
        AxisLimits<float> m_limits;
                
    }; // class: Axis
} // namespace: Galil
