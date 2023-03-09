//
//  Robot.hpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#pragma once

#include "Galil.hpp"
#include "Axis.hpp"
#include "Controller.hpp"

#include <map>
#include <vector>

namespace Galil
{
    class Robot
    {
    public:
        // constructors
        Robot(const std::string& ip_address);
        Robot(const std::shared_ptr<Controller>& controller);
        Robot(const std::string& ip_address, std::initializer_list<Axis> axes);
        Robot(const std::shared_ptr<Controller>& controller, std::initializer_list<Axis> axes);
        
        ~Robot() = default;
        
        /** Add an axis to the robot. True if axis is inserted, false if axis is not inserted */
        bool attachAxis(const Axis& axis, const bool setprops = false);
        std::vector<bool> attachAxes(std::initializer_list<Axis> axes);

        Array<bool>         galilAxesAttached() const;
        std::vector<size_t> galilAxisIndicesAttached() const;
        
        /** Get the associated axis
         
            raises an 'std::out_of_range' exception if the axis doesn't exist
         */
        Axis  getAxis   (size_t axisIndex) const { return m_axes.at(axisIndex); }
        Axis  getAxis   (char axisLetter)  const { return getAxis(Controller::axisIndex(axisLetter)); }
        Axis& getAxisRef(size_t axisIndex)       { return m_axes.at(axisIndex); }
        Axis& getAxisRef(char axisLetter)        { return getAxisRef(Controller::axisIndex(axisLetter)); }
        
        /** Check if a specific axis is attached to the robot */
        bool isAxisAttached(size_t axisIndex) const { return m_axes.find(axisIndex) != m_axes.end(); }
        bool isAxisAttached(char axisLetter)  const { return isAxisAttached( Controller::axisIndex(axisLetter) ); }
        bool isAxisAttached(const Axis& axis) const { return isAxisAttached( axis.getAxisIndex() ); }
        
        /** Pop an axis out of the current robot .*/
        Axis& removeAxis(size_t axisIndex);
        Axis& removeAxis(char axisLetter){ return removeAxis(Controller::axisIndex(axisLetter)); }
        
        /** Swap axes from two different locations */
        bool swapAxis(size_t axis1, size_t axis2);
        bool swapAxis(char   axis1, size_t axis2){ return swapAxis(Controller::axisIndex(axis1), axis2); }
        bool swapAxis(size_t axis1, char   axis2){ return swapAxis(axis1, Controller::axisIndex(axis2)); }
        bool swapAxis(char   axis1, char   axis2){ return swapAxis(Controller::axisIndex(axis1), Controller::axisIndex(axis2)); }
        
        /** Galil Commands */
        bool abort()        const { m_controller->abort();        return true; }
        bool allMotorsOff() const { m_controller->allMotorsOff(); return true; }
        
        // default galil commands
        GCStringOut galilCommand(GCStringIn command)   const { return m_controller->command(command); }
        GCStringOut galilCommand(const std::string& command) const { return m_controller->command(command); }

        // get the axis positions
        Array<bool>  getAxesMoving();
        Array<bool>  getAxesMoving(const std::vector<size_t>& axisIndices);
        Array<float> getPosition  (const std::vector<size_t>& axisIndices, bool absolute=true) const;

        // motor control: turning on and off motors
        Array<bool> getMotorsOn ();
        Array<bool> getMotorsOff();
        Array<bool> motorsOn (const std::vector<size_t>& axes);
        Array<bool> motorsOff(const std::vector<size_t>& axes);
        
        // move the axes (Relative/Absolute)
        Array<bool> moveAxes(const std::map<size_t, float>& positions, bool absolute=false)
        {
            if(absolute) return moveAxesAbsolute(positions);
            else         return moveAxesRelative(positions);
            
        } // moveAxes
        Array<bool> moveAxesAbsolute(const std::map<size_t, float>& positions);
        Array<bool> moveAxesRelative(const std::map<size_t, float>& positions);
        
        // All Axes properties
        Array<bool> setAxesProperties() { return setAxesProperties(m_axes); } // sets all axis properties attached
        Array<bool> setAxesProperties(const std::vector<size_t>& axes); // sets current axis properties by index
        Array<bool> setAxesProperties(const std::vector<Axis>& axes);   // sets axis properties by Axis
        Array<bool> setAxesProperties(const std::map<size_t, Axis>& axisProps); // sets axis properties by Axis
        Array<bool> setAxesProperties(const std::map<size_t, AxisProperties>& axes); // main function to change axis properties

        // set Axis limts
        Array<bool> setAxisLimits(const std::map<size_t, AxisLimits<float>>& limits);

        // set (Speed/Acceleration/Deceleration)
        Array<bool> setAcceleration(const std::map<size_t, float>& accelerations);
        Array<bool> setDeceleration(const std::map<size_t, float>& decelerations);
        Array<bool> setSpeed(const std::map<size_t, float>& speeds);
        
        // setPID_(P/I/D) (Proportional/Integral/Derivative gains)
        Array<bool> setPID_P(const std::map<size_t, long>& kps);
        Array<bool> setPID_I(const std::map<size_t, long>& kis);
        Array<bool> setPID_D(const std::map<size_t, long>& kds);

        // stop the axes
        Array<bool> stopAxes(const std::vector<size_t>& axes);
        
        // zero the axes
        Array<bool> zeroAxes(const std::vector<size_t>& axes);

    protected:
        bool        setAxisProperties(const Axis& axis) { return setAxesProperties({axis})[axis.getAxisIndex()]; }
        
    private:
        std::shared_ptr<Controller> m_controller;
        
        std::map<size_t, Axis> m_axes;
        
    }; // class: Robot
} // namespace: Galil
