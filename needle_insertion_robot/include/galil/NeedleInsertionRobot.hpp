#pragma once

/**
 * @brief: this is to serve as an example and an actual implementation of the needle insertion robot
 * 
 * 
 */


#include "Galil.hpp"

#include <array>
#include <cstring>
#include <map>

#define ROBOT_NUM_AXES 4

// helpful typedefs
template <typename T> using RobotArray = std::array<T, ROBOT_NUM_AXES>;
template <typename T> const auto newRobotArray = Galil::newSTDArray<T, ROBOT_NUM_AXES>;

// Needle Insertion Robot class
class NeedleInsertionRobot
{
public:
    // constructors and destructors
    NeedleInsertionRobot(const std::string& ip_address);
    NeedleInsertionRobot(const std::string& ip_address, const RobotArray<size_t>& axisMapping);
    ~NeedleInsertionRobot();

    /** Status Checks */
    // Position
    RobotArray<float> getPosition(const RobotArray<bool>& axes, bool absolute = true) const; 
    float getPosition(size_t axisIndex, bool absolute = true) const { RobotArray<bool> axes = newRobotArray<bool>(); axes[axisIndex] = true; return getPosition(axes, absolute)[axisIndex]; }
    float getPositionX (bool absolute = true) const { return getPosition(s_gidx_X,  absolute); }
    float getPositionY (bool absolute = true) const { return getPosition(s_gidx_Y,  absolute); }
    float getPositionZ (bool absolute = true) const { return getPosition(s_gidx_Z,  absolute); }
    float getPositionLS(bool absolute = true) const { return getPosition(s_gidx_LS, absolute); }

    // Axis Moving
    RobotArray<bool> getAxesMoving()     const; 
    bool getAxisMoving(size_t axisIndex) const { RobotArray<bool> axes = newRobotArray<bool>(); axes[axisIndex] = true; return getAxesMoving()[axisIndex]; }
    bool getAxisMovingX () const { return getAxisMoving(s_gidx_X); }
    bool getAxisMovingY () const { return getAxisMoving(s_gidx_Y); }
    bool getAxisMovingZ () const { return getAxisMoving(s_gidx_Z); }
    bool getAxisMovingLS() const { return getAxisMoving(s_gidx_LS); }

    // Motor Status
    RobotArray<bool> getMotorsOn()             const { return GalilToRobotArray<bool>( m_robot->getMotorsOn()  ); };
    bool getMotorsOn  (const size_t axisIndex) const { return getMotorsOn()[axisIndex]; }
    bool getMotorsOnX () const { return getMotorsOn()[s_gidx_X];  }
    bool getMotorsOnY () const { return getMotorsOn()[s_gidx_Y];  }
    bool getMotorsOnZ () const { return getMotorsOn()[s_gidx_Z];  }
    bool getMotorsOnLS() const { return getMotorsOn()[s_gidx_LS]; }

    RobotArray<bool> getMotorsOff()             const { return GalilToRobotArray<bool>( m_robot->getMotorsOff() ); };
    bool getMotorsOff  (const size_t axisIndex) const { return getMotorsOff()[axisIndex]; }
    bool getMotorsOffX () const { return getMotorsOff()[s_gidx_X];  }
    bool getMotorsOffY () const { return getMotorsOff()[s_gidx_Y];  }
    bool getMotorsOffZ () const { return getMotorsOff()[s_gidx_Z];  }
    bool getMotorsOffLS() const { return getMotorsOff()[s_gidx_LS]; }

    /** Commanding the robot */
    // abort command
    void abort() { m_robot->abort(); }

    // motion
    void moveAxes(const RobotArray<float>& command, bool absolute = false)
    {
        if (absolute) moveAxesAbsolute(command);
        else          moveAxesRelative(command);
    } // moveAxes
    void moveAxesAbsolute(const RobotArray<float>& command); 
    void moveAxesRelative(const RobotArray<float>& command); 

    // Motor control
    void allMotorsOn () { RobotArray<bool> axes = newRobotArray<bool>(true); motorsOn(axes); }
    void allMotorsOff() { m_robot->allMotorsOff(); }
    void motorsOn (const RobotArray<bool>& axes); 
    void motorsOff(const RobotArray<bool>& axes); 

    // set Axis Parameters
    void setAxisProperties(const std::map<size_t, Galil::AxisProperties>& props); 

    void setSpeed       (const RobotArray<float>& speed);        
    void setAcceleration(const RobotArray<float>& acceleration); 
    void setDeceleration(const RobotArray<float>& deceleration); 

    void setPID_P(const RobotArray<long>& kps); 
    void setPID_I(const RobotArray<long>& kis); 
    void setPID_D(const RobotArray<long>& kds); 

    // axis limits
    void setAxesLimits  (const std::map<size_t, Galil::AxisLimits<float>>& limits); 
    void setAxisLimits  (const size_t index, const Galil::AxisLimits<float>& limit) { setAxesLimits({{index, limit}}); }
    void setAxisLimitsX (const Galil::AxisLimits<float>& limit) { setAxisLimits(s_gidx_X, limit);  }
    void setAxisLimitsY (const Galil::AxisLimits<float>& limit) { setAxisLimits(s_gidx_Y, limit);  }
    void setAxisLimitsZ (const Galil::AxisLimits<float>& limit) { setAxisLimits(s_gidx_Z, limit);  }
    void setAxisLimitsLS(const Galil::AxisLimits<float>& limit) { setAxisLimits(s_gidx_LS, limit); }

    // Stopping axes
    void stopAxes(const RobotArray<bool>& axes) const; 

    // zeroing axes
    void zeroAxes(const RobotArray<bool>& axes) const; 

private:
    std::shared_ptr<Galil::Robot> m_robot;
    RobotArray<size_t> m_axisMappings; // maps Standardized (0-3 | position in array) -> Galil Controller mappings

private: // static default values
    // ORGANIZED AS: LINEAR STAGE, X, Y, Z (indices below)
    static const size_t s_gidx_X  = 1,
                        s_gidx_Y  = 2,
                        s_gidx_Z  = 3,
                        s_gidx_LS = 0;

    /* Speed defaults */
    static const RobotArray<float> s_countsPerDistance;    // = {2000.0, 2000.0, 2000.0, 43680.0}; // calibrated counts/mm
    
    static const RobotArray<float> s_default_speed;        // = {2.5, 2.5, 2.5, 2.0}; // speeds per mm
    static const RobotArray<float> s_default_acceleration; // = {1.5, 1.5, 1.5, 10000.0/43680.0};
    static const RobotArray<float> s_default_deceleration; // = {1.5, 1.5, 1.5, 10000.0/43680.0};
    
    /* PID defaults*/
    static const RobotArray<long> s_default_kP; // = { 54,  54,  54,  25};
    static const RobotArray<long> s_default_kI; // = {  4,   4,   4,   4};
    static const RobotArray<long> s_default_kD; // = {480, 480, 480, 480};

private: // functions
    // The index in the Galil Controller
    size_t GalilIndex(size_t idx)   const { return m_axisMappings[idx]; }
    size_t GalilIndexX()            const { return GalilIndex(s_gidx_X); }
    size_t GalilIndexY()            const { return GalilIndex(s_gidx_Y); }
    size_t GalilIndexZ()            const { return GalilIndex(s_gidx_Z); }
    size_t GalilIndexLS()           const { return GalilIndex(s_gidx_LS); }

    template <typename T>
    RobotArray<T> GalilToRobotArray(Galil::Array<T> galil_array) const // remaps to standard robot indexing
    {
        RobotArray<T> robot_array = newRobotArray<T>();

        robot_array[ 0 ] = galil_array[ GalilIndexX()  ];
        robot_array[ 1 ] = galil_array[ GalilIndexY()  ];
        robot_array[ 2 ] = galil_array[ GalilIndexZ()  ];
        robot_array[ 3 ] = galil_array[ GalilIndexLS() ];

        return robot_array;

    } // GalilToRobotArray

    template <typename T>
    Galil::Array<T> RobotToGalilArray(const RobotArray<T>& robot_array) const // remaps to standard robot indexing
    {
        Galil::Array<T> galil_array = Galil::newArray<T>();

        galil_array[ GalilIndexX()  ] = robot_array[ 0 ];
        galil_array[ GalilIndexY()  ] = robot_array[ 1 ];
        galil_array[ GalilIndexZ()  ] = robot_array[ 2 ];
        galil_array[ GalilIndexLS() ] = robot_array[ 3 ];

        return robot_array;

    } // GalilToRobotArray

    template <typename T>
    std::map<size_t, T> RobotArrayToGalilRobotMap(const RobotArray<T>& robot_array) const
    {
        std::map<size_t, T> galil_map; 

        galil_map[ GalilIndexX()  ] = robot_array[ 0 ];
        galil_map[ GalilIndexY()  ] = robot_array[ 1 ];
        galil_map[ GalilIndexZ()  ] = robot_array[ 2 ];
        galil_map[ GalilIndexLS() ] = robot_array[ 3 ];

        return galil_map;

    } // RobotToGalilRobotMap

    template <typename T>
    std::map<size_t, T> RobotMapToGalilRobotMap(const std::map<size_t, T>& robot_map) const
    {   
        std::map<size_t, T> galil_map;

        for (const auto& kv : robot_map)
            galil_map[GalilIndex(kv.first)] = robot_map[kv.second];

        return galil_map;

    } // RobotMapToGalilRobotMap

    std::vector<size_t> RobotToggleToGalilIndices(const RobotArray<bool>& robot_array) const
    {
        std::vector<size_t> axis_indices;

        for (size_t robot_idx = 0; robot_idx < robot_array.size(); robot_idx++)
        {
            if (robot_array[robot_idx])
                axis_indices.push_back(GalilIndex(robot_idx));

        } // for

        return axis_indices;

    } // RobotToggleToGalilIndices

}; // class: NeedleInsertionRobot

/** static member initializations */
// default parameters
const RobotArray<float> NeedleInsertionRobot::s_countsPerDistance    = {2000.0, 2000.0, 2000.0, 43680.0}; // calibrated counts/mm

const RobotArray<float> NeedleInsertionRobot::s_countsPerDistance    = {2000.0, 2000.0, 2000.0, 43680.0}; // calibrated counts/mm
const RobotArray<float> NeedleInsertionRobot::s_default_speed        = {2.5, 2.5, 2.5, 2.0}; // speeds per mm
const RobotArray<float> NeedleInsertionRobot::s_default_acceleration = {1.5, 1.5, 1.5, 10000.0/43680.0};

const RobotArray<long> NeedleInsertionRobot::s_default_kP = { 54,  54,  54,  25};
const RobotArray<long> NeedleInsertionRobot::s_default_kI = {  4,   4,   4,   4};
const RobotArray<long> NeedleInsertionRobot::s_default_kD = {480, 480, 480, 480};