//
//  NeedleInsertionRobot.h
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/4/22.
//

#pragma once

#include "GalilController.h"


#define DEFAULT_GALIL_IP "192.168.1.201"
#define ROBOT_NUM_AXES 4

// NULL AXES operations
const float NULL_FLOAT_AXIS = std::numeric_limits<float>::lowest();
inline static bool isNullAxis(float axis){return axis == NULL_FLOAT_AXIS; }

class NeedleInsertionRobot
{
public:
    // constructor and destructor
    /** Default constructor with pre-defined galil IP address.
            
     */
    NeedleInsertionRobot();
   /** Constructor with galil controller IP address
    
    @param ipAddress GStringIn of the IP address of the controller.
  
    */
    NeedleInsertionRobot(GCStringIn ipAddress);
    ~NeedleInsertionRobot();
    
    /** Get the galil controller for custom commands
            
            @returns GalilController pointer to current controller
     */
    const std::shared_ptr<GalilController> getGalilController() const {return m_galilController;}
    /** Send a direct commadn to the galiil controller
            @param command GCStringIn of the galil command to send
     */
    GCStringOut galilCommand(GCStringIn command) const { return m_galilController->command(command); }
    GCStringOut galilCommand(std::string& command) const { return m_galilController->command(command); }
    
    /** Motor control commands */
    void abort(){m_galilController->abort();}
    
    /** Motor Control shortcuts */
    void allMotorsOn () const { motorsOn (s_axes); }
    void allMotorsOff() const { motorsOff(s_axes); }
    
    /** Get motor position commands
        @param axes boolean array of which axes that would like to be queried
        @param absolute (default=false) whether to use absolute positioning or not
     */
    float* getPosition(const bool axes[ROBOT_NUM_AXES], const bool absolute=false) const;
    // single axis-implementations
    float getPositionX (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {true,  false, false, false}; return getPosition(axes, absolute)[0]; }
    float getPositionY (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, true,  false, false}; return getPosition(axes, absolute)[1]; }
    float getPositionZ (const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, false, true,  false}; return getPosition(axes, absolute)[2]; }
    float getPositionLS(const bool absolute=false) const {bool axes[ROBOT_NUM_AXES] = {false, false, true,  false}; return getPosition(axes, absolute)[3]; }
    
    // turn on/off the motors
    void motorsOn (const bool axes[ROBOT_NUM_AXES]) const;
    void motorsOff(const bool axes[ROBOT_NUM_AXES]) const;
    
    /**
     Move the axes
     
     @param axes (long[GALIL_NUM_AXES]) array of axes move counts to move
     @param absolute (bool, Default=false) whether to perform absolute movements or not
     
     */
    void moveAxes(const float axes[ROBOT_NUM_AXES], bool absolute=false) const {if (absolute) return moveAxesAbsolute(axes); else return moveAxesRelative(axes);}
    void moveAxesAbsolute(const float axes[ROBOT_NUM_AXES]) const ; // absolute move axes
    void moveAxesRelative(const float axes[ROBOT_NUM_AXES]) const ; // relative move axes
    
    /* Controller commands to set control parameters */
    /** Set PID constants
     
     */
    void setPID_P(const long kp_axes[ROBOT_NUM_AXES]) const;
    void setPID_I(const long ki_axes[ROBOT_NUM_AXES]) const;
    void setPID_D(const long kd_axes[ROBOT_NUM_AXES]) const;
    
    
    /** Set speed control variables  */
    void setAcceleration(const float ac_axes[ROBOT_NUM_AXES]) const;
    void setDeceleration(const float dc_axes[ROBOT_NUM_AXES]) const;
    void setSpeed(const float sp_axes[ROBOT_NUM_AXES]) const;
    
    /** Stop moving axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    void stopAxes(const bool axes[ROBOT_NUM_AXES]) const;
    void stopAllAxes() const { stopAxes(s_axes); }
    
    /** Zero specific axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    void zeroAxes(const bool axes[ROBOT_NUM_AXES]) const;
    void zeroAllAxes() const { zeroAxes(s_axes); }

// public member functions
public: // static defaults
    /* Speed defaults */
    constexpr static const float s_default_speed[ROBOT_NUM_AXES]        = {2.5, 2.5, 2.5, 2.0}; // speeds per mm
    constexpr static const float s_default_acceleration[ROBOT_NUM_AXES] = {1.5, 1.5, 1.5, 10000.0/43680.0};
    constexpr static const float s_default_deceleration[ROBOT_NUM_AXES] = {1.5, 1.5, 1.5, 10000.0/43680.0};
    constexpr static float s_countsPerDistance[ROBOT_NUM_AXES] = {2000.0, 2000.0, 2000.0, 43680.0}; // calibrated counts/mm
    
    /* PID defaults*/
    constexpr static const long s_default_kP[ROBOT_NUM_AXES] = { 54,  15,  54,  25};
    constexpr static const long s_default_kI[ROBOT_NUM_AXES] = {  4,   4,   4,   4};
    constexpr static const long s_default_kD[ROBOT_NUM_AXES] = {480, 332, 480, 480};

    /* Axes defaults */
    constexpr static bool s_axes[GALIL_NUM_AXES] = {false, true, true, true, true}; // Galil axes that are turned on
    
// public static members
    
private: // private members
    std::shared_ptr<GalilController> m_galilController;
    
// private members
protected:
    /* Index for each axes in the Galil Controller Axes */
    const static size_t m_xIdx  = 1; // B: x axis
    const static size_t m_yIdx  = 2; // C: y axis
    const static size_t m_zIdx  = 3; // D: z axis
    const static size_t m_lsIdx = 4; // E: linear stage axis
    
    size_t getGalilAxisIndex(size_t axis) const
    {
        switch (axis)
        {
            case 0:
                return m_xIdx;
                
            case 1:
                return m_yIdx;
                
            case 2:
                return m_zIdx;
                
            case 3:
                return m_lsIdx;
                
            default:
                return -1;
        }
    } // getGalilAxisIndex
    
    /* Robot <---> Galil Axis mappings */
    template <typename T>
    T* galilToRobotAxes(const T axes[GALIL_NUM_AXES]) const
    {
        T* robot_axes = new T[ROBOT_NUM_AXES];
        
        // copy over data
        for (size_t i = 0; i < ROBOT_NUM_AXES; i++)
            robot_axes[i] = axes[getGalilAxisIndex(i)];
        
        return robot_axes;
        
    } // galilToRobotAxes
    
    bool* robotToGalilAxes(const bool axes[ROBOT_NUM_AXES]) const
    {
        bool* gc_axes = new bool[GALIL_NUM_AXES];
        
        // initialize gc_axes
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            gc_axes[i] = false;
        
        // remap the axis index
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            gc_axes[getGalilAxisIndex(i)] = axes[i];
        
        return gc_axes;
        
    } // robotToGalilAxes (bool)
    
    long* robotToGalilAxes(const long axes[ROBOT_NUM_AXES]) const
    {
        long* gc_axes = new long[GALIL_NUM_AXES];
        
        // initialize gc_axes
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            gc_axes[i] = NULL_LONG_AXIS;
        
        // remap the axis index
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            gc_axes[getGalilAxisIndex(i)] = axes[i];
        
        return gc_axes;
        
    } // robotToGalilAxes (long)
    
    /* Conversions */
    /** Convert encoder counts to distance
        @param axes const long array axes of the distance measurements to counts
     
        @returns float array of the measurements in encoder counts
     
     */
    float* countsToDistance(const long axes[ROBOT_NUM_AXES]) const
    {
        float* f_axes = new float[ROBOT_NUM_AXES];
        
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            f_axes[i] = isNullAxis(axes[i]) ? NULL_FLOAT_AXIS : ((float) axes[i])/s_countsPerDistance[i];
        
        return f_axes;
        
    } // float
    
    /** Convert distance to encoder counts
        @param axes const float array axes of the distance measurements to counts
     
        @returns long array of the measurements in encoder counts
     */
    long* distanceToCounts(const float axes[ROBOT_NUM_AXES]) const
    {
        long* l_axes = new long[ROBOT_NUM_AXES];
        
        // perform the conversion while checking for null axis
        for (int i = 0; i < ROBOT_NUM_AXES; i++)
            l_axes[i] = isNullAxis(axes[i]) ? NULL_LONG_AXIS : std::round(axes[i] * s_countsPerDistance[i]);
        
        return l_axes;
        
    } // distanceToCounts
    
// protected
    
}; // class: NeedleInsertionRobot
