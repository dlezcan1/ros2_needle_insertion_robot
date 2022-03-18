#pragma once

#include "gclib.h"
#include "gclibo.h"

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <cstring>

#define GALIL_BUFFER_SIZE G_SMALL_BUFFER
#define GALIL_NUM_AXES 5

// NULL AXES operations
const long NULL_LONG_AXIS = std::numeric_limits<long>::lowest();
inline static bool isNullAxis(long axis) { return (axis == NULL_LONG_AXIS); }

// Helper functions for command preparation
template <typename T>
std::string commaSeparateValues(std::vector<T> values)
{
    std::string csv = "";
    for (int i = 0; i < values.size(); i++)
        csv += std::to_string(values[i]) +
        ((i < values.size() - 1) && !isNullAxis(values[i]) ? "," : "");
    
    return csv;
    
} // commaSeparateValues: vector

template <typename T, std::size_t N>
std::string commaSeparateValues(std::array<T, N> values)
{
    std::string csv = "";
    for (int i = 0; i < values.size(); i++)
        csv += std::to_string(values[i]) +
        ((i < values.size() - 1) && !isNullAxis(values[i]) ? "," : "");
    
    return csv;
    
} // commaSeparateValues: array

class GalilController
{
public:
    GalilController(GCStringIn ipAddress);
    ~GalilController();
    /** Abort motion  */
    GReturn abort(){ command("AB"); return 0; }
    
    /**
     Send commands to Galil Controller
     
     @param command (GCStringIn Galil command to send to GMC
     
     @returns GCStringOut of command response
     
     */
    GCStringOut command(GCStringIn command);
    GCStringOut command(std::string& command){ return this->command(command.c_str()); }
    
    /** Get the axes positions
     
     @param axes (boolean array of whether to get this axes for not
     @param absolute (bool, Default=false) whether to get the absolute positions or not.
     
     */
    long* getPosition (bool axes[GALIL_NUM_AXES], bool absolute=false);
    
    /* Check if motion is complete */
    GCStringOut motionComplete();
    
    // turn on/off the motors
    GReturn motorsOn(bool axes[GALIL_NUM_AXES]);
    GReturn motorsOff(bool axes[GALIL_NUM_AXES]);
    
    /**
     Move the axes
     
     @param axes (long[GALIL_NUM_AXES]) array of axes move counts to move
     @param absolute (bool, Default=false) whether to perform absolute movements or not
     
     */
    GReturn moveAxes(long axes[GALIL_NUM_AXES], bool absolute=false);
    GReturn moveAxesAbsolute(long axes[GALIL_NUM_AXES]); // absolute move axes
    GReturn moveAxesRelative(long axes[GALIL_NUM_AXES]); // relative move axes
    
    /** Set PID constants
     
     */
    GReturn setPID_P(long kp_axes[GALIL_NUM_AXES]);
    GReturn setPID_I(long kI_axes[GALIL_NUM_AXES]);
    GReturn setPID_D(long kd_axes[GALIL_NUM_AXES]);
    
    
    /** Set speed control variables
     
     */
    GReturn setAcceleration(long ac_axes[GALIL_NUM_AXES]);
    GReturn setDeceleration(long dc_axes[GALIL_NUM_AXES]);
    GReturn setSpeed(long sp_axes[GALIL_NUM_AXES]);
    
    /** Stop moving axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    GReturn stopAxes(bool axes[GALIL_NUM_AXES]);
    
    /** Zero specific axes
     @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
     */
    GReturn zeroAxes(bool axes[GALIL_NUM_AXES]);
    
    /** Get the axis Name for each axes */
    static inline char axisName(unsigned int axis){ return static_cast<char>(65 + axis); }
    
private: // members
    GCon m_gc = 0;
    GSize m_bytesRead = 0;
    char m_buffer[GALIL_BUFFER_SIZE];
    
private: // methods
    
    // static methods
    static inline GCStringOut bufferToGCStringOut(char* buffer, unsigned int buffer_size);
    static inline void e(GReturn rc){ if (rc != G_NO_ERROR) throw rc; }
    
    // member methods
    /** Flush the buffer memory  */
    inline void flushBuffer(){ memset(m_buffer, 0, GALIL_BUFFER_SIZE); }
    
    
}; // class: GalilController


