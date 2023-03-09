//
//  Controller.hpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#pragma once

#include "gclib.h"
#include "gclibo.h"

#include "Galil.hpp"

#include <memory>
#include <string>
#include <cstring>

#define GALIL_BUFFER_SIZE G_SMALL_BUFFER
#define GALIL_NUM_AXES 5

namespace Galil
{
    class Controller
    {
    public:
        Controller(GCStringIn ipAddress);
        virtual ~Controller();
        /** Abort motion  */
        GReturn abort()        { command("AB"); return G_NO_ERROR; }
        GReturn allMotorsOff() { command("MO"); return G_NO_ERROR; }
        
        /**
         Send commands to Galil Controller
         
         @param command (GCStringIn Galil command to send to GMC
         
         @returns GCStringOut of command response
         
         */
        GCStringOut command(GCStringIn command);
        GCStringOut command(const std::string& command) { return this->command(command.c_str()); }
        
        /** Get the axes positions and statuses
         
         @param axes (boolean array of whether to get this axes for not
         @param absolute (bool, Default=false) whether to get the absolute positions or not.
         
         */
        bool getAxisMoving(size_t axis) { return getAxesMoving()[axis]; }
        bool getAxisMoving(char axis)   { return getAxisMoving( axisIndex(axis) ); }
        Array<bool> getAxesMoving(); // queries all axes if thehy are moving or not
        Array<long> getPosition (const Array<bool>& axes, bool absolute=true);
        
        /* Check if motion is complete */
        GCStringOut motionComplete();
        
        // turn on/off the motors
        Array<bool> getMotorsOn ();
        Array<bool> getMotorsOff();

        GReturn motorsOn (const Array<bool>& axes);
        GReturn motorsOff(const Array<bool>& axes);
        
        /**
         Move the axes
         
         @param axes (long[GALIL_NUM_AXES]) array of axes move counts to move
         @param absolute (bool, Default=false) whether to perform absolute movements or not
         
         */
        GReturn moveAxes(const Array<long>& axes, bool absolute=false)
        {
            if (absolute) return moveAxesAbsolute(axes);
            else          return moveAxesRelative(axes);
        } // moveAxess
        GReturn moveAxesAbsolute(const Array<long>& axes); // absolute move axes
        GReturn moveAxesRelative(const Array<long>& axes); // relative move axes
        
        /** Set PID constants
         
         */
        GReturn setPID_P(const Array<long>& kp_axes);
        GReturn setPID_I(const Array<long>& kI_axes);
        GReturn setPID_D(const Array<long>& kd_axes);
        
        
        /** Set speed control variables
         
         */
        GReturn setAcceleration(const Array<long>& ac_axes);
        GReturn setDeceleration(const Array<long>& dc_axes);
        GReturn setSpeed(const Array<long>& sp_axes);
        
        /** Stop moving axes
         @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
         */
        GReturn stopAxes(const Array<bool>& axes);
        
        /** Zero specific axes
         @param axes bool[GALIL_NUM_AXES[] on whether to stop particular axes or not
         */
        GReturn zeroAxes(const Array<bool>& axes);
        
        /** Get the axis Name for each axes */
        static inline char   axisName(size_t axis)      { return static_cast<char>(axisBaseIndex + axis); }
        static inline size_t axisIndex(char axisLetter) { return static_cast<size_t>(axisLetter - axisBaseIndex); }

        /** Check for Valid GReturn code */
        static inline bool validGalilReturnCode(const GReturn rc) { return rc == G_NO_ERROR; }
        
    private: // fields
        // members
        GCon m_gc = 0;
        GSize m_bytesRead = 0;
        char m_buffer[GALIL_BUFFER_SIZE];
        
        // static
        static const size_t axisBaseIndex = 65;
        
    private: // methods
        // static methods
        static inline GCStringOut bufferToGCStringOut(char* buffer, unsigned int buffer_size);
        static inline void e(const GReturn rc){ if ( !validGalilReturnCode(rc) ) throw rc; } // checks for galil error codes
        
        // member methods
        /** Flush the buffer memory  */
        inline void flushBuffer(){ memset(m_buffer, 0, GALIL_BUFFER_SIZE); }
        
        
    }; // class: GalilController

} // namespace: Galil
