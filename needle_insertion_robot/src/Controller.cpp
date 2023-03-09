//
//  Controller.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#include "galil/Controller.hpp"

#include <algorithm>
#include <vector>
#include <stdexcept>

// Helper functions for command preparation
template <typename T>
std::string commaSeparateValues(const std::vector<T>& values)
{
    std::string csv = "";
    for (int i = 0; i < values.size(); i++)
        csv += std::to_string(values[i]) +
        ((i < values.size() - 1) && !Galil::isNullAxis(values[i]) ? "," : "");
    
    return csv;
    
} // commaSeparateValues: vector

template <typename T, std::size_t N>
std::string commaSeparateValues(const std::array<T, N>& values)
{
    std::string csv = "";
    for (int i = 0; i < values.size(); i++)
        csv += std::to_string(values[i]) +
        ((i < values.size() - 1) && !Galil::isNullAxis(values[i]) ? "," : "");
    
    return csv;
    
} // commaSeparateValues: array

template <typename T>
std::string commaSeparateValues(const Galil::Array<T>& values)
{
    std::string csv = "";
    for (int i = 0; i < values.size(); i++)
        csv += std::to_string(values[i]) +
        ((i < values.size() - 1) && !Galil::isNullAxis(values[i]) ? "," : "");
    
    return csv;
    
} // commaSeparateValues: Galil::Array



namespace Galil
{

    Controller::Controller(GCStringIn ipAddress)
    {
        GOpen(ipAddress, &m_gc); // open the connection
        
        
    } // GalilController Constructor

    Controller::~Controller()
    {
        GClose(m_gc);
        
    } // GalilController destructor

    GCStringOut Controller::bufferToGCStringOut(char* buffer, unsigned int buffer_size)
    {
        GCStringOut stringout = new char[GALIL_BUFFER_SIZE];
        
        memcpy(stringout, buffer, buffer_size);
        
        return stringout;
        
    } // GalilController::bufferToGCStringOut

    GCStringOut Controller::command(GCStringIn command)
    {
        e(GCmdT(m_gc, command, m_buffer, GALIL_BUFFER_SIZE, NULL)); // trimmed version
        // this->e(GCommand(m_gc, command, m_buffer, G_SMALL_BUFFER, &m_bytesRead)); // full version
        
        GCStringOut response = bufferToGCStringOut(m_buffer, GALIL_BUFFER_SIZE); // get the response
        flushBuffer(); // flush the buffer
        
        return response;
        
    } // GalilController::command

    GCStringOut Controller::motionComplete()
    {
        return (GCStringOut) "1";  // TODO: implement
        
    } // GalilController:: motionComplete

    Array<bool> Controller::getMotorsOn ()
    {
        Array<bool> on = newArray<bool>();

        // setup the command
        std::string command = "SH ";

        for (size_t i = 0; i < GALIL_NUM_AXES; i++)
            command += i < GALIL_NUM_AXES - 1 ? "?," : "?";

        // send the command
        GCStringOut response = this->command(command);
        std::string s_response(response);
       
        // parse the command
        std::string token;
        s_response += ","; // add-on to end to get last part
        std::size_t pos = 0;

        int counter = 0;
        while((pos = s_response.find(",")) != std::string::npos && (counter < GALIL_NUM_AXES))
        {
            token = s_response.substr(0, pos);            
            try
            {
                // add to result
                if (counter >= GALIL_NUM_AXES)
                    break;
            

                on[counter++] = std::stof(token) > 0;
                
            } // try
            catch (std::invalid_argument)
            {
                continue; // not a float
            }

            s_response.erase(0, pos + 1); // remove the processed part of string

        } // while


        return on;

    } // Controller::getMotorsOn

    Array<bool> Controller::getMotorsOff()
    {
        Array<bool> off = newArray<bool>();

        // setup the command
        std::string command = "MO ";

        for (size_t i = 0; i < GALIL_NUM_AXES; i++)
            command += i < GALIL_NUM_AXES - 1 ? "?," : "?";

        // send the command
        GCStringOut response = this->command(command);
        std::string s_response(response);
       
        // parse the command
        std::string token;
        s_response += ","; // add-on to end to get last part
        std::size_t pos = 0;

        int counter = 0;
        while((pos = s_response.find(",")) != std::string::npos && (counter < GALIL_NUM_AXES))
        {
            token = s_response.substr(0, pos);

            try
            {
                // add to result
                if (counter >= GALIL_NUM_AXES)
                    break;
            

                off[counter++] = std::stof(token) > 0;
                
            } // try
            catch (std::invalid_argument)
            {
                continue; // not a float
            }

            s_response.erase(0, pos + 1); // remove the processed part of string

        } // while

        return off;

    } // Controller::getMotorsOff

    GReturn Controller::motorsOn(const Array<bool>& axes)
    {
        bool any_on = false;
        std::string command = "SH ";
        for (int i = 0; i < GALIL_NUM_AXES; i++)
        {
            any_on = any_on || axes[i];
            if (axes[i])
                command += axisName(i);
            
        } // for
        
        if (any_on)
            this->command( command );
        
        return 0;
        
    } // GalilController::motorsOn

    GReturn Controller::motorsOff(const Array<bool>& axes)
    {
        bool any_on = false;
        std::string command = "MO ";
        for (int i = 0; i < GALIL_NUM_AXES; i++)
        {
            any_on = any_on || axes[i];
            if (axes[i])
                command += axisName(i);
            
        } // for
        
        if (any_on)
            this->command( command );
        
        return 0;
        
    } // GalilController::motorsOff
    
    GReturn Controller::moveAxesAbsolute(const Array<long>& axes)
    {
        // format the command message
        std::string mv_command = "PA " + commaSeparateValues(axes);
        std::string bg_command = "BG ";
        for (int i = 0; i < GALIL_NUM_AXES; i++)
        {
            //        mv_command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
            if (axes[i] != 0)
                bg_command += axisName(i);
            
        } // for
        
        this->command(mv_command); // send the move command
        this->command(bg_command); // send the begin command
        
        return 0;
        
    } // GalilController::moveAxesAbsolute

    
    GReturn Controller::moveAxesRelative(const Array<long>& axes)
    {
        // format the command message
        std::string mv_command = "PR " + commaSeparateValues(axes);
        std::string bg_command = "BG ";
        for (int i = 0; i < GALIL_NUM_AXES; i++)
        {
            //        mv_command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
            if (axes[i] != 0)
                bg_command += axisName(i);
            
        } // for
        
        this->command(mv_command); // send the move command
        this->command(bg_command); // send the begin command
        
        return 0;
        
    } // GalilController::moveAxesRelative

    Array<bool> Controller::getAxesMoving()
    {
        std::string command = "MG ";
        // sample command "MG _BGA, _BGB, _BGC, _BGD, _BGE";

        // Set up the galil command
        for (size_t axis = 0; axis < GALIL_NUM_AXES; axis++)
        {
            command += std::string("_BG") + axisName(axis);
            if (axis < GALIL_NUM_AXES - 1) // check for last axis
                command += ", \",\", "; // add comma-separator

        } // for

        // get the response
        GCStringOut response = this->command(command);
        std::string s_response(response);
        
        // setup return
        Array<bool> axesRunning;
        axesRunning.fill(false);
        
        // parse the response
        std::string token;
        s_response += ","; // add-on to end to get last part
        std::size_t pos = 0;

        int counter = 0;
        
        while((pos = s_response.find(",")) != std::string::npos && (counter < GALIL_NUM_AXES))
        {
            token = s_response.substr(0, pos);
            bool isnumeric = token.find_first_not_of("-0123456789.") == std::string::npos;
            
            try
            {
                // add to result
                if (counter >= GALIL_NUM_AXES)
                    break;
            

                axesRunning[counter++] = std::stof(token) > 0;
                
            } // try
            catch (std::invalid_argument)
            {
                continue; // not a float
            }

            s_response.erase(0, pos + 1); // remove the processed part of string

        } // while

        return axesRunning;

    } // Controller::getAxesMoving

    Array<long> Controller::getPosition(const Array<bool>& axes, bool absolute)
    {
        // prepare the command
        std::string command = "";
        if (absolute)
            command += "TP ";
        else
            command += "PR ";
        
        for (int i = 0; i < GALIL_NUM_AXES; i++)
        {
            if (i < GALIL_NUM_AXES - 1 )
                command += axes[i] ? "?," : ",";
            
            else // end of list
                command += axes[i] ? "?" : "";
            
        } // for
        
        // get the response
    GCStringOut response = this->command(command);
        std::string s_response(response);
        
        // parse the response
        std::array<long,GALIL_NUM_AXES> positions;
        positions.fill( NULL_LONG_AXIS ); // initalize array

        // parse the response
        std::string token;
        s_response += ","; // add-on to end to get last part
        std::size_t pos = 0;

        int counter = -1;
        auto update_counter = [&counter, &axes](){
            while(!axes[++counter] && counter < GALIL_NUM_AXES)
                continue;
            };
        update_counter();
        
        while((pos = s_response.find(",")) != std::string::npos)
        {
            token = s_response.substr(0, pos);
            bool isnumeric = token.find_first_not_of("0123456789 ") == std::string::npos;

            if (isnumeric)
            {
                // add to result
                if (counter >= GALIL_NUM_AXES)
                    break;
            

                positions[counter] = std::stol(token);
                update_counter();
                
            } // if

            s_response.erase(0, pos + 1); // remove the processed part of string

        } // while

        return positions;
        
    } // GalilController::getPosition


    GReturn Controller::setPID_P(const Array<long>& kp_axes)
    {
        std::string command = "KP " + commaSeparateValues(kp_axes);
        
        this->command(command);
        
        return 0;
        
    } // GalilController::setPID_P


    GReturn Controller::setPID_I(const Array<long>& ki_axes)
    {
        std::string command = "KI " + commaSeparateValues(ki_axes);
        
        this->command(command);
        
        return 0;
        
    } // GalilController::setPID_I


    GReturn Controller::setPID_D(const Array<long>& kd_axes)
    {
        std::string command = "KD " + commaSeparateValues(kd_axes);
        
        this->command(command);
        
        return 0;
        
    } // GalilController::setPID_D


    GReturn Controller::setAcceleration(const Array<long>& ac_axes)
    {
        std::string command = "AC " + commaSeparateValues(ac_axes);
        
        this->command(command); // send update command
        
        return 0;
        
    } // GalilController::setAcceleration


    GReturn Controller::setDeceleration(const Array<long>& dc_axes)
    {
        std::string command = "DC " + commaSeparateValues(dc_axes);
        
        this->command(command); // send update command
        
        return 0;
        
    } // GalilController::setDeceleration


    GReturn Controller::setSpeed(const Array<long>& sp_axes)
    {
        std::string command = "SP " + commaSeparateValues(sp_axes);
        
        this->command(command); // send update command
        
        return 0;
        
    } // GalilController::setSpeed


    GReturn Controller::stopAxes(const Array<bool>& axes)
    {
        std::string command = "ST ";
        
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            command += axes[i] ? std::string(1, axisName(i)) : "";
        
        this->command(command); // send the command
        
        return 0;
        
    } // GalilController::stopAxes


    GReturn Controller::zeroAxes(const Array<bool>& axes)
    {
        std::string command = "DP ";
        
        for (int i = 0; i < GALIL_NUM_AXES; i++)
            command += axes[i] ? std::string(1, axisName(i)) : "";
        
        this->command(command); // send the command
        
        return 0;
        
    } // GalilController::zeroAxes


} // namespace: Galil
