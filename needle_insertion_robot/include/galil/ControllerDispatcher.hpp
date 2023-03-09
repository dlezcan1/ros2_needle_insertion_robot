//
//  ControllerDispatcher.hpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#pragma once
#include "Controller.hpp"

#include <array>

namespace Galil
{
    enum Command
    {
        None = 0,
        
        GetPositionRelative,
        GetPositionAbsolute,
        GetPosition = GetPositionAbsolute,
        
        MoveAbsolute,
        MoveRelative,
        
        StopAxes,
        ZeroAxes,
        
        MotorsOn,
        MotorsOff,
        
        SetAcceleration,
        SetDeceleration,
        SetSpeed,
        
        SetPID_P,
        SetPID_I,
        SetPID_D,
        
    }; // enum: Command
    
    struct DispatcherCommand
    {
        Command command = Command::None;
        std::array<long, GALIL_NUM_AXES> values;
        std::array<bool, GALIL_NUM_AXES> switches;
        
        // constuctors
        DispatcherCommand()
        {
            values.fill(NULL_LONG_AXIS);
            switches.fill(false);
            
        } // constructor
        
        DispatcherCommand(Command command) : command(command)
        {
            values.fill(NULL_LONG_AXIS);
            switches.fill(false);
            
        } // constructor
        
        DispatcherCommand(Command command, const std::array<long, GALIL_NUM_AXES>& values) : command(command), values(values)
        {
            switches.fill(false);
            
        } // constructor
        
        DispatcherCommand(Command command, const std::array<bool, GALIL_NUM_AXES>& switches) : command(command), switches(switches)
        {
            values.fill( NULL_LONG_AXIS );
            
        } // constructor
        
        
        
    }; // struct: DispatcherCommand
    
    struct DispatcherReturn
    {
        std::string returnType;
        long axesPositions[GALIL_NUM_AXES] = {NULL_LONG_AXIS,NULL_LONG_AXIS,NULL_LONG_AXIS,NULL_LONG_AXIS,NULL_LONG_AXIS};
        bool axesSwitches[GALIL_NUM_AXES] = {false, false, false, false, false};
        
        void setPositions(long* positions)
        {
            for (size_t i = 0; i < GALIL_NUM_AXES; i++)
                axesPositions[i] = positions[i];
            
        } // setPositions
        
        void setSwitches(bool* switches)
        {
            for (size_t i = 0; i < GALIL_NUM_AXES; i++)
                axesSwitches[i] = switches[i];
            
        } // setSwitches
        
    }; // struct: DispatcherReturn
    
    
    class ControllerDispatcher
    {
    public:
        // constructors
        ControllerDispatcher(std::string& ip_address);
        ControllerDispatcher(std::shared_ptr<Controller> controller);
        
        virtual ~ControllerDispatcher() = default;
        
        // set the command type
        
        
        // galil command interface
        void abort() { m_controller->abort(); }
        void allMotorsOff() { m_controller->allMotorsOff(); }
        
        // dispatch the command to the galil controller
        DispatcherReturn dispatch();
        
        Command getCurrentCommmand() const { return m_currentCommand; }
        
        /** Set the current axis value you would like to input
         
         */
        bool setAxisValue(size_t axisIndex, long value);
        
        /** Set the current axis toggle you would like to input
         
         */
        bool setAxisToggle(size_t axisIndex);
        
        /** Set the current command
                          
            If the command type is a new command, it will clear all buffered data.
            
         */
        void setGalilCommand(Command cmd_type);
        
        
        
    private:
        std::shared_ptr<Controller> m_controller;
        
        Command m_currentCommand = Command::GetPosition;
        
        long m_axesValues[GALIL_NUM_AXES];
        bool m_axesSwitches[GALIL_NUM_AXES];
        
    // private members
        
    private:
        void clearAxes(); // wipe the axes clear
        
        DispatcherReturn dispatchCommand();
        
        
    }; // class: ControllerDispatcher
} // namespace: Galil
