//
//  ControllerDispatcher.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#include "galil/ControllerDispatcher.hpp"

namespace Galil
{
    
    ControllerDispatcher::ControllerDispatcher(std::string& ip_address): ControllerDispatcher::ControllerDispatcher(std::make_shared<Controller>(ip_address.c_str()))
    {
        
    } // default cosntructor
    
    ControllerDispatcher::ControllerDispatcher(std::shared_ptr<Controller> controller) :  m_controller(controller)
    {
        clearAxes();
        
    } // argument cosntructor
    
    void ControllerDispatcher::clearAxes()
    {
        for (size_t i = 0; i < GALIL_NUM_AXES; i++)
        {
            m_axesValues[i] = NULL_LONG_AXIS;
            m_axesSwitches[i] = false;
        }
    } // ControllerDispatcher::clearAxes
    
    DispatcherReturn ControllerDispatcher::dispatch()
    {
        DispatcherReturn retval = dispatchCommand();
        
        // empty the current axes containers
        clearAxes();
        
        return retval;
        
    } // ControllerDispatcher::dispatch
    
    DispatcherReturn ControllerDispatcher::dispatchCommand()
    {
        DispatcherReturn retval;
        
        // handle the command
        switch (m_currentCommand)
        {
            // get position commands
            case Command::GetPositionAbsolute:
                retval.setPositions(m_controller->getPosition( m_axesSwitches, true  ));
                break;
                
            case Command::GetPositionRelative:
                retval.setPositions(m_controller->getPosition( m_axesSwitches, false ));
                break;
                
            // move commands
            case Command::MoveAbsolute:
                m_controller->moveAxes( m_axesValues, true  );
                
                for(size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis(m_axesValues[i]);
                        
                break;
                
            case Command::MoveRelative:
                m_controller->moveAxes( m_axesValues, false );
                
                for(size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            // stop and zero axes commands
            case Command::StopAxes:
                m_controller->stopAxes( m_axesSwitches );
                
                retval.setSwitches( m_axesSwitches );
                
                break;
                
            case Command::ZeroAxes:
                m_controller->zeroAxes( m_axesSwitches );
                
                retval.setSwitches( m_axesSwitches );
                
                break;
                
            // motors on and off commands
            case Command::MotorsOn:
                m_controller->motorsOn( m_axesSwitches );
                
                retval.setSwitches( m_axesSwitches );
                
                break;
                
            case Command::MotorsOff:
                m_controller->motorsOff( m_axesSwitches );
                
                retval.setSwitches( m_axesSwitches );
                
                break;
                
            // set speed commands
            case Command::SetAcceleration:
                m_controller->setAcceleration( m_axesValues );
                
                for(size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            case Command::SetDeceleration:
                m_controller->setDeceleration( m_axesValues );
                
                for(size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            case Command::SetSpeed:
                m_controller->setSpeed( m_axesValues );
                
                for(size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            // set PID commands
            case Command::SetPID_P:
                m_controller->setPID_P( m_axesValues );
                
                for (size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            case Command::SetPID_I:
                m_controller->setPID_I( m_axesValues );
                
                for (size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
            case Command::SetPID_D:
                m_controller->setPID_D( m_axesValues );
                
                for (size_t i = 0; i < GALIL_NUM_AXES; i++)
                    retval.axesSwitches[i] = !isNullAxis( m_axesValues[i] );
                
                break;
                
        } // switch
        
        return retval;
        
    } // ControllerDispatcher::dispatch
    
    void ControllerDispatcher::setGalilCommand(Command cmd_type)
    {
        if (cmd_type == m_currentCommand)
            return; // do nothing
        
        m_currentCommand = cmd_type;
        clearAxes();
        
    } // ControllerDispatcher::setGalilCommand
    
    bool ControllerDispatcher::setAxisValue(size_t axisIndex, long value)
    {
        if (axisIndex < 0 || axisIndex >= GALIL_NUM_AXES)
            return false;
        
        m_axesValues[axisIndex] = value;
        
        return true;
        
    } // ControllerDispatcher::setAxisValue
    
    
    bool ControllerDispatcher::setAxisToggle(size_t axisIndex)
    {
        if (axisIndex < 0 || axisIndex >= GALIL_NUM_AXES)
            return false;
        
        m_axesSwitches[axisIndex] = true;
        
        return true;
        
    } // ControllerDispatcher::setAxisToggle
    
} // namespace: Galil
