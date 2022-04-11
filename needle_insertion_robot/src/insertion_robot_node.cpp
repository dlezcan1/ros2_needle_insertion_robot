#include <cstdio>
#include <memory>

// rclcpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"

// ROS messages
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

// ROS services
#include "std_srvs/srv/trigger.hpp"

// custom headers
#include "NeedleInsertionRobot.h"

// helpful macros
#define ROBOT_BIND_AXIS_CMD_CB(msg_t, x, axis) (std::function<void(const msg_t)>) std::bind(&NeedleInsertionRobotNode::x, this, axis, std::placeholders::_1)
#define ROBOT_BIND_FN(x) std::bind(&NeedleInsertionRobotNode::x, this, std::placeholders::_1)
#define ROBOT_BIND_PUBLISHER(x) std::bind(&NeedleInsertionRobotNode::x, this)
#define ROBOT_BIND_SERVICE(x) std::bind(&NeedleInsertionRobotNode::x, this, std::placeholders::_1, std::placeholders::_2)
#define ROBOT_BIND_AXIS_SERVICE(srv_t, x, axis) (std::function <void (const srv_t::Request::SharedPtr, const srv_t::Response::SharedPtr)>) \
                                                 std::bind(&NeedleInsertionRobotNode::x, this, axis, std::placeholders::_1, std::placeholders::_2)

using namespace std::chrono_literals;
using std_srvs::srv::Trigger;
using std_msgs::msg::Bool;

class NeedleInsertionRobotNode : public rclcpp::Node
{
public:
    typedef std_msgs::msg::Float32 AxisMsg_t;
    
    
    NeedleInsertionRobotNode(const std::string& name = "NeedleInsertionRobot")
    : Node(name), m_robotAxesMoving({false, false, false, false})
    {
        // declare parameters
        std::string ip_address = this->declare_parameter("robot.ip_address", DEFAULT_GALIL_IP); // IP address of the robot
        
        // initialize the robot
        RCLCPP_INFO(this->get_logger(), "Connecting to Robot at IP Address: %s", ip_address.c_str());
        m_robot = std::make_shared<NeedleInsertionRobot>(ip_address.c_str());
        RCLCPP_INFO(this->get_logger(), "Connection established.");
        m_robot->allMotorsOff();
        
        // parameters
        float x_speed = this->declare_parameter( "axis.x.speed",        NeedleInsertionRobot::s_default_speed[0] );
        float x_accel = this->declare_parameter( "axis.x.acceleration", NeedleInsertionRobot::s_default_acceleration[0] );
        float x_decel = this->declare_parameter( "axis.x.deceleration", NeedleInsertionRobot::s_default_deceleration[0] );
        long  x_kp    = this->declare_parameter( "axis.x.kp",           NeedleInsertionRobot::s_default_kP[0] );
        long  x_ki    = this->declare_parameter( "axis.x.ki",           NeedleInsertionRobot::s_default_kI[0] );
        long  x_kd    = this->declare_parameter( "axis.x.kd",           NeedleInsertionRobot::s_default_kD[0] );

        float y_speed = this->declare_parameter( "axis.y.speed",        NeedleInsertionRobot::s_default_speed[1] );
        float y_accel = this->declare_parameter( "axis.y.acceleration", NeedleInsertionRobot::s_default_acceleration[1] );
        float y_decel = this->declare_parameter( "axis.y.deceleration", NeedleInsertionRobot::s_default_deceleration[1] );
        long  y_kp    = this->declare_parameter( "axis.y.kp",           NeedleInsertionRobot::s_default_kP[1] );
        long  y_ki    = this->declare_parameter( "axis.y.ki",           NeedleInsertionRobot::s_default_kI[1] );
        long  y_kd    = this->declare_parameter( "axis.y.kd",           NeedleInsertionRobot::s_default_kD[1] );

        float z_speed = this->declare_parameter( "axis.z.speed",        NeedleInsertionRobot::s_default_speed[2] );
        float z_accel = this->declare_parameter( "axis.z.acceleration", NeedleInsertionRobot::s_default_acceleration[2] );
        float z_decel = this->declare_parameter( "axis.z.deceleration", NeedleInsertionRobot::s_default_deceleration[2] );
        long  z_kp    = this->declare_parameter( "axis.z.kp",           NeedleInsertionRobot::s_default_kP[2] );
        long  z_ki    = this->declare_parameter( "axis.z.ki",           NeedleInsertionRobot::s_default_kI[2] );
        long  z_kd    = this->declare_parameter( "axis.z.kd",           NeedleInsertionRobot::s_default_kD[2] );

        float ls_speed = this->declare_parameter( "axis.linear_stage.speed",        NeedleInsertionRobot::s_default_speed[3] );
        float ls_accel = this->declare_parameter( "axis.linear_stage.acceleration", NeedleInsertionRobot::s_default_acceleration[3] );
        float ls_decel = this->declare_parameter( "axis.linear_stage.deceleration", NeedleInsertionRobot::s_default_deceleration[3] );
        long  ls_kp    = this->declare_parameter( "axis.linear_stage.kp",           NeedleInsertionRobot::s_default_kP[3] );
        long  ls_ki    = this->declare_parameter( "axis.linear_stage.ki",           NeedleInsertionRobot::s_default_kI[3] );
        long  ls_kd    = this->declare_parameter( "axis.linear_stage.kd",           NeedleInsertionRobot::s_default_kD[3] );

        float speed[] = { x_speed, y_speed, z_speed, ls_speed };
        float accel[] = { x_accel, y_accel, z_accel, ls_accel };
        float decel[] = { x_decel, y_decel, z_decel, ls_decel };
        long     kp[] = { x_kp   , y_kp   , z_kp   , ls_kp    };
        long     ki[] = { x_ki   , y_ki   , z_ki   , ls_ki    };
        long     kd[] = { x_kd   , y_kd   , z_kd   , ls_kd    };
        
        update_robotParams( speed, accel, decel, kp, ki, kd); // set Robot control parameters   

        // this->add_on_set_parameters_callback( ROBOT_BIND_FN(service_onSetParamsCallback) ); // DO NOT USE

        // initalize subscribers
        m_sub_AxCommandX  = this->create_subscription<AxisMsg_t>("axis/command/x",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0));
        m_sub_AxCommandY  = this->create_subscription<AxisMsg_t>("axis/command/y",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1));
        m_sub_AxCommandZ  = this->create_subscription<AxisMsg_t>("axis/command/z",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2));
        m_sub_AxCommandLS = this->create_subscription<AxisMsg_t>("axis/command/linear_stage", 1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3));
        
        // initalize publishers
        m_pub_AxPosX  = this->create_publisher<AxisMsg_t>("axis/position/x",            10);
        m_pub_AxPosY  = this->create_publisher<AxisMsg_t>("axis/position/y",            10);
        m_pub_AxPosZ  = this->create_publisher<AxisMsg_t>("axis/position/z",            10);
        m_pub_AxPosLS = this->create_publisher<AxisMsg_t>("axis/position/linear_stage", 10);

        m_pub_AxMovingX  = this->create_publisher<Bool>("axis/state/moving/x",            10);
        m_pub_AxMovingY  = this->create_publisher<Bool>("axis/state/moving/y",            10);
        m_pub_AxMovingZ  = this->create_publisher<Bool>("axis/state/moving/z",            10);
        m_pub_AxMovingLS = this->create_publisher<Bool>("axis/state/moving/linear_stage", 10);

        m_pub_AxStateX  = this->create_publisher<Bool>("axis/state/on/x",            10);
        m_pub_AxStateY  = this->create_publisher<Bool>("axis/state/on/y",            10);
        m_pub_AxStateZ  = this->create_publisher<Bool>("axis/state/on/z",            10);
        m_pub_AxStateLS = this->create_publisher<Bool>("axis/state/on/linear_stage", 10);

        // initialize services
        m_srv_abort         = this->create_service<Trigger>("abort",                    ROBOT_BIND_SERVICE(service_abort));
        m_srv_toggleAxisX   = this->create_service<Trigger>("axis/state/toggle/x",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 0));
        m_srv_toggleAxisY   = this->create_service<Trigger>("axis/state/toggle/y",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 1));
        m_srv_toggleAxisZ   = this->create_service<Trigger>("axis/state/toggle/z",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 2));
        m_srv_toggleAxisLS  = this->create_service<Trigger>("axis/state/toggle/linear_stage", ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 3));

        m_srv_zeroAxisX     = this->create_service<Trigger>("axis/zero/x",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   0));
        m_srv_zeroAxisY     = this->create_service<Trigger>("axis/zero/y",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   1));
        m_srv_zeroAxisZ     = this->create_service<Trigger>("axis/zero/z",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   2));
        m_srv_zeroAxisLS    = this->create_service<Trigger>("axis/zero/linear_stage",   ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   3));

        // create timers
        m_positionTimer  = this->create_wall_timer( 10ms, ROBOT_BIND_PUBLISHER(publish_CurrentPosition) );
        m_stateTimer     = this->create_wall_timer( 20ms, ROBOT_BIND_PUBLISHER(publish_CurrentState));
        
        RCLCPP_INFO(this->get_logger(), "Robot initialized and ready for operation.");
        
    } // Constructor
    ~NeedleInsertionRobotNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down robot node: '%s'", this->get_name());
        
	m_robot->allMotorsOff();
    };

private:
    /* Service callbacks */
    void service_abort(const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
    {
        RCLCPP_WARN(this->get_logger(), "Abort command triggered!");
        m_robot->abort();
        RCLCPP_WARN(this->get_logger(), "Aborted!");

        res->success = true;

    } // service_abort

    rcl_interfaces::msg::SetParametersResult service_onSetParamsCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        // setup of return message
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // initialize arrays
        float speed[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
        float accel[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
        float decel[] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};

        long kp[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};
        long ki[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};
        long kd[] = {NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS, NULL_LONG_AXIS};

        // TODO: need to add functionality for changing speed and gain params adaptively
        for (const auto& param : parameters )
        {
            // axis: x
            if(param.get_name().compare("axis.x.speed"))
                speed[0] = param.as_double();
            
            else if(param.get_name().compare("axis.x.acceleration"))
                accel[0] = param.as_double();

            else if(param.get_name().compare("axis.x.deceleration"))
                decel[0] = param.as_double();

            else if(param.get_name().compare("axis.x.kp"))
                kp[0] = param.as_int();

            else if(param.get_name().compare("axis.x.ki"))
                ki[0] = param.as_int();
                
            else if(param.get_name().compare("axis.x.kd"))
                kd[0] = param.as_int();

            // axis: y
            else if(param.get_name().compare("axis.y.speed"))
                speed[1] = param.as_double();
            
            else if(param.get_name().compare("axis.y.acceleration"))
                accel[1] = param.as_double();

            else if(param.get_name().compare("axis.y.deceleration"))
                decel[1] = param.as_double();

            else if(param.get_name().compare("axis.y.kp"))
                kp[1] = param.as_int();

            else if(param.get_name().compare("axis.y.ki"))
                ki[1] = param.as_int();

            else if(param.get_name().compare("axis.y.kd"))
                kd[1] = param.as_int();

            // axis: z
            else if(param.get_name().compare("axis.z.speed"))
                speed[2] = param.as_double();
            
            else if(param.get_name().compare("axis.z.acceleration"))
                accel[2] = param.as_double();

            else if(param.get_name().compare("axis.z.deceleration"))
                decel[2] = param.as_double();

            else if(param.get_name().compare("axis.z.kp"))
                kp[2] = param.as_int();
                
            else if(param.get_name().compare("axis.z.ki"))
                ki[2] = param.as_int();

            else if(param.get_name().compare("axis.z.kd"))
                kd[2] = param.as_int();
                
            // axis: linear_stage
            else if(param.get_name().compare("axis.linear_stage.speed"))
                speed[3] = param.as_double();
          
            else if(param.get_name().compare("axis.linear_stage.acceleration"))
                accel[3] = param.as_double();
                
            else if(param.get_name().compare("axis.linear_stage.deceleration"))
                decel[3] = param.as_double();
                
            else if(param.get_name().compare("axis.linear_stage.kp"))
                kp[3] = param.as_int();
                
            else if(param.get_name().compare("axis.linear_stage.ki"))
                ki[3] = param.as_int();
                
            else if(param.get_name().compare("axis.linear_stage.kd"))
                kd[3] = param.as_int();
            
            else
                result.successful = false;

        } // for

        // update the robot parameters
        if (result.successful)
            update_robotParams(speed, accel, decel, kp, ki, kd); 

        return result;


    } // service_onSetParamsCallback

    void service_toggleAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
    {
        bool command_positions[ROBOT_NUM_AXES] = { false, false, false, false };
        std::string axisName;

        switch(axis)
        {
            case 0: // axis X
                axisName = "X";
                break;
                
            case 1: // axis Y
                axisName = "Y";
                break;
                
            case 2: // axis Z
                axisName = "Z";
                break;
                
            case 3: // axis LS
                axisName = "LS";
                break;
                
            default:
                res->success = false;
                return;
                
        } // switch

        command_positions[axis] = true;

        if (m_robot->getMotorsOn()[axis])
        {
            RCLCPP_INFO(this->get_logger(), "Toggling axis %s off", axisName.c_str());
            m_robot->motorsOff(command_positions);
        
        } // if
        else
        {
            RCLCPP_INFO(this->get_logger(), "Toggling axis %s on", axisName.c_str());
            m_robot->motorsOn(command_positions);

        } // else
        
        // set result success
        res->success = true;

    } // service_toggleAxis

    void service_zeroAxis(int axis, const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
    {
        bool command_positions[ROBOT_NUM_AXES] = { false, false, false, false };
        std::string axisName;
        
        switch(axis)
        {
            case 0: // axis X
                axisName = "X";
                break;
                
            case 1: // axis Y
                axisName = "Y";
                break;
                
            case 2: // axis Z
                axisName = "Z";
                break;
                
            case 3: // axis LS
                axisName = "LS";
                break;
                
            default:
                res->success = false;
                return;
                
        } // switch

        command_positions[axis] = true;

        RCLCPP_INFO(this->get_logger(), "Zeroing axis %s.", axisName.c_str());

        res->success = true;

    } // service_zeroAxis


    /* Subscriber callbacks */
    void topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg)
    {
        std::string axisName;
        
        // perform command to specified axis
        float command_positions[ROBOT_NUM_AXES] = {NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS, NULL_FLOAT_AXIS};
        switch(axis)
        {
            case 0: // axis X
                axisName = "X";
                break;
                
            case 1: // axis Y
                axisName = "Y";
                break;
                
            case 2: // axis Z
                axisName = "Z";
                break;
                
            case 3: // axis LS
                axisName = "LS";
                break;
                
            default:
                return;
                
        } // switch
        
        float cmd_pos = msg->data;
        command_positions[axis] = cmd_pos;
        try
        {
            m_robot->moveAxes(command_positions, false);
            RCLCPP_INFO(this->get_logger(), "Commanding axis '%s' for %.2f mm", axisName.c_str(), cmd_pos);
            
            // m_robot->motionComplete(); // ensure motion is finished
            
        } // try
        catch( GReturn ec )
        {
            RCLCPP_WARN(this->get_logger(), "Moving axis throwing error code: %d! You may be publishing a command before motion has finished.", ec);
            return;

        } // catch
        
    } // topic_callbackAxisCommand

    /* Publisher functions */
    void publish_CurrentPosition() // publishes the position of the axes
    {
        auto msg_x  = AxisMsg_t();
        auto msg_y  = AxisMsg_t();
        auto msg_z  = AxisMsg_t();
        auto msg_ls = AxisMsg_t();

        // TODO: sample robot coordinates
        float* positions;
        try
        {
            positions = m_robot->getPosition(m_robotAxes, true);
        
        } // try
        catch(GReturn ec)
        {
            RCLCPP_WARN(this->get_logger(), "Error getting positions: error code = %d. Robot maybe buffered.", ec);
            return;
        
        } // catch
        catch(std::invalid_argument e)
        {
            RCLCPP_WARN(this->get_logger(), "Error getting positions: error = '%s'. Robot maybe buffered.", e.what());
            return;

        } // catch: invalid_argument

        RCLCPP_DEBUG(this->get_logger(), "Position: %.2f, %.2f, %.2f, %.2f", positions[0], positions[1], positions[2], positions[3]);

        // set the position data
        msg_x.data  = positions[0];
        msg_y.data  = positions[1];
        msg_z.data  = positions[2];
        msg_ls.data = positions[3];

        // publish
        m_pub_AxPosX  -> publish( msg_x );
        m_pub_AxPosY  -> publish( msg_y );
        m_pub_AxPosZ  -> publish( msg_z );
        m_pub_AxPosLS -> publish( msg_ls );
        
    } // publish_CurrentState

    void publish_CurrentState() // publishes the state of the axis
    {
        // get the current axis states
        const bool* axes_on;
        try
        {
            axes_on = m_robot->getMotorsOn();
        }
        catch (GReturn ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting which motors are on! Galil Error code = %d!", ec);
            return;

        } // catch
        
        const bool* axes_moving;
        try
        {
            axes_moving = m_robot->getAxesMoving();
        }
        catch (GReturn ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting which motors are moving! Galil Error code = %d!", ec);
            return;
            
        } // catch

        // setup the messages
        auto msg_on_x      = Bool();
        auto msg_on_y      = Bool();
        auto msg_on_z      = Bool();
        auto msg_on_ls     = Bool();

        auto msg_moving_x  = Bool();
        auto msg_moving_y  = Bool();
        auto msg_moving_z  = Bool();
        auto msg_moving_ls = Bool();

        // set the data for the messages
        msg_on_x.data      = axes_on[0];
        msg_on_y.data      = axes_on[1];
        msg_on_z.data      = axes_on[2];
        msg_on_ls.data     = axes_on[3];

        msg_moving_x.data  = axes_moving[0];
        msg_moving_y.data  = axes_moving[1];
        msg_moving_z.data  = axes_moving[2];
        msg_moving_ls.data = axes_moving[3];

        // publish the messages
        m_pub_AxStateX   -> publish( msg_on_x );
        m_pub_AxStateY   -> publish( msg_on_y );
        m_pub_AxStateZ   -> publish( msg_on_z );
        m_pub_AxStateLS  -> publish( msg_on_ls ); 

        m_pub_AxMovingX  -> publish( msg_moving_x );
        m_pub_AxMovingY  -> publish( msg_moving_y );
        m_pub_AxMovingZ  -> publish( msg_moving_z );
        m_pub_AxMovingLS -> publish( msg_moving_ls ); 

    } // publish_CurrentState

    void update_robotParams(float speed[ROBOT_NUM_AXES],
                            float accel[ROBOT_NUM_AXES],
                            float decel[ROBOT_NUM_AXES],
                            long     kp[ROBOT_NUM_AXES],
                            long     ki[ROBOT_NUM_AXES],
                            long     kd[ROBOT_NUM_AXES])
    {
        m_robot->setSpeed(speed);
        m_robot->setAcceleration(accel);
        m_robot->setDeceleration(decel);
        
        m_robot->setPID_P(kp);
        m_robot->setPID_I(ki);
        m_robot->setPID_D(kd);

        RCLCPP_INFO(this->get_logger(), "Updating Robot configuration parameters.");

    } // update_robotParams

private: // functions
    bool axisMoving(size_t axis){ return m_robot->getAxisMoving(axis); }
    
private: // members
    std::shared_ptr<NeedleInsertionRobot> m_robot;
    const bool m_robotAxes[4] = {true, true, true, true};     // Robot axes to use (use them all)
    bool m_robotAxesMoving[4]; // whether robot axes are moving or not
    
    // timers
    rclcpp::TimerBase::SharedPtr m_positionTimer, m_stateTimer;

    // subscribers
    rclcpp::Subscription<AxisMsg_t>::SharedPtr m_sub_AxCommandX, 
                                               m_sub_AxCommandY,
                                               m_sub_AxCommandZ, 
                                               m_sub_AxCommandLS;
    
    // publishers
    rclcpp::Publisher<AxisMsg_t>::SharedPtr m_pub_AxPosX, 
                                            m_pub_AxPosY, 
                                            m_pub_AxPosZ, 
                                            m_pub_AxPosLS;
    
    rclcpp::Publisher<Bool>::SharedPtr m_pub_AxStateX,
                                       m_pub_AxStateY,
                                       m_pub_AxStateZ,
                                       m_pub_AxStateLS,
                                       
                                       m_pub_AxMovingX,
                                       m_pub_AxMovingY,
                                       m_pub_AxMovingZ,
                                       m_pub_AxMovingLS;

    // services
    rclcpp::Service<Trigger>::SharedPtr m_srv_abort,
                                                       
                                        m_srv_toggleAxisX,
                                        m_srv_toggleAxisY,
                                        m_srv_toggleAxisZ,
                                        m_srv_toggleAxisLS,
                                        
                                        m_srv_zeroAxisX,
                                        m_srv_zeroAxisY,
                                        m_srv_zeroAxisZ,
                                        m_srv_zeroAxisLS;
                                        
}; // class: NeedleInsertionRobotNode

/* =================== MAIN METHOD ======================================*/

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<NeedleInsertionRobotNode>());
    rclcpp::shutdown();
    
    return 0;
    
} // main
