#include <cstdio>
#include <memory>

// rclcpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

// ROS messages
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

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

class NeedleInsertionRobotNode : public rclcpp::Node
{
public:
    typedef std_msgs::msg::Float32 AxisMsg_t;
    
    
    NeedleInsertionRobotNode(const std::string& name = "NeedleInsertionRobot")
    : Node(name)
    {
        // declare parameters
        std::string ip_address = this->declare_parameter("robot.ipAddress", DEFAULT_GALIL_IP); // IP address of the robot
        
        // initialize the robot
        RCLCPP_INFO(this->get_logger(), "Connecting to Robot at IP Address: %s", ip_address.c_str());
        m_robot = std::make_shared<NeedleInsertionRobot>(ip_address.c_str());
        RCLCPP_INFO(this->get_logger(), "Connection established.");
        m_robot->allMotorsOff();
        
        // initalize subscribers
        m_sub_AxCommandX  = this->create_subscription<AxisMsg_t>("stage/command/axis/x",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0));
        m_sub_AxCommandY  = this->create_subscription<AxisMsg_t>("stage/command/axis/y",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1));
        m_sub_AxCommandZ  = this->create_subscription<AxisMsg_t>("stage/command/axis/z",            1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2));
        m_sub_AxCommandLS = this->create_subscription<AxisMsg_t>("stage/command/axis/linear_stage", 1, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3));
        
        // initalize publishers
        m_pub_AxStateX  = this->create_publisher<AxisMsg_t>("stage/state/axis/x",            10);
        m_pub_AxStateY  = this->create_publisher<AxisMsg_t>("stage/state/axis/y",            10);
        m_pub_AxStateZ  = this->create_publisher<AxisMsg_t>("stage/state/axis/z",            10);
        m_pub_AxStateLS = this->create_publisher<AxisMsg_t>("stage/state/axis/linear_stage", 10);

        // initialize services
        m_srv_abort         = this->create_service<Trigger>("stage/abort",                    ROBOT_BIND_SERVICE(service_abort));
        m_srv_toggleAxisX   = this->create_service<Trigger>("stage/toggle/axis/x",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 0));
        m_srv_toggleAxisY   = this->create_service<Trigger>("stage/toggle/axis/y",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 1));
        m_srv_toggleAxisZ   = this->create_service<Trigger>("stage/toggle/axis/z",            ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 2));
        m_srv_toggleAxisLS  = this->create_service<Trigger>("stage/toggle/axis/linear_stage", ROBOT_BIND_AXIS_SERVICE(Trigger, service_toggleAxis, 3));

        m_srv_zeroAxisX     = this->create_service<Trigger>("stage/zero/axis/x",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   0));
        m_srv_zeroAxisY     = this->create_service<Trigger>("stage/zero/axis/y",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   1));
        m_srv_zeroAxisZ     = this->create_service<Trigger>("stage/zero/axis/z",              ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   2));
        m_srv_zeroAxisLS    = this->create_service<Trigger>("stage/zero/axis/linear_stage",   ROBOT_BIND_AXIS_SERVICE(Trigger, service_zeroAxis,   3));

        // create timers
        m_stateTimer = this->create_wall_timer( 1000ms, ROBOT_BIND_PUBLISHER(publish_CurrentState) );
        
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
        RCLCPP_INFO(this->get_logger(), "Abort command triggered!");
        m_robot->abort();
        RCLCPP_INFO(this->get_logger(), "Aborted!");

        res->success = true;

    } // service_abort

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
    void topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg) const
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
            m_robot->motionComplete(); // ensure motion is finished

        } // try
        catch( int ec )
        {
            RCLCPP_WARN(this->get_logger(), "Moving axis throwing error code: %d! You may be publishing a command before motion has finished.", ec);
            return;
        } // catch
        
    } // topic_callbackAxisCommand
    
    /* Publisher functions */
    void publish_CurrentState()
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
        catch(int ec)
        {
            RCLCPP_WARN(this->get_logger(), "Error getting positions: error code = %d. Robot maybe buffered.", ec);
            return;
        
        } // catch

        // RCLCPP_INFO(this->get_logger(), "Position: %.2f, %.2f, %.2f, %.2f", positions[0], positions[1], positions[2], positions[3]);
        msg_x.data  = positions[0];
        msg_y.data  = positions[1];
        msg_z.data  = positions[2];
        msg_ls.data = positions[3];

        // publish
        m_pub_AxStateX  -> publish(msg_x);
        m_pub_AxStateY  -> publish(msg_y);
        m_pub_AxStateLS -> publish(msg_ls);
        m_pub_AxStateZ  -> publish(msg_z);
      
        
    } // publish_CurrentState
    
private: // members
    std::shared_ptr<NeedleInsertionRobot> m_robot;
    const bool m_robotAxes[4] = {true, true, true, true};
    
    // timers
    rclcpp::TimerBase::SharedPtr m_stateTimer;
    
    // subscribers
    rclcpp::Subscription<AxisMsg_t>::SharedPtr m_sub_AxCommandX, 
                                               m_sub_AxCommandY,
                                               m_sub_AxCommandZ, 
                                               m_sub_AxCommandLS;
    
    // publishers
    rclcpp::Publisher<AxisMsg_t>::SharedPtr m_pub_AxStateX, 
                                            m_pub_AxStateY, 
                                            m_pub_AxStateZ, 
                                            m_pub_AxStateLS;

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
