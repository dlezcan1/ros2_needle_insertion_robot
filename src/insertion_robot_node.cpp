#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "NeedleInsertionRobot.h"

#define ROBOT_BIND_PUBLISHER(x) std::bind(&NeedleInsertionRobotNode::x, this)
#define ROBOT_BIND_FN(x) std::bind(&NeedleInsertionRobotNode::x, this, std::placeholders::_1)
#define ROBOT_BIND_AXIS_CMD_CB(msg_t, x, axis) (std::function<void(const msg_t)>) std::bind(&NeedleInsertionRobotNode::x, this, axis, std::placeholders::_1)

using namespace std::chrono_literals;

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
        m_robot->allMotorsOff();
        RCLCPP_INFO(this->get_logger(), "Connection established.");
        
        // initalize subscribers
        m_sub_AxCommandX  = this->create_subscription<AxisMsg_t>("stage/command/axis/x",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0));
        m_sub_AxCommandY  = this->create_subscription<AxisMsg_t>("stage/command/axis/y",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1));
        m_sub_AxCommandZ  = this->create_subscription<AxisMsg_t>("stage/command/axis/z",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2));
        m_sub_AxCommandLS = this->create_subscription<AxisMsg_t>("stage/command/axis/linear_stage", 10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3));
        
        // initalize publishers
        m_pub_AxStateX  = this->create_publisher<AxisMsg_t>("stage/state/axis/x",            10);
        m_pub_AxStateY  = this->create_publisher<AxisMsg_t>("stage/state/axis/y",            10);
        m_pub_AxStateZ  = this->create_publisher<AxisMsg_t>("stage/state/axis/z",            10);
        m_pub_AxStateLS = this->create_publisher<AxisMsg_t>("stage/state/axis/linear_stage", 10);
        
        // create timers
        m_stateTimer = this->create_wall_timer( 1000ms, ROBOT_BIND_PUBLISHER(publish_CurrentState) );
        
        
    } // Constructor
    ~NeedleInsertionRobotNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down robot node: '%s'", this->get_name());
        
        // release subscriptions
        m_sub_AxCommandX.reset();
        m_sub_AxCommandY.reset();
        m_sub_AxCommandZ.reset();
        m_sub_AxCommandLS.reset();
        
        // release publishers
        m_pub_AxStateX.reset();
        m_pub_AxStateY.reset();
        m_pub_AxStateZ.reset();
        m_pub_AxStateLS.reset();

        // release timers
        m_stateTimer.reset();
        
        // release robot pointer
//        m_robot.reset();
    };

private:
    /* Subscriber callbakcs*/
    void topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg) const
    {
        std::string axisName;
        
        // TODO: perform command to specified axis
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
        RCLCPP_INFO(this->get_logger(), "Commanding axis '%s' for %.2f mm", axisName.c_str(), cmd_pos);
        command_positions[axis] = cmd_pos;

        m_robot->moveAxes(command_positions, false);
        
    } // topic_callbackAxisCommand
    
    /* Publisher functions */
    void publish_CurrentState()
    {
        auto msg_x  = std_msgs::msg::Float32();
        auto msg_y  = std_msgs::msg::Float32();
        auto msg_z  = std_msgs::msg::Float32();
        auto msg_ls = std_msgs::msg::Float32();

        // TODO: sample robot coordinates
        float* positions = m_robot->getPosition(m_robotAxes, false);
        msg_x.data  = positions[0];
        msg_y.data  = positions[1];
        msg_z.data  = positions[2];
        msg_ls.data = positions[3];

        // publish
        m_pub_AxStateX  -> publish(msg_x);
        m_pub_AxStateY  -> publish(msg_y);
        m_pub_AxStateZ  -> publish(msg_z);
        m_pub_AxStateLS -> publish(msg_ls);
      
        
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
    
    rclcpp::Publisher<AxisMsg_t>::SharedPtr m_pub_AxStateX, 
                                            m_pub_AxStateY, 
                                            m_pub_AxStateZ, 
                                            m_pub_AxStateLS;
    
}; // class: NeedleInsertionRobotNode

/* =================== MAIN METHOD ======================================*/

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<NeedleInsertionRobotNode>());
    rclcpp::shutdown();
    
    return 0;
    
} // main
