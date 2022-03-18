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
        RCLCPP_INFO(this->get_logger(), "Galil IP address: %s", ip_address.c_str());

        // initialize the robot
//        m_robot = std::make_shared<NeedleInsertionRobot>(ip_address.c_str());
        
        // initalize subscribers
        m_subAxesX  = this->create_subscription<AxisMsg_t>("/stage/command/axis/x",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 0));
        m_subAxesY  = this->create_subscription<AxisMsg_t>("/stage/command/axis/y",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 1));
        m_subAxesZ  = this->create_subscription<AxisMsg_t>("/stage/command/axis/z",            10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 2));
        m_subAxesLS = this->create_subscription<AxisMsg_t>("/stage/command/axis/linear_stage", 10, ROBOT_BIND_AXIS_CMD_CB(AxisMsg_t::SharedPtr, topic_callbackAxisCommand, 3));
        
        // initalize publishers
        m_pubAxesX  = this->create_publisher<AxisMsg_t>("/stage/state/axis/x",            10);
        m_pubAxesY  = this->create_publisher<AxisMsg_t>("/stage/state/axis/y",            10);
        m_pubAxesZ  = this->create_publisher<AxisMsg_t>("/stage/state/axis/z",            10);
        m_pubAxesLS = this->create_publisher<AxisMsg_t>("/stage/state/axis/linear_stage", 10);
        
        // create timers
//        m_timer = this->create_wall_timer( 1000ms, [this](){ RCLCPP_INFO(this->get_logger(), "Running..."); } );
        m_timer = this->create_wall_timer( 1000ms, ROBOT_BIND_PUBLISHER(publish_CurrentState) );
        
        
    } // Constructor
    ~NeedleInsertionRobotNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down robot node: '%s'", this->get_name());
        
        // release subscriptions
        m_subAxesX.reset();
        m_subAxesY.reset();
        m_subAxesZ.reset();
        m_subAxesLS.reset();
        
        // release publishers
        m_pubAxesX.reset();
        m_pubAxesY.reset();
        m_pubAxesZ.reset();
        m_pubAxesLS.reset();
        
        // release robot pointer
//        m_robot.reset();
    };

private:
    /* Subscriber callbakcs*/
    void topic_callbackString(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    void topic_callbackAxisCommand(int axis, const AxisMsg_t::SharedPtr msg) const
    {
        std::string axisName;
        
        // TODO: perform command to specified axis
        
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
                axisName = "Not Found";
                
        } // switch
        
        float cmd_pos = msg->data;
        RCLCPP_INFO(this->get_logger(), "Commanding axis '%s' for '%.2f' mm", axisName.c_str(), cmd_pos);
        
        
    } // topic_callbackAxisCommand
    
    /* Publisher functions */
    void publish_CurrentState()
    {
        auto msg = std_msgs::msg::Float32();
        
        // TODO: sample robot coordinates
        msg.data = 0;
        m_pubAxesX->publish(msg);
        m_pubAxesY->publish(msg);
        m_pubAxesZ->publish(msg);
        m_pubAxesLS->publish(msg);
        
        
    } // publish_CurrentState
    
private: // members
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subString;
//    std::shared_ptr<NeedleInsertionRobot> m_robot;
    
    // timers
    rclcpp::TimerBase::SharedPtr m_timer;
    
    // subscribers
    rclcpp::Subscription<AxisMsg_t>::SharedPtr m_subAxesX, m_subAxesY, m_subAxesZ, m_subAxesLS;
    
    rclcpp::Publisher<AxisMsg_t>::SharedPtr m_pubAxesX, m_pubAxesY, m_pubAxesZ, m_pubAxesLS;
    
}; // class: NeedleInsertionRobotNode

/* =================== MAIN METHOD ======================================*/

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<NeedleInsertionRobotNode>());
    rclcpp::shutdown();
    
    return 0;
    
} // main
