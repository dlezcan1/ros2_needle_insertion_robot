# standard libraries
from functools import partial
import math
import asyncio
import time


# ros2 libraries
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

# actions 
from needle_insertion_robot_translation_interfaces.action import MoveStage

# msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32

# services
from std_srvs.srv import Trigger

class CoordinateConversions:
    """
        Translation configuration

        Axis directions:
            Robot -->   Stage         | Physical Motion (facing needle insertion direction)
            --------------------------------------------------------------------------------
              x   -->     y           |   insert needle
              y   -->    -x           |   left
             -z   -->     z           |   up

    """
    @staticmethod
    def RobotAxisPositionsToRobot(x_ax, y_ax, z_ax, ls_ax):
        """ convert the Robot Axis Positions to Robot Coordinates"""
        x_r = x_ax - ls_ax
        y_r = y_ax
        z_r = z_ax

        return x_r, y_r, z_r
        
    # RobotAxisPositionsToRobot

    @staticmethod
    def RobotAxisPositionsToStage(x_ax, y_ax, z_ax, ls_ax):
        """ convert the Robot Axis Positions to Robot Coordinates"""
        x_r, y_r, z_r = CoordinateConversions.RobotAxisPositionsToRobot(x_ax, y_ax, z_ax, ls_ax)

        return CoordinateConversions.RobotToStage(x_r, y_r, z_r)
        
    # RobotAxisPositionsToRobot

    @staticmethod
    def RobotToStage(x_r, y_r, z_r):
        """ Convert robot coordinates to the stage coordinates """
        x_s =  y_r # * 1e-3 # convert mm -> m
        y_s =  z_r # * 1e-3 # convert mm -> m
        z_s =  x_r # * 1e-3 # convert mm -> m

        return x_s, y_s, z_s

    # RobotToStage

    @staticmethod
    def StageToRobot(x_s, y_s, z_s):
        """ Convert Stage to Robot Coordinates """
        x_r =  z_s # * 1e3 # convert m -> mm
        y_r =  x_s # * 1e3 # convert m -> mm
        z_r =  y_s # * 1e3 # convert m -> mm

        return x_r, y_r, z_r

    # StageToRobot


# class: 

class NeedleInsertionRobotAxisMoving(Node):
    def __init__( self, name = "NeedleInsertionRobotAxisMovingNode", ns: str = "" ):
        super().__init__(name)

        self.robot_axisMoving = [ False, False, False, False ]
        
        self.sub_robot_axisMoving_x  = self.create_subscription(
            Bool,
            "axis/moving/x",
            lambda msg: self.sub_axisMoving_callback(msg, axis=0),
            10,
        )
        self.sub_robot_axisMoving_y  = self.create_subscription(
            Bool,
            "axis/moving/y",
            lambda msg: self.sub_axisMoving_callback(msg, axis=1),
            10,
        )
        self.sub_robot_axisMoving_z  = self.create_subscription(
            Bool,
            "axis/moving/z",
            lambda msg: self.sub_axisMoving_callback(msg, axis=2),
            10,
        )
        self.sub_robot_axisMoving_ls = self.create_subscription(
            Bool,
            "axis/moving/linear_stage",
            lambda msg: self.sub_axisMoving_callback(msg, axis=3),
            10,
        )

    # __init__

    def sub_axisMoving_callback(self, msg: Bool, axis: int):
        self.robot_axisMoving[axis] = msg.data

    # sub_axisMoving_callback

# class: NeedleInsertionRobotAxisMoving


class NeedleInsertionRobotActionServer(Node):
    def __init__(self, name = "NeedleInsertionRobotActionServerNode" ):
        super().__init__( name )

        # parameters
        self.actionsrv_translation_running = False
        # - stored robot states
        self.robot_axisPositions = [   0.0,   0.0,   0.0,   0.0 ] # current robot axis positions

        # publishers
        # - robot command
        self.pub_robot_axisCommand_x  = self.create_publisher( Float32, "axis/command/absolute/x",            1 )
        self.pub_robot_axisCommand_y  = self.create_publisher( Float32, "axis/command/absolute/y",            1 )
        self.pub_robot_axisCommand_z  = self.create_publisher( Float32, "axis/command/absolute/z",            1 )
        self.pub_robot_axisCommand_ls = self.create_publisher( Float32, "axis/command/absolute/linear_stage", 1 )

        # clients
        self.cli_robot_abort          = self.create_client( Trigger, "abort" )

        # action servers
        self.actionsrv_translation = ActionServer(self, MoveStage, "move_stage",
                                                  execute_callback = self.action_translation_execute_callback,
                                                  callback_group   = MutuallyExclusiveCallbackGroup(),
                                                  goal_callback    = self.action_translation_goal_callback,
                                                  cancel_callback  = self.action_translation_cancel_callback
                                                )
        # subscribers
        # - helper robot axis moving node
        self._robot_axisMoving_node = NeedleInsertionRobotAxisMoving(
            f"{self.get_name()}_Helper",
            ns=self.get_namespace()
        )

        # inform node is running
        self.get_logger().info("Running needle insertion robot action server.")
        
    # __init__

    @property
    def robot_axisMoving(self):
        return self._robot_axisMoving_node.robot_axisMoving

    def action_translation_cancel_callback( self, goal_handle: ServerGoalHandle ):
        """ Cancel the current action """
        # abort robot motion
        self.cli_robot_abort.call()

        goal_handle.abort()
        
        return CancelResponse.ACCEPT

    # action_translation_cancel_callback

    async def action_translation_execute_callback( self, goal_handle: ServerGoalHandle ) :
        """ Handle the goal """
        self.get_logger().info(
            f"Executing goal to axis: "
            f"x={goal_handle.request.x} mm, "
            f"y={goal_handle.request.y} mm, "
            f"z={goal_handle.request.z} mm, "
            f"ls={goal_handle.request.linear_stage} mm..."
        )

        # create lambda error function
        compute_error = lambda current_x, current_y, current_z, current_ls: math.sqrt( 
            abs(current_x - goal_handle.request.x) * float(goal_handle.request.move_x)
            + abs(current_y - goal_handle.request.y) * float(goal_handle.request.move_y)
            + abs(current_z - goal_handle.request.z) * float(goal_handle.request.move_z)
            + abs(current_ls - goal_handle.request.linear_stage) * float(goal_handle.request.move_linear_stage)
        )
        
        # command the robot to goal position
        self.actionsrv_translation_running = True  # action server is running
        if goal_handle.request.move_x:
            self.pub_robot_axisCommand_x.publish( Float32( data=goal_handle.request.x ) )   # command x-axis
        if goal_handle.request.move_y:
            self.pub_robot_axisCommand_y.publish( Float32( data=goal_handle.request.y ) )   # command y-axis
        if goal_handle.request.move_z: 
            self.pub_robot_axisCommand_z.publish( Float32( data=goal_handle.request.z ) )   # command z-axis
        if goal_handle.request.move_linear_stage:
            self.pub_robot_axisCommand_ls.publish( Float32( data=goal_handle.request.linear_stage ) ) # command linear stage-axis 
        
        time.sleep(0.5)

         # provide feedback
        feedback_msg = MoveStage.Feedback()
        while any(self.robot_axisMoving): # check if any axis is moving
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                # setup the result
                result_msg              = MoveStage.Result()
                result_msg.x            = float( self.robot_axisPositions[0] )
                result_msg.y            = float( self.robot_axisPositions[1] )
                result_msg.z            = float( self.robot_axisPositions[2] )
                result_msg.linear_stage = float( self.robot_axisPositions[3] )
                result_msg.error        = compute_error( result_msg.x, result_msg.y, result_msg.z, result_msg.linear_stage )

                return result_msg

            # if 

            feedback_msg.x            = float( self.robot_axisPositions[0] )
            feedback_msg.y            = float( self.robot_axisPositions[1] )
            feedback_msg.z            = float( self.robot_axisPositions[2] )
            feedback_msg.linear_stage = float( self.robot_axisPositions[3] )
            time_now                  = self.get_clock().now().seconds_nanoseconds()
            feedback_msg.time         = float( time_now[0]*1e9 + time_now[1] )
            feedback_msg.error        = compute_error(feedback_msg.x, feedback_msg.y, feedback_msg.z, feedback_msg.linear_stage)
        
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug(f"Publishing feedback: x: {feedback_msg.x} m, z: {feedback_msg.z} m, error {feedback_msg.error} m ")
            self.get_logger().debug(f"Robot axes moving: {self.robot_axisMoving}")

        # while

        # setup the result
        result_msg              = MoveStage.Result()
        result_msg.x            = float( self.robot_axisPositions[0] )
        result_msg.y            = float( self.robot_axisPositions[1] )
        result_msg.z            = float( self.robot_axisPositions[2] )
        result_msg.linear_stage = float( self.robot_axisPositions[3] )
        result_msg.error        = compute_error( result_msg.x, result_msg.y, result_msg.z, result_msg.linear_stage )

        self.actionsrv_translation_running = False # action server finished
        
        if result_msg.error <= goal_handle.request.eps:
            goal_handle.succeed()

        else: # only attempt one command
            goal_handle.abort()
            
        self.get_logger().info(f"Finished execution with following: x={result_msg.x} m, z={result_msg.z} m, error={result_msg.error} m")

        return result_msg

    # action_callback_translation

    def action_translation_goal_callback( self, goal_handle: ServerGoalHandle ):
        """ This action server will only handle one action at a time """
        # check if the action server is running right now or not.        
        if self.actionsrv_translation_running or any(self.robot_axisMoving):
            response = GoalResponse.REJECT

        else:
            response = GoalResponse.ACCEPT

        return response

    # action_translation_goal_callback

    def configure(self):
        """ Configure the node """
        self.executor.add_node(self._robot_axisMoving_node)

    # configure

    def destroy_node( self ) -> bool:
        self.get_logger().info( "Shutting down needle insertion robot action server." )
        
        return super().destroy_node()

    # destroy_node

    def get_robotCoordinates(self):
        """ Get the current robot coordinates """
        
        return CoordinateConversions.RobotAxisPositionsToRobot(self.robot_axisPositions[0],
                                                               self.robot_axisPositions[1],
                                                               self.robot_axisPositions[2],
                                                               self.robot_axisPositions[3]
                                                            )

    # get_robotCoordinates

    def get_stageCoordinates(self):
        """ Get the current stage coordinates """
        x_r, y_r, z_r  = self.get_robotCoordinates()

        return CoordinateConversions.RobotToStage(x_r, y_r, z_r)

    # get_stageCoordinates

# class: NeedleInsertionRobotActionServer


class NeedleInsertionRobotTranslationNode(Node):
    
    def __init__( self, name = "NeedleInsertionRobotTranslationNode", action_server: NeedleInsertionRobotActionServer=None ):
        super().__init__( name )

        # parameters
        self.action_server = action_server # the access to the access server
        
        # - stored robot states
        self.robot_axisPositions = [   0.0,   0.0,   0.0,   0.0 ] # current robot axis positions
        self.robot_axisMoving    = [ False, False, False, False ] # whether the robot is moving or not

        # subscriptions
        # - robot axis positions
        self.sub_robot_axisposition_x  = self.create_subscription( Float32, "axis/position/x"           , partial( self.sub_axisPosition_callback, axis=0 ), 1 )
        self.sub_robot_axisposition_y  = self.create_subscription( Float32, "axis/position/y"           , partial( self.sub_axisPosition_callback, axis=1 ), 1 )
        self.sub_robot_axisposition_z  = self.create_subscription( Float32, "axis/position/z"           , partial( self.sub_axisPosition_callback, axis=2 ), 1 )
        self.sub_robot_axisposition_ls = self.create_subscription( Float32, "axis/position/linear_stage", partial( self.sub_axisPosition_callback, axis=3 ), 1 )

        # - robot moving states
        self.sub_robot_axismoving_x  = self.create_subscription( Bool, "axis/state/moving/x",            partial(self.sub_axisMoving_callback, axis=0), 1)
        self.sub_robot_axismoving_y  = self.create_subscription( Bool, "axis/state/moving/y",            partial(self.sub_axisMoving_callback, axis=1), 1)
        self.sub_robot_axismoving_z  = self.create_subscription( Bool, "axis/state/moving/z",            partial(self.sub_axisMoving_callback, axis=2), 1)
        self.sub_robot_axismoving_ls = self.create_subscription( Bool, "axis/state/moving/linear_stage", partial(self.sub_axisMoving_callback, axis=3), 1)

        # publishers
        # - robot state (translation)
        self.pub_needlepose = self.create_publisher( PoseStamped, "state/needle_pose", 10 )

        # timers
        self.timer_publish_needlepose = self.create_timer( 0.01, self.publish_needlepose )
        
        # inform node is running
        self.get_logger().info("Running needle insertion robot translation unit.")

    # __init__

    def destroy_node( self ) -> bool:
        self.get_logger().info( "Shutting down needle insertion robot translation unit." )
        
        return super().destroy_node()

    # destroy_node

    def get_robotCoordinates(self):
        """ Get the current robot coordinates """
        
        return CoordinateConversions.RobotAxisPositionsToRobot(self.robot_axisPositions[0],
                                                               self.robot_axisPositions[1],
                                                               self.robot_axisPositions[2],
                                                               self.robot_axisPositions[3]
                                                            )

    # get_robotCoordinates

    def get_stageCoordinates(self):
        """ Get the current stage coordinates """
        x_r, y_r, z_r  = self.get_robotCoordinates()

        return CoordinateConversions.RobotToStage(x_r, y_r, z_r)

    # get_stageCoordinates

    def publish_needlepose( self ):
        """ Publish the current needle pose """ 
        # configure the needle pose message
        msg = PoseStamped()

        # configure the header
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "robot"

        # configure the needle pose message in stage coordinates
        current_x_s, current_y_s, current_z_s = self.get_stageCoordinates()
        msg.pose.position.x = current_x_s
        msg.pose.position.y = current_y_s
        msg.pose.position.z = current_z_s

        msg.pose.orientation.w = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0 
        msg.pose.orientation.z = 0.0 # axis to rotate needle axis

        # publish the message
        self.pub_needlepose.publish( msg )

    # publish_needlepose

    def sub_axisMoving_callback( self, msg: Bool, axis: int ):
        """ Subscriber for the axis call back """
        self.robot_axisMoving[axis] = msg.data

        # update axis on action server
        self.action_server.robot_axisMoving[axis] = self.robot_axisMoving[axis]

    # sub_axisMoving_callback

    def sub_axisPosition_callback( self, msg: Float32, axis: int ):
        """ Subscriber callback to update the current axis positions """
        self.robot_axisPositions[axis] = msg.data

        self.action_server.robot_axisPositions[axis] = self.robot_axisPositions[axis]

    # sub_axisPosition_callback

# class: NeedleInsertionRobotTranslationNode
