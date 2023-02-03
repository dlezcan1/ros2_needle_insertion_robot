# standard libraries
from functools import partial
from math import sqrt
import asyncio
import time

# ros2 libraries
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
        x_s = -y_r # * 1e-3 # convert mm -> m
        y_s =  x_r # * 1e-3 # convert mm -> m
        z_s =  z_r # * 1e-3 # convert mm -> m

        return x_s, y_s, z_s

    # RobotToStage

    @staticmethod
    def StageToRobot(x_s, y_s, z_s):
        """ Convert Stage to Robot Coordinates """
        x_r =  y_s # * 1e3 # convert m -> mm
        y_r = -x_s # * 1e3 # convert m -> mm
        z_r = -z_s # * 1e3 # convert m -> mm

        return x_r, y_r, z_r

    # StageToRobot


# class: CoordinateConversions

class NeedleInsertionRobotActionServer(Node):
    def __init__(self, name = "NeedleInsertionRobotActionServerNode" ):
        super().__init__( name )

        # parameters
        self.actionsrv_translation_running = False
        # - stored robot states
        self.robot_axisPositions = [   0.0,   0.0,   0.0,   0.0 ] # current robot axis positions
        self.robot_axisMoving    = [ False, False, False, False ] # whether the robot is moving or not

        # publishers
        # - robot command
        self.pub_robot_axisCommand_x  = self.create_publisher( Float32, "axis/command/x",            1 )
        self.pub_robot_axisCommand_y  = self.create_publisher( Float32, "axis/command/y",            1 )
        self.pub_robot_axisCommand_z  = self.create_publisher( Float32, "axis/command/z",            1 )
        self.pub_robot_axisCommand_ls = self.create_publisher( Float32, "axis/command/linear_stage", 1 )

        # clients
        self.cli_robot_abort         = self.create_client( Trigger, "abort" )

        # action servers
        self.actionsrv_translation = ActionServer(self, MoveStage, "move_stage",
                                                  execute_callback = self.action_translation_execute_callback,
                                                  callback_group   = MutuallyExclusiveCallbackGroup(),
                                                  goal_callback    = self.action_translation_goal_callback,
                                                  cancel_callback  = self.action_translation_cancel_callback
                                                )

        # inform node is running
        self.get_logger().info("Running needle insertion robot action server.")
        
    # __init__

    def action_translation_cancel_callback( self, goal_handle: ServerGoalHandle ):
        """ Cancel the current action """
        # abort robot motion
        self.cli_robot_abort.call()

        goal_handle.abort()
        
        return CancelResponse.ACCEPT

    # action_translation_cancel_callback

    async def action_translation_execute_callback( self, goal_handle: ServerGoalHandle ) :
        """ Handle the goal """
        self.get_logger().info(f"Executing goal to axis: x={goal_handle.request.x} m, z={goal_handle.request.z} m...")

        # create lambda error function
        compute_error = lambda current_x, current_z: sqrt( (current_x - goal_handle.request.x)**2 + (current_z - goal_handle.request.z)**2 )
        
        self.actionsrv_translation_running = True  # action server is running

        feedback_msg = MoveStage.Feedback()
        
        # command the robot to goal position ( convert m -> mm )
        _, current_y_r, current_z_r = self.get_robotCoordinates()
        _, desired_y_r, desired_z_r = CoordinateConversions.StageToRobot(goal_handle.request.x, 0, goal_handle.request.z)
        self.pub_robot_axisCommand_y.publish( Float32( data=desired_y_r - current_y_r ) ) # command y-axis
        self.pub_robot_axisCommand_z.publish( Float32( data=desired_z_r - current_z_r ) ) # command z-axis 
        
        time.sleep(0.5)

         # provide feedback
        while self.robot_axisMoving[1] or self.robot_axisMoving[2]: # check if axis Y or Z are moving
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                # setup the result
                result_msg = MoveStage.Result()
                current_x_s, _, current_z_s = self.get_stageCoordinates()
                result_msg.x = float( current_x_s ) # robot y-axis
                result_msg.z = float( current_z_s ) # robot z-axis
                result_msg.error = compute_error( result_msg.x, result_msg.z )

                return result_msg

            # if 

            current_x_s, _, current_z_s = self.get_stageCoordinates()
            feedback_msg.x = float( current_x_s ) # robot y-axis
            feedback_msg.z = float( current_z_s ) # robot z-axis
            time_now = self.get_clock().now().seconds_nanoseconds()
            feedback_msg.time = float( time_now[0]*1e9 + time_now[1] )
            feedback_msg.error = compute_error(feedback_msg.x, feedback_msg.z)
        
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Publishing feedback: x: {feedback_msg.x} m, z: {feedback_msg.z} m, error {feedback_msg.error} m ")

        # while

        # setup the result
        result_msg = MoveStage.Result()
        current_x_s, _, current_z_s = self.get_stageCoordinates()
        result_msg.x = float( current_x_s ) # robot y-axis
        result_msg.z = float( current_z_s ) # robot z-axis
        result_msg.error = compute_error( result_msg.x, result_msg.z )

        self.actionsrv_translation_running = False # action server finished
        self.get_logger().info(f"Finished execution with following: x={result_msg.x} m, z={result_msg.z} m, error={result_msg.error} m")
        
        if result_msg.error <= goal_handle.request.eps:
            goal_handle.succeed()

        else: # only attempt one command
            goal_handle.abort()
            

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
        msg.pose.orientation.y = 0.0 # axis to rotate needle axis
        msg.pose.orientation.z = 0.0

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