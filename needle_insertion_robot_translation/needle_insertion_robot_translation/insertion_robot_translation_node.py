# standard libraries
from functools import partial
from math import sqrt
import asyncio

# ros2 libraries
import rclpy

from rclpy.executors import MultiThreadedExecutor

# custom imports
from .nodes import NeedleInsertionRobotActionServer, NeedleInsertionRobotTranslationNode


def main(args=None):
    rclpy.init(args=args)

    # setup executor for multi-threaded functionality
    executor = MultiThreadedExecutor( num_threads=4 )

    # setup nodes
    action_server_node = NeedleInsertionRobotActionServer()
    translation_node = NeedleInsertionRobotTranslationNode(action_server=action_server_node)

    # add nodes to executor
    executor.add_node( action_server_node )
    executor.add_node( translation_node )

    # spin the executor
    try:
        executor.spin()
        
    # try
    finally:
        executor.shutdown()
        action_server_node.destroy_node()
        translation_node.destroy_node()

    # finally


    rclpy.shutdown()


# main

if __name__ == '__main__':
    main()

# if __main__