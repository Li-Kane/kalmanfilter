from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

simnode = Node(package='square_robot_sim', executable='square_robot_node',
               name='square_robot_node')


def generate_launch_description():
    return LaunchDescription([
        simnode,
        Node(package='square_robot_sim', executable='error_publisher',
             name='error_publisher'),
        RegisterEventHandler(
            OnProcessExit(
                target_action=simnode,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='Pybullet window closed / other error'))
                ]
            )
        ),
    ])
