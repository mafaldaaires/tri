import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('firstrobot_webots')
    
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    follower_robot_description_path = os.path.join(package_dir, 'resource', 'follower_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
            {'cmd_vel_topic': '/cmd_vel'}
        ]
    )

    follower_robot_driver = WebotsController(
        robot_name='follower_robot',
        parameters=[
            {'robot_description': follower_robot_description_path},
            {'cmd_vel_topic': '/follower_cmd_vel'}
        ]
    )

    obstacle_avoider = Node(
        package='firstrobot_webots',
        executable='obstacle_avoider',
        output='screen'
    )

    follower_robot = Node(
        package='firstrobot_webots',
        executable='follower_robot',
        output='screen'
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        follower_robot_driver,
        obstacle_avoider,
        follower_robot,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])