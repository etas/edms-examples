import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='interprocess_chain',
            executable='publisher',
            name='publisher_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='interprocess_chain',
            executable='callback1_node',
            name='callback1_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='interprocess_chain',
            executable='callback2_node',
            name='callback2_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='interprocess_chain',
            executable='callback3_node',
            name='callback3_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='interprocess_chain',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        )
    ])
