from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='campus_tour',
            executable='ci_agent_node',
            name='ci_agent',
            output='screen',
            parameters=[{'config_path': 'src/campus_tour/campus_tour/config/CI_agents.yaml'}]
        ),
        Node(
            package='campus_tour',
            executable='bi_agent_node',
            name='bi_agent',
            output='screen',
            parameters=[{'config_path': 'src/campus_tour/campus_tour/config/BI_agents.yaml'}]
        ),
        Node(
            package='campus_tour',
            executable='visitor_node',
            name='visitor_agent',
            output='screen',
            parameters=[{'config_path': 'src/campus_tour/campus_tour/config/Visitor_agents.yaml'}]
        ),
        Node(
            package='campus_tour',
            executable='performance_analyzer_node',
            name='performance_analyzer',
            output='screen'
        ),
        Node(
            package='campus_tour',
            executable='visualization_node',
            name='campus_visualizer',
            output='screen'
        )
    ])
