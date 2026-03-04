from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    turn2_angle = LaunchConfiguration('turn2_angle')
    side_travel = LaunchConfiguration('side_travel')

    return LaunchDescription([

        DeclareLaunchArgument(
            'turn2_angle',
            default_value='2.356',   
            description='Turn angle for second turn'
        ),

        DeclareLaunchArgument(
            'side_travel',
            default_value='1.414',
            description='Distance for triangle side'
        ),

        Node(
            package='p3_triangle',
            executable='tf_triangle_node',
            name='tf_triangle_mover',
            parameters=[
                {'turn2_angle': turn2_angle},
                {'side_travel': side_travel}
            ]
        )
    ])