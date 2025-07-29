# library to move between files and folders in the O.S.
import os
from ament_index_python.packages import get_package_share_directory
# libraries to define the Launch file and Function
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='wall_following_yankee' #<--- CHANGE ME

    kp_arg = DeclareLaunchArgument('kp', default_value='1.5', description='Proportional gain')
    kd_arg = DeclareLaunchArgument('kd', default_value='3.0', description='Derivative gain')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='1.0', description='Maximum speed')
    min_speed_arg = DeclareLaunchArgument('min_speed', default_value='0.5', description='Minimum speed')

    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    max_speed = LaunchConfiguration('max_speed')
    min_speed = LaunchConfiguration('min_speed')

    dist_finder_yankee = Node(package=package_name,
                             executable='dist_finder_yankee',
                             name='dist_finder_yankee_node',
    )
    control_yankee = Node(package=package_name,
                          executable='control_yankee',
                          name='control_yankee_node',
                          parameters=[{
                                 'kp': kp,
                                 'kd': kd,
                                 'max_speed': max_speed,
                                 'min_speed': min_speed
                             }],
    )

# Launch them all!
    return LaunchDescription([
        dist_finder_yankee,
        control_yankee
    ])