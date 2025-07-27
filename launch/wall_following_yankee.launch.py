# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='wall_following_yankee' #<--- CHANGE ME

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    # twist_mux_node = Node(package='twist_mux', 
    #                 executable='twist_mux',
    #                 parameters=[twist_mux_params,{'use_sim_time': True}],
    #                 remappings=[('/cmd_vel_out','/cmd_vel')]
    # )
    dist_finder_yankee = Node(package=package_name,
                             executable='dist_finder_yankee',
                             name='dist_finder_yankee_node',
    )
    control_yankee = Node(package=package_name,
                          executable='control_yankee',
                          name='control_yankee_node',
    )

# Launch them all!
    return LaunchDescription([
        dist_finder_yankee,
        control_yankee
    ])