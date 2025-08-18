import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the package directories
    obstacle_detection_share = get_package_share_directory('obstacle_detection')

    # Declare Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='fleet/skid_steered_two_lidars_0/',
        description='Top-level namespace')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(obstacle_detection_share, 'config', 'odpp.yaml'),
        description='Full path to the ROS2 parameters file')
    
    # Setting Launch Configuration
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')


    # Description of Nodes
    odpp_node = Node(
            package='obstacle_detection',
            executable='dynamic_obstacle_node',
            name='dynamic_obstacle_node',
            output='screen',
            namespace = namespace,
            parameters = [params_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')],
    )

    predictor_node = Node(
        package='obstacle_predictor',
        executable='predictor_node',
        name='predictor_node',
        output='screen',
        namespace=namespace,
        parameters=[params_file]
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(odpp_node)
    ld.add_action(predictor_node)
    return ld

