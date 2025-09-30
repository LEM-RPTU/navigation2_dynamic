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
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='fleet/skid_steered_two_lidars_0/',
        description='Top-level robot namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(obstacle_detection_share, 'config', 'odpp.yaml'),
        description='Full path to the ROS2 parameters file')
    
    # Setting Launch Configuration
    robot_namespace = LaunchConfiguration('robot_namespace')
    params_file = LaunchConfiguration('params_file')


    # Description of Nodes
    odpp_node = Node(
            package='obstacle_detection',
            executable='dynamic_obstacle_node',
            name='dynamic_obstacle_node',
            output='screen',
            namespace = robot_namespace,
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
        namespace=robot_namespace,
        parameters=[params_file]
    )

    obstacle_viz_node = Node(
        package='obstacle_detection',
        executable='obstacle_viz_node',
        name='obstacle_viz_node',
        output='screen',
        namespace=robot_namespace,
        parameters=[params_file]
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(odpp_node)
    ld.add_action(predictor_node)
    ld.add_action(obstacle_viz_node)
    return ld

