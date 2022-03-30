from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    interrogator_ip_arg = DeclareLaunchArgument( 'ip',
                                                 default_value='192.168.1.11'
                                                 )
    num_samples_arg = DeclareLaunchArgument( 'numSamples',
                                             default_value='200'
                                             )

    # Nodes
    sm130_node = Node(
            package='sm130_interrogator_py',
            namespace='needle',
            executable='sm130_interrogator',
            output='screen',
            emulate_tty=True,
            parameters=[ {
                    "interrogator.ip_address": LaunchConfiguration( 'ip' ),
                    "sensor.num_samples"     : LaunchDescription( 'numSamples' ),
                    } ]
            )

    # add to launch description
    ld.add_action( interrogator_ip_arg )
    ld.add_action( num_samples_arg )
    ld.add_action( sm130_node )

    return ld

# generate_launch_description
