import os

from ament_index_python import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    # Config files
    audio_config = os.path.join(
        get_package_share_directory('ros_audition'),
        'config',
        'kinect_config.yaml'
    )

    # Audio acquisition node
    acq_node = Node(
        package='ros_audition',
        executable='audio_acq_node.py',
        name='audio_acq_node',
        output='screen',
        parameters=[audio_config]
    )
    ld.add_action(acq_node)

    # Direction of arrival (DOA)/PyRoomAcoustics node
    pra_node = Node(
        package='ros_audition',
        executable='pra_node.py',
        name='pra_node',
        output='screen',
        parameters=[audio_config]
    )
    ld.add_action(pra_node)

    # Directional speech recognition node
    speech_node = Node(
        package='ros_audition',
        executable='doa_speech_rec_node.py',
        name='directional_speech_rec_node',
        output='screen',
        parameters=[audio_config]
    )
    ld.add_action(speech_node)

    return ld