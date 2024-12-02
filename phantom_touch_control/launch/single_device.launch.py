import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="phantom_touch_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # prefix=['gdbserver localhost:3000'],
        composable_node_descriptions=[
            ComposableNode(
                package="phantom_touch_control",
                plugin="phantom_touch_control::TouchControlNode",
                name="phantom_touch_control",
                namespace="",
                parameters=[
                    {
                        "device_names": [
                            "TouchB"
                        ],  # Device name (if left blank the "Default Device" is used) #24019000802, 24019000809
                        "prefixes": [
                            "touch_b"
                        ],  # Joint names prefix (one prefix for each device)
                        # 'scheduler_rate': 1000,  # Can be 500 or 1000 Hz (default: 1000)
                        'publish_joint_states': True,  # Publishes joint states if True (default: False)
                    }
                ],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
