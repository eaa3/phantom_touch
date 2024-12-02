"""
Single-container phantom touch control nodes. Both containers run independly in their own processes.

"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="touch_control_container",
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
                            "TouchA",
                            "TouchB",
                        ],  # Device name (if left blank the "Default Device" is used)
                        "prefixes": [
                            "touch_a_",
                            "touch_b_",
                        ],  # Joint names prefix (one prefix for each device)
                        'scheduler_rate': 500,  # Can be 500 or 1000 Hz (default: 1000)
                        'publish_joint_states': True,  # Publishes joint states if True (default: False)
                    }
                ],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output="log",
        # prefix=["gdb -ex run -ex 'break phantom_touch_control_node.cpp:305' --args"]
    )

    return launch.LaunchDescription([container])
