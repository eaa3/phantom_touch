"""
Multi-container control nodes. Both containers run independly in their own processes.

"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    device_dict = {"TouchA": "touch_a_", "TouchB": "touch_b_"}
    container_list = []
    for device in device_dict:

        container = ComposableNodeContainer(
            name=f"phantom_touch_control_container_{device}",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            # prefix=['gdbserver localhost:3000'],
            composable_node_descriptions=[
                ComposableNode(
                    package="phantom_touch_control",
                    plugin="phantom_touch_control::TouchControlNode",
                    name=f"phantom_touch_control_{device}",
                    namespace="",
                    parameters=[
                        {
                            "device_names": [
                                f"{device}"
                            ],  # Device name (if left blank the "Default Device" is used) #24019000802, 24019000809
                            "prefixes": [
                                device_dict[device]
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

        container_list.append(container)

    return launch.LaunchDescription(container_list)
