/*
 * Copyright (c) 2021, Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 // modified and maintained by Ermano Arruda 2024 (ermano.arruda@gmail.com)

#include "hd++.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "phantom_touch_msgs/msg/button_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

#include <range/v3/view/zip.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/constants/constants.hpp>

namespace phantom_touch_control {

using boost::math::double_constants::pi;

geometry_msgs::msg::Pose pose_msg(const hd::Transform& tf)
{
    geometry_msgs::msg::Pose m(rosidl_runtime_cpp::MessageInitialization::SKIP);
    m.position.x = tf.translation().x();
    m.position.y = tf.translation().y();
    m.position.z = tf.translation().z();
    Eigen::Quaterniond q(tf.rotation());
    m.orientation.w = q.w();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    return m;
}

geometry_msgs::msg::Twist twist_msg(const hd::Vector6d& twist)
{
    geometry_msgs::msg::Twist m(rosidl_runtime_cpp::MessageInitialization::SKIP);
    m.linear.x = twist[0];
    m.linear.y = twist[1];
    m.linear.z = twist[2];
    m.angular.x = twist[3];
    m.angular.y = twist[4];
    m.angular.z = twist[5];
    return m;
}

/*
    3rd order Butterworth lowpass filter with cut-off frequency at 12 Hz

    Design:

    ```python
    import numpy as np
    from scipy import signal
    np.set_printoptions(formatter={'float': '{: 0.15f}'.format})

    n = 3              # filter order
    fs = 1000          # sample frequency
    fc = 12            # cut-off frequency of the filter
    w = fc / (fs / 2)  # normalized frequency
    b, a = signal.butter(n, w, btype='lowpass')
    print('b: {}'.format(b))
    print('a: {}'.format(a))
    ```
*/
template<typename T>
class ButterworthLowpassFilter3
{
public:
    static constexpr int N = 3;

    explicit ButterworthLowpassFilter3(int fs, T init_val = T{})
        : b_{}
        , a_{}
        , y_(N)
        , x_(N)
    {
        if (fs == 500) {
            b_ = {0.000370683374298, 0.001112050122894, 0.001112050122894, 0.000370683374298};
            a_ = {1.000000000000000, -2.698687669534221, 2.441079708544681, -0.739426572016076};
        } else if (fs == 1000) {
            b_ = {0.000049757435769, 0.000149272307306, 0.000149272307306, 0.000049757435769};
            a_ = {1.000000000000000, -2.849239095203546, 2.709629132810162, -0.859991978120466};
        } else {
            throw std::invalid_argument("ButterworthLowpassFilter3 only knows filter coefficients for sampling frequencies 500 and 1000");
        }

        reset(init_val);
    }

    T update(const T& in)
    {
        // Calculate transfer function
        T filtered = b_[0] * in
            + b_[1] * x_[0]
            + b_[2] * x_[1]
            + b_[3] * x_[2]
            - a_[1] * y_[0]
            - a_[2] * y_[1]
            - a_[3] * y_[2];

        // Update history (most recent samples are in the front)
        x_.push_front(in);
        y_.push_front(filtered);

        return filtered;
    }

    void reset(const T& val = T{})
    {
        y_.assign(y_.capacity(), val);
        x_.assign(x_.capacity(), val);
    }

private:
    std::array<double, N + 1> b_;
    std::array<double, N + 1> a_;
    boost::circular_buffer<T> y_;
    boost::circular_buffer<T> x_;
};

class DeviceController
{
public:
    DeviceController(rclcpp::Node* node,
                     const std::string& device_name,
                     const std::string& prefix,
                     int scheduler_rate_hz,
                     bool publish_joint_states)
        : node_(node)
        , device_(device_name)
        , filter_(scheduler_rate_hz)
        , dt_(1.0 / scheduler_rate_hz)
    {
        // Prefix frame/joint names
        base_frame_name_ = prefix + "base";
        joint_names_ = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};

        for (auto& n : joint_names_)
            n = prefix + n;

        std::string ns = device_name.empty() ? "" : device_name + "/";
        pub_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(ns + "pose_stylus_current", 1);
        pub_twist_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(ns + "twist_stylus_current", 1);
        pub_button_ = node->create_publisher<phantom_touch_msgs::msg::ButtonEvent>(ns + "button_event", 10);

        if (publish_joint_states)
            pub_joints_ = node->create_publisher<sensor_msgs::msg::JointState>(ns + "joint_states", 1);
    }

    void publish_state()
    {
        hd::ScopedFrame frame(device_);
        auto t_now = node_->now();

        // Joint angles
        hd::Vector6d joint_position = device_.joint_angles_current();

        // Joint velocities: compute from joint angles because the haptic
        // device API is buggy and just returns zeros
        hd::Vector6d joint_velocity = hd::Vector6d::Zero();

        if (device_.inkwell_switch_current()) {
            // Reset filter and force zero velocity while stylus is placed in
            // the inkwell (this triggers device calibration and likely
            // discontinous jumps in position)
            filter_.reset(joint_velocity);
        } else {
            // Backward difference velocity approximation, low pass filtered
            hd::Vector6d dq = joint_position - device_.joint_angles_last();
            joint_velocity = filter_.update(dq / dt_);
        }

        // End-effector (stylus) transform seen wrt. device base frame
        hd::Transform transform = device_.transform_current();

        // Compute Cartesian space velocities using the end-effector Jacobian
        hd::Vector6d twist = device_.jacobian_current() * joint_velocity;

        // Button states
        auto buttons_current = device_.buttons_current();
        auto buttons_last = device_.buttons_last();

        // Publish if button states changed
        if (!buttons_last[0] && buttons_current[0]) {
            auto m_btn = std::make_unique<phantom_touch_msgs::msg::ButtonEvent>(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_btn->button = phantom_touch_msgs::msg::ButtonEvent::BUTTON_GRAY;
            m_btn->event = phantom_touch_msgs::msg::ButtonEvent::EVENT_PRESSED;
            pub_button_->publish(std::move(m_btn));
        } else if (buttons_last[0] && !buttons_current[0]) {
            auto m_btn = std::make_unique<phantom_touch_msgs::msg::ButtonEvent>(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_btn->button = phantom_touch_msgs::msg::ButtonEvent::BUTTON_GRAY;
            m_btn->event = phantom_touch_msgs::msg::ButtonEvent::EVENT_RELEASED;
            pub_button_->publish(std::move(m_btn));
        }

        if (!buttons_last[1] && buttons_current[1]) {
            auto m_btn = std::make_unique<phantom_touch_msgs::msg::ButtonEvent>(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_btn->button = phantom_touch_msgs::msg::ButtonEvent::BUTTON_WHITE;
            m_btn->event = phantom_touch_msgs::msg::ButtonEvent::EVENT_PRESSED;
            pub_button_->publish(std::move(m_btn));
        } else if (buttons_last[1] && !buttons_current[1]) {
            auto m_btn = std::make_unique<phantom_touch_msgs::msg::ButtonEvent>(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_btn->button = phantom_touch_msgs::msg::ButtonEvent::BUTTON_WHITE;
            m_btn->event = phantom_touch_msgs::msg::ButtonEvent::EVENT_RELEASED;
            pub_button_->publish(std::move(m_btn));
        }

        // Rotate such that X points forward, Y left and Z up, according to
        // ROS convention
        transform = transform
            * Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitX());

        // and scale translation to meters
        transform.translation() /= 1000;

        auto m_pose = std::make_unique<geometry_msgs::msg::PoseStamped>(rosidl_runtime_cpp::MessageInitialization::SKIP);
        m_pose->header.frame_id = base_frame_name_;
        m_pose->header.stamp = t_now;
        m_pose->pose = pose_msg(transform);
        pub_pose_->publish(std::move(m_pose));

        // Scale linear part of twist to m/s
        twist.head<3>() /= 1000;

        auto m_twist = std::make_unique<geometry_msgs::msg::TwistStamped>(rosidl_runtime_cpp::MessageInitialization::SKIP);
        m_twist->header.frame_id = base_frame_name_;
        m_twist->header.stamp = t_now;
        m_twist->twist = twist_msg(twist);
        pub_twist_->publish(std::move(m_twist));

        if (pub_joints_) {
            auto m_jnt = std::make_unique<sensor_msgs::msg::JointState>(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_jnt->header.stamp = t_now;
            m_jnt->name = joint_names_;
            std::copy(joint_position.data(), std::next(joint_position.data(), joint_position.size()), std::back_inserter(m_jnt->position));
            std::copy(joint_velocity.data(), std::next(joint_velocity.data(), joint_velocity.size()), std::back_inserter(m_jnt->velocity));
            pub_joints_->publish(std::move(m_jnt));
        }
        // RCLCPP_INFO_STREAM(node_->get_logger(), "publish state");
    }

private:
    rclcpp::Node* node_;
    hd::Device device_;
    ButterworthLowpassFilter3<hd::Vector6d> filter_;
    double dt_;
    std::string base_frame_name_;
    std::vector<std::string> joint_names_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::Publisher<phantom_touch_msgs::msg::ButtonEvent>::SharedPtr pub_button_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
};

class TouchControlNode : public rclcpp::Node
{
public:
    TouchControlNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("phantom_touch_control", options)
    {
        auto device_names = declare_parameter<std::vector<std::string>>("device_names", {""});
        auto prefixes = declare_parameter<std::vector<std::string>>("prefixes", {""});
        auto scheduler_rate_hz = declare_parameter<int>("scheduler_rate", 1000);
        auto publish_joint_states = declare_parameter<bool>("publish_joint_states", false);

        for (auto [name, prefix] : ranges::views::zip(device_names, prefixes)){

            device_controllers_.emplace_back(this, name, prefix, scheduler_rate_hz, publish_joint_states);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        }

        // Continuously publish device states from the "haptics loop" thread
        for (auto& dc : device_controllers_) {
            scheduler_.schedule_asynchronous([&dc]() {
                dc.publish_state();
                return rclcpp::ok();
            });
        }

        scheduler_.start(scheduler_rate_hz);
    }

private:
    std::list<DeviceController> device_controllers_;
    hd::Scheduler scheduler_;
};

} // namespace phantom_touch_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(phantom_touch_control::TouchControlNode)
