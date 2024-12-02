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

#pragma once

#include <HD/hd.h>

#include <Eigen/Geometry>

#include <functional>
#include <list>
#include <thread>
#include <chrono>

namespace hd {

using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Transform = Eigen::Isometry3d;

class Device
{
public:
    using handle_t = HHD;

    explicit Device(const std::string& name = {});
    ~Device();

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    Device(Device&& other);
    Device& operator=(Device&& other);

    handle_t handle() const;
    std::string model_type() const;
    std::string driver_version() const;
    std::string vendor() const;
    std::string serial_number() const;

    void make_current() const;
    void set_force_enabled(bool enable);

    /*!
     * \brief Get the button states of the device.
     * \return An array of booleans which are true if the button is pressed down
     *         and false otherwise.
     * \note Should only be called from within a "haptics frame"!
     */
    std::array<bool, 4> buttons_current() const;
    std::array<bool, 4> buttons_last() const;

    /*!
     * \brief Get the inkwell switch active status.
     * \return True if the switch is pressed (the stylus is placed in the
     *         inkwell), false otherwise
     * \note Should only be called from within a "haptics frame"!
     */
    bool inkwell_switch_current() const;
    bool inkwell_switch_last() const;

    /*!
     * \brief Get the position of the device end-effector wrt. the base frame.
     *
     * The translation is in millimeters.
     *
     * The base frame has origin "in" the inkwell, +X is right, +Y is up
     * and +Z is toward the user.
     *
     * \return The 3x1 position vector of the end-effector.
     * \note Should only be called from within a "haptics frame"!
     */
    Vector3d position_current() const;
    Vector3d position_last() const;

    /*!
     * \brief Get the transform of the device end-effector wrt. the base frame.
     *
     * The translation is in millimeters.
     *
     * The base frame has origin "in" the inkwell, +X is right, +Y is up
     * and +Z is toward the user.
     *
     * \return The 4x4 homogenous transformation matrix of the end-effector.
     * \note Should only be called from within a "haptics frame"!
     */
    Transform transform_current() const;
    Transform transform_last() const;

    /*!
     * \brief Get the twist of the device end-effector wrt. the base frame.
     * \return The 6x1 twist (Cartesian space linear and angular velocities).
     * \note Should only be called from within a "haptics frame"!
     */
    Vector6d twist_current() const;
    Vector6d twist_last() const;

    /*!
     * \brief Get the wrench of the device end-effector.
     *
     * Get the current force and torque that the user is commanding to the
     * device during the frame in which this is called. Returns zeros if no
     * force/torque has been commanded yet in the frame.
     *
     * \return The 6x1 commanded wrench (Cartesian space force and torque).
     * \note Should only be called from within a "haptics frame"!
     */
    Vector6d wrench_current() const;
    Vector6d wrench_last() const;

    /*!
     * \brief Get the Jacobian of the device end-effector wrt. the base frame.
     * \return The 6x6 end-effector Jacobian matrix.
     * \note Should only be called from within a "haptics frame"!
     */
    Matrix6d jacobian_current() const;
    Matrix6d jacobian_last() const;

    /*!
     * \brief Get the joint angles of the device.
     * \return The 6x1 vector of joint angles.
     * \note Should only be called from within a "haptics frame"!
     */
    Vector6d joint_angles_current() const;
    Vector6d joint_angles_last() const;

    /*!
     * \brief Get the joint torques of the device.
     *
     * Gets the current joint torques that the user is commanding to the device
     * during the frame in which this is called. Returns zeros if no torques
     * have been commanded yet in the frame.

     * \return The 6x1 vector of commanded joint torques.
     * \note Should only be called from within a "haptics frame"!
     */
    Vector6d joint_torque_current() const;
    Vector6d joint_torque_last() const;

    /*!
     * \brief Check whether the current calibration is OK.
     * \return True if the calibration is deemed OK, false otherwise.
     * \note Should only be called from within a "haptics frame"!
     */
    bool calibration_ok() const;

private:
    handle_t handle_;
    std::string name_;
};

class Scheduler
{
public:
    using handle_t = HDSchedulerHandle;

    enum class Priority : HDushort
    {
        PRIORITY_MIN = HD_MIN_SCHEDULER_PRIORITY,
        PRIORITY_DEFAULT = HD_DEFAULT_SCHEDULER_PRIORITY,
        PRIORITY_MAX = HD_MAX_SCHEDULER_PRIORITY,
    };

private:
    using callback_t = std::function<bool()>;

    struct SchedOp
    {
        handle_t handle;
        callback_t callback;
        Scheduler* scheduler; // the scheduler which scheduled this op
    };

public:
    Scheduler();
    ~Scheduler();

    void start(unsigned desired_rate_hz);
    void stop();
    bool is_running() const;

    void unschedule(handle_t handle);
    void unschedule_all();

    // Schedule a function to run in the scheduler thread and wait for completion
    template<typename F>
    void schedule_synchronous(F callback, Priority priority = Priority::PRIORITY_DEFAULT)
    {
        schedule_synchronous(
            [](void* f) -> HDCallbackCode {
                (*static_cast<decltype(callback)*>(f))();
                return HD_CALLBACK_DONE;
            },
            &callback,
            static_cast<HDushort>(priority));
    }

    template<typename F>
    handle_t schedule_asynchronous(F&& callback, Priority priority = Priority::PRIORITY_DEFAULT)
    {
        // Pass pointer to SchedOp as "user data" and use it from an intermediate
        // captureless lambda (which can be passed as a function pointer)
        operations_.push_back({HD_INVALID_HANDLE, std::forward<F>(callback), this});
        operations_.back().handle = schedule_asynchronous(
            [](void* op_vp) -> HDCallbackCode {

                auto op = static_cast<SchedOp*>(op_vp);

                if (op->callback()) {

                    return HD_CALLBACK_CONTINUE;
                } else {
                    printf("Callback returned false. Removing from scheduler list...\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    // remove op from scheduler list once it has ended (the callback returned false)
                    op->scheduler->operations_.remove_if([&op](const auto& other) { return op->handle = other.handle; });
                    return HD_CALLBACK_DONE;
                }
                

                return HD_CALLBACK_CONTINUE; //delme
            },
            &operations_.back(),
            static_cast<HDushort>(priority));
        return operations_.back().handle;
    }

private:
    void schedule_synchronous(HDSchedulerCallback cb, void* data, HDushort prio);
    handle_t schedule_asynchronous(HDSchedulerCallback cb, void* data, HDushort prio);

private:
    bool is_running_;
    std::list<SchedOp> operations_;
};

class ScopedFrame
{
public:
    using handle_t = HHD;

    explicit ScopedFrame(const Device& device);
    ~ScopedFrame();

    ScopedFrame(const ScopedFrame&) = delete;
    ScopedFrame& operator=(const ScopedFrame&) = delete;

    ScopedFrame(ScopedFrame&& other);
    ScopedFrame& operator=(ScopedFrame&& rhs);

private:
    handle_t handle_;
};

} // namespace hd
