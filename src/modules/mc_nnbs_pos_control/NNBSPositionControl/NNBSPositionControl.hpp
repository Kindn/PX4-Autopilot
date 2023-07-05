/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct NNBSPositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
	matrix::Quatf orientation;
	matrix::Vector3f angular_velocity;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */

constexpr uint8_t kHiddenLayerSize = 12;

class NNBSPositionControl
{
public:
	NNBSPositionControl()
	{
		_alpha.setZero();
		_w.setZero();
	}
	~NNBSPositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P) { _gain_vel = P; }

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const NNBSPositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	void setCutoffFrequency(const matrix::Vector3f &wc) { _wc = wc; }

	void setUseCommandFilter(bool use_command_filter) { _use_command_filter = use_command_filter; }

	void setRBFWidth(float b) { _b = b; }

	void setAlphaDotLimits(const matrix::Vector3f &lim_alpha_dot_min,
			       const matrix::Vector3f &lim_alpha_dot_max);

	void setNNOutputLimits(const matrix::Vector3f &lim_nn_output_min,
			       const matrix::Vector3f &lim_nn_output_max);

	void setNNErrorLimits(const matrix::Vector3f &lim_nn_error_min,
			      const matrix::Vector3f &lim_nn_error_max);

	void setLearningRatesW(const matrix::Vector3f &lrw) { _lrw = lrw; }

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

private:
	bool _updateSuccessful();

	inline void getNNInputX(matrix::Vector<float, 12U> &input_x);

	// Gains
	matrix::Vector3f _gain_pos; ///< k1, k3, k5
	matrix::Vector3f _gain_vel; ///< k2, k4, k6

	// Errors
	matrix::Vector3f _error_pos; ///< z1, z3, z5
	matrix::Vector3f _error_vel; ///< z2, z4, z6

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have
	matrix::Vector3f _lim_alpha_dot_min;
	matrix::Vector3f _lim_alpha_dot_max;
	matrix::Vector3f _lim_nn_output_min;
	matrix::Vector3f _lim_nn_output_max;
	matrix::Vector3f _lim_nn_error_min;
	matrix::Vector3f _lim_nn_error_max;

	float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position: x1, x3, x5 */
	matrix::Vector3f _vel; /**< current velocity: x2, x4, x6 */
	matrix::Vector3f _vel_dot; /**< velocity derivative: x2_dot, x4_dot, x6_dot (replacement for acceleration estimate) */
	float _yaw{}; /**< current heading */
	matrix::Quatf _att; /**< current attitude */
	matrix::Vector3f _ang_vel; /**< current angular velocity */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position: xd1, xd3, xd5 */
	matrix::Vector3f _vel_sp; /**< desired velocity: xd1_dot, xd3_dot, xd5_dot */
	matrix::Vector3f _acc_sp; /**< desired acceleration: xd1_ddot, xd3_ddot, xd5_ddot */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	// Virtual controllers
	matrix::Vector3f _alpha;

	// Weights of RBFNN
	matrix::Matrix<float, kHiddenLayerSize, 3UL> _w;
	float _b{0.5f};

	// Learning rates of RBFNN
	matrix::Vector3f _lrw{6e-5, 6e-5, 6e-5};

	// Cutoff frequency of the first-order command filter used
	// for alpha_dot estimation.
	matrix::Vector3f _wc{1.0f, 1.0f, 1.0f};
	bool _use_command_filter{false};
};
