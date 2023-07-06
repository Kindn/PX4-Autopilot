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
 * @file NNBSPositionControl.cpp
 */

#include "NNBSPositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

using namespace matrix;

void NNBSPositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void NNBSPositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void NNBSPositionControl::updateHoverThrust(const float hover_thrust_new)
{
	setHoverThrust(hover_thrust_new);
}

void NNBSPositionControl::setState(const NNBSPositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
	_att = states.orientation;
	_ang_vel = states.angular_velocity;
}

void NNBSPositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	// PX4_INFO("setpoint.acceleration: [%f %f %f]", (double)setpoint.acceleration[0], (double)setpoint.acceleration[1], (double)setpoint.acceleration[2]);

	_vel_sp.xy() = ControlMath::constrainXY(_vel_sp.xy(), Vector2f(0.0f, 0.0f), _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void NNBSPositionControl::setAlphaDotLimits(const matrix::Vector3f &lim_alpha_dot_min,
			       		    const matrix::Vector3f &lim_alpha_dot_max)
{
	_lim_alpha_dot_min = lim_alpha_dot_min;
	_lim_alpha_dot_max = lim_alpha_dot_max;
}

void NNBSPositionControl::setNNOutputLimits(const matrix::Vector3f &lim_nn_output_min,
			       		    const matrix::Vector3f &lim_nn_output_max)
{
	_lim_nn_output_min = lim_nn_output_min;
	_lim_nn_output_max = lim_nn_output_max;
}

void NNBSPositionControl::setNNErrorLimits(const matrix::Vector3f &lim_nn_error_min,
			      		   const matrix::Vector3f &lim_nn_error_max)
{
	_lim_nn_error_min = lim_nn_error_min;
	_lim_nn_error_max = lim_nn_error_max;
}

bool NNBSPositionControl::update(const float dt)
{
	// x and y input setpoints always have to come in pairs
	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// Since _hover_thrust [0.1, 0.9] does not represent the actual thrust,
	// this expression is not very proper exactly.
	// Maybe it is just a matter of proportional coefficient.
	// double mass = _hover_thrust / CONSTANTS_ONE_G;
	// PX4_INFO("vel: [%f %f %f]", (double)_vel(0), (double)_vel(1), (double)_vel(2));
	// PX4_INFO("pos: [%f %f %f]", (double)_pos(0), (double)_pos(1), (double)_pos(2));
	// PX4_INFO("vel_sp: [%f %f %f]", (double)_vel_sp(0), (double)_vel_sp(1), (double)_vel_sp(2));
	// PX4_INFO("pos_sp: [%f %f %f]", (double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2));
	// PX4_INFO("acc_sp: [%f %f %f]", (double)_acc_sp(0), (double)_acc_sp(1), (double)_acc_sp(2));
	/* Calculate zi */
	ControlMath::setZeroIfNanVector3f(_pos_sp);
	ControlMath::setZeroIfNanVector3f(_vel_sp);
	ControlMath::setZeroIfNanVector3f(_acc_sp);
	matrix::Vector3f alpha_new;
	_error_pos = _pos - _pos_sp;
	alpha_new = -_gain_pos.emult(_error_pos) + _vel_sp;
	_error_vel = _vel - alpha_new;
	// PX4_INFO("alpha_new: [%f %f %f]", (double)alpha_new(0), (double)alpha_new(1), (double)alpha_new(2));

	/* Estimate alpha_dot */
	matrix::Vector3f alpha_dot;
	if (_use_command_filter) {
		alpha_dot = -_wc.emult(_alpha - alpha_new) * dt; // first-order command filter
	} else {
		alpha_dot = -_gain_pos.emult(_vel) + _acc_sp;
	}
	for (int i = 0; i < 3; ++i) {
		alpha_dot(i) = math::constrain(alpha_dot(i), _lim_alpha_dot_min(i), _lim_alpha_dot_max(i));
	}
	_alpha = alpha_new;

	/* Calculate desired thrust */
	matrix::Vector<float, 12U> input_x;
	getNNInputX(input_x);
	float input_x_sqr_norm = input_x.norm_squared();

	// Since the centers and the widths of all the RBF nodes are the same,
	// we only calculate s once.
	matrix::Vector<float, 12U> s;
	s.setOne();
	s *= std::exp(-input_x_sqr_norm / (2.0f * _b * _b));
	// PX4_INFO("w(0:3): [%f %f %f]", (double)_w(0, 0), (double)_w(0, 1), (double)_w(0, 2));
	// PX4_INFO("s(0:3): [%f %f %f]", (double)s(0), (double)s(1), (double)s(2));

	matrix::Vector3f nn_outputs = _w.transpose() * s;
	for (int i = 0; i < 3; ++i) {
		nn_outputs(i) = math::constrain(nn_outputs(i), _lim_nn_output_min(i), _lim_nn_output_max(i));
	}
	// PX4_INFO("nn_outputs: [%f %f %f]", (double)nn_outputs(0), (double)nn_outputs(1), (double)nn_outputs(2));
	matrix::Vector3f u =
		-_gain_vel.emult(_error_vel) + alpha_dot - nn_outputs - _error_pos;
	// _thr_sp = -_gain_vel.emult(_error_vel) + alpha_dot - _w.transpose() * s - _error_pos;
	// _thr_sp *= mass;
	u(2) += _hover_thrust;
	_thr_sp = u;

	// PX4_INFO("error_vel: [%f %f %f]", (double)_error_vel(0), (double)_error_vel(1), (double)_error_vel(2));
	// PX4_INFO("error_pos: [%f %f %f]", (double)_error_pos(0), (double)_error_pos(1), (double)_error_pos(2));
	// PX4_INFO("u: [%f %f %f]", (double)u(0), (double)u(1), (double)u(2));

	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-u(0), -u(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = u(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

	// Get allowed horizontal thrust after prioritizing vertical control
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	const matrix::Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}
	// PX4_INFO("thr_sp: [%f %f %f]", (double)_thr_sp(0), (double)_thr_sp(1), (double)_thr_sp(2));

	/* Update weights of RBFNN */
	// Calculate NN errors
	matrix::Vector3f nn_errors =
		(_vel_dot - alpha_dot) + _gain_vel.emult(_error_vel) + _error_pos;
	nn_errors(2) -= CONSTANTS_ONE_G;

	for (int i = 0; i < 3; ++i) {
		nn_errors(i) = math::constrain(nn_errors(i), _lim_nn_error_min(i), _lim_nn_error_max(i));
		_w.col(i) += s * (_lrw(i) * nn_errors(i));
	}

	// Make sure weights contain no NANs
	for (int i = 0; i < 12; ++i) {
		for (int j = 0; j < 3; ++j) {
			ControlMath::addIfNotNan(_w(i, j), 0.0f);
		}
	}

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
}

inline void NNBSPositionControl::getNNInputX(matrix::Vector<float, 12U> &input_x)
{
	input_x.slice<3, 1>(0, 0) = _pos;
	input_x.slice<3, 1>(3, 0) = _vel;
	input_x.slice<3, 1>(6, 0) = matrix::Eulerf(_att);
	input_x.slice<3, 1>(9, 0) = _ang_vel;
}

bool NNBSPositionControl::_updateSuccessful()
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_thr_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	// There has to be a valid output accleration and thrust setpoint otherwise there was no
	// setpoint-state pair for each axis that can get controlled
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void NNBSPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void NNBSPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
