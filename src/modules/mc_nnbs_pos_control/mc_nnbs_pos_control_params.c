/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file mc_nnbs_pos_control_params.c
 * Multicopter NNBS position controller parameters.
 *
 * @author Peiyan Liu =
 */

/**
 * Backstepping parameter k1
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K1, 2.0f);

/**
 * Backstepping parameter k2
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K2, 3.0f);

/**
 * Backstepping parameter k3
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K3, 2.0f);

/**
 * Backstepping parameter k4
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K4, 3.0f);

/**
 * Backstepping parameter k5
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K5, 5.0f);

/**
 * Backstepping parameter k6
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_K6, 5.0f);


/**
 * Cutoff frequency of the first-order command filter (if used) for alpha_dot.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_WC1, 1.0f);

/**
 * Cutoff frequency of the first-order command filter (if used) for alpha_dot.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_WC2, 1.0f);

/**
 * Cutoff frequency of the first-order command filter (if used) for alpha_dot.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_WC3, 1.0f);

/**
 * Whether use command filter for alpha_dot or not.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_NNBS_CMD_FLT, 0);

/**
 * Standard error of an RBF node.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_RBF_B, 0.5f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XDA_MIN, -20.0f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XDA_MAX, 20.0f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YDA_MIN, -20.0f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YDA_MAX, 20.0f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZDA_MIN, -20.0f);

/**
 * Limits of alpha_dot
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZDA_MAX, 20.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XNN_MIN, -5.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XNN_MAX, 5.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YNN_MIN, -5.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YNN_MAX, 5.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZNN_MIN, -5.0f);

/**
 * Limits of neural network outputs
 *
 * @min -100.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZNN_MAX, 5.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XE_MIN, -100.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_XE_MAX, 100.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YE_MIN, -100.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_YE_MAX, 100.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZE_MIN, -100.0f);

/**
 * Limits of neural network errors
 *
 * @min -1000.0
 * @max 1000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_ZE_MAX, 100.0f);

/**
 * Learning rate of weights (10^3).
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_LRW_X, 0.1f);

/**
 * Learning rate of weights (10^3)
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_LRW_Y, 0.1f);

/**
 * Learning rate of weights (10^3)
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_NNBS_LRW_Z, 0.1f);
