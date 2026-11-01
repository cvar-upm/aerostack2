#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""EKF definition."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import casadi as ca
from ekf_definition.casadi_utils import Utils


class EKF():
    """Extended Kalman Filter (EKF) class."""

    def f_continuous(self, X, input_acc, input_angular_velocity):
        """
        Continuous time state transition function.

        :param X: State vector.
        :param input_acc: Input acceleration vector (measured - bias).
        :return: State derivative.
        """
        # Derivatives
        p_dot = X[3:6]  # v
        v_dot = Utils.quaternion_velocity_derivative(
            X[6:10],
            input_acc,
            self.g
        )  # R(q)*(a_meas − b_a) + g
        q_dot = Utils.quaternion_derivate(
            X[6:10],
            input_angular_velocity,
        )  # 0.5 q x [0, w_meas − b_w]
        biases_dot = ca.SX.zeros(6, 1)  # biases are constant
        # New state
        return ca.vertcat(
            p_dot,
            v_dot,
            q_dot,
            biases_dot
        )

    def __init__(self):
        """Initialize the EKF."""
        # Time step
        self.dt = ca.SX.sym('dt')
        # self.g = ca.DM(9.81)  # Gravity constant
        self.g = ca.SX.sym('g', 3)  # Gravity vector (3D)

        # State vector
        # x, y, z, vx, vy, vz, qw, qx, qy, qz, abx, aby, abz, wbx, wby, wbz
        # The quaternion is base->map (Hamilton, w first). Nothing here keeps it unit: the
        # wrapper renormalizes it after every step, and the covariance is left as it is
        self.X = ca.SX.sym('X', 16)
        # self.x, self.y, self.z, \
        #     self.vx, self.vy, self.vz, \
        #     self.qw, self.qx, self.qy, self.qz, \
        #     self.abx, self.aby, self.abz, \
        #     self.wbx, self.wby, self.wbz = ca.vertsplit(self.X)
        state_position = self.X[0:3]
        state_velocity = self.X[3:6]
        state_orientation = self.X[6:10]
        state_accelerometer_bias = self.X[10:13]
        state_gyrometer_bias = self.X[13:16]
        state_bias = ca.vertcat(
            state_accelerometer_bias,
            state_gyrometer_bias
        )

        # Inputs
        # axm, aym, azm, wxm, wym, wzm
        self.U = ca.SX.sym('U', 6)
        # self.axm, self.aym, self.azm, \
        #     self.wxm, self.wym, self.wzm = ca.vertsplit(self.U)
        # input_acceleration = self.U[0:3]
        # input_angular_velocity = self.U[3:6]

        # Inputs noise
        # axw, ayw, azw, wxw, wyw, wzw
        self.W = ca.SX.sym('W', 6)
        # self.axw, self.ayw, self.azw, \
        #     self.wxw, self.wyw, self.wzw = ca.vertsplit(self.W)
        # input_noise_acceleration = self.W[0:3]
        # input_noise_angular_velocity = self.W[3:6]

        # Inputs without bias and noise
        # iax, iay, iaz, iwx, iwy, iwz
        self.IN = self.U - state_bias - self.W
        # self.iax, self.iay, self.iaz, \
        #     self.iwx, self.iwy, self.iwz = ca.vertsplit(self.IN)
        input_wo_noise_acceleration = self.IN[0:3]
        input_wo_noise_angular_velocity = self.IN[3:6]

        # Runge-Kutta 4th order integration for state transition function
        k1 = self.f_continuous(self.X,
                               input_wo_noise_acceleration,
                               input_wo_noise_angular_velocity)
        k2 = self.f_continuous(self.X + 0.5 * self.dt * k1,
                               input_wo_noise_acceleration,
                               input_wo_noise_angular_velocity)
        k3 = self.f_continuous(self.X + 0.5 * self.dt * k2,
                               input_wo_noise_acceleration,
                               input_wo_noise_angular_velocity)
        k4 = self.f_continuous(self.X + self.dt * k3,
                               input_wo_noise_acceleration,
                               input_wo_noise_angular_velocity)

        self.f_step = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.f = self.X + self.dt * self.f_step

        acc_in_world = self.f_step[3:6]

        # print(self.f)

        # Pose measurement: x, y, z, qw, qx, qy, qz, in the map frame. Its residual has six
        # components, the position difference and the small rotation from the state's
        # orientation to the measured one about the map's axes, 2 vec(q_meas x q_state^-1).
        # The caller puts q_meas on the state's hemisphere (q and -q are the same rotation).
        self.Z_pose = ca.SX.sym('Z_pose', 7)
        orientation_error = Utils.quaternion_multiply(
            self.Z_pose[3:7],
            Utils.quaternion_conjugate(Utils.normalize_quaternion(state_orientation)))
        self.Y_residual_pose = ca.vertcat(
            self.Z_pose[0:3] - state_position,
            2.0 * orientation_error[1:4],
        )

        # Output function velocity
        # vx, vy, vz
        self.h_velocity = ca.vertcat(
            state_velocity
        )

        # Jacobians
        self.F = ca.jacobian(self.f, self.X)
        self.L = ca.jacobian(self.f, self.W)
        # The residual is not z - h(X): its Jacobian is minus the measurement model's
        self.H_pose = -ca.jacobian(self.Y_residual_pose, self.X)
        self.H_velocity = ca.jacobian(self.h_velocity, self.X)

        # Substitute W with 0
        self.f = ca.substitute(self.f, self.W, 0)
        self.F = ca.substitute(self.F, self.W, 0)
        self.L = ca.substitute(self.L, self.W, 0)
        self.H_pose = ca.substitute(self.H_pose, self.W, 0)
        self.H_velocity = ca.substitute(self.H_velocity, self.W, 0)

        # print("F:", self.F)
        # print("L:", self.L)

        # covariance and extra matrices
        # State covariance matrix
        self.P = ca.SX.sym('P', self.X.size()[0], self.X.size()[0])

        # Process Noise covariance matrix
        # self.aux_Q_vector = ca.SX.sym('Q', self.W.size()[0])
        # self.Q = ca.SX.zeros(self.W.size()[0], self.W.size()[0])
        # for i in range(self.W.size()[0]):
        #     for j in range(i, self.W.size()[0]):
        #         if i == j:
        #             self.Q[i, j] = self.aux_Q_vector[i]
        self.Q = ca.SX.sym('Q', self.X.size()[0], self.X.size()[0])

        # Predict step
        # X_pred = new state prediction
        # f = result of the state transition function
        # P_pred = new state covariance prediction
        # F = Jacobian of f with respect to state
        # P = current state covariance
        # Q = process noise covariance
        self.X_pred = self.f
        # self.P_pred = self.F @ self.P @ self.F.T + self.L @ self.Q @ self.L.T
        self.P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update step with pose measurement
        # Measurement Noise covariance matrix: x, y, z and the rotation about the map's x, y, z
        residual_size = self.Y_residual_pose.size()[0]
        self.aux_R_vector_pose = ca.SX.sym('R_pose', residual_size)
        self.R_pose = ca.SX.zeros(residual_size, residual_size)
        for i in range(residual_size):
            self.R_pose[i, i] = self.aux_R_vector_pose[i]

        # Y_residual_pose = measurement residual (defined above)
        # Z_pose = measurement
        # S_pose = residual covariance
        # H_pose = minus the Jacobian of the residual with respect to state
        # P = current state covariance
        # R_pose = measurement noise covariance
        # K_pose = Kalman gain
        # X_update_pose = updated state
        # P_update_pose = updated state covariance
        self.S_pose = self.H_pose @ self.P @ self.H_pose.T + self.R_pose
        self.K_pose = self.P @ self.H_pose.T @ ca.pinv(self.S_pose)

        self.X_update_pose = self.X + self.K_pose @ self.Y_residual_pose
        self.P_update_pose = (
            ca.SX.eye(self.X.size()[0]) - self.K_pose @ self.H_pose) @ self.P

        # Update step with pose and velocity measurement
        # Measurement vector (velocity)
        self.Z_velocity = ca.SX.sym(
            'Z_velocity', self.h_velocity.size()[0])
        # Measurement Noise covariance matrix
        self.aux_R_vector_velocity = ca.SX.sym(
            'R_velocity', self.h_velocity.size()[0])
        self.R_velocity = ca.SX.zeros(self.h_velocity.size()[
                                           0], self.h_velocity.size()[0])
        for i in range(self.h_velocity.size()[0]):
            for j in range(i, self.h_velocity.size()[0]):
                if i == j:
                    self.R_velocity[i, j] = self.aux_R_vector_velocity[i]

        self.Y_residual_velocity = self.Z_velocity - self.h_velocity
        self.S_velocity = self.H_velocity @ self.P @ self.H_velocity.T + \
            self.R_velocity
        self.K_velocity = self.P @ self.H_velocity.T @ ca.pinv(
            self.S_velocity)

        self.X_update_velocity = self.X + \
            self.K_velocity @ self.Y_residual_velocity
        self.P_update_velocity = (
            ca.SX.eye(self.X.size()[0]) - self.K_velocity @ self.H_velocity) @ self.P

        # Functions
        # Define the CasADi function for prediction
        # self.predict_function = ca.Function(
        #     'predict_function',
        #     [self.X, self.U, self.W, self.dt, self.P, self.aux_Q_vector, self.g],
        #     [self.X_pred, self.P_pred, acc_in_world],
        #     ['X', 'U', 'W', 'dt', 'P', 'Q', 'g'],
        #     ['X_pred', 'P_pred', 'acc_in_world']
        # )
        self.predict_function = ca.Function(
            'predict_function',
            [self.X, self.U, self.W, self.dt, self.P, self.Q, self.g],
            [self.X_pred, self.P_pred, acc_in_world],
            ['X', 'U', 'W', 'dt', 'P', 'Q', 'g'],
            ['X_pred', 'P_pred', 'acc_in_world']
        )
        # Define the CasADi function for update with pose measurement
        self.update_pose_function = ca.Function(
            'update_pose_function',
            [self.X, self.W, self.Z_pose, self.P, self.aux_R_vector_pose],
            [self.X_update_pose, self.P_update_pose],
            ['X', 'W', 'Z_pose', 'P', 'R_pose'],
            ['X_update_pose', 'P_update_pose']
        )
        # Define the CasADi function for update with velocity measurement
        self.update_velocity_function = ca.Function(
            'update_velocity_function',
            [self.X, self.W, self.Z_velocity,
                self.P, self.aux_R_vector_velocity],
            [self.X_update_velocity, self.P_update_velocity],
            ['X', 'W', 'Z_velocity', 'P', 'R_velocity'],
            ['X_update_velocity', 'P_update_velocity']
        )
