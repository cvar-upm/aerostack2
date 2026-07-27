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

"""Calibration EKF definition (quaternion attitude)."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import casadi as ca
from ekf_definition.casadi_utils import Utils

# State layout
N_STATES = 26
IDX_POSITION = slice(0, 3)
IDX_VELOCITY = slice(3, 6)
IDX_QUATERNION = slice(6, 10)
IDX_ACC_BIAS = slice(10, 13)
IDX_GYRO_BIAS = slice(13, 16)
IDX_EXTRINSIC_TRANSLATION = slice(16, 19)
IDX_EXTRINSIC_ROTATION = slice(19, 22)
IDX_INTRINSICS = slice(22, 26)

# Number of calibration states: extrinsics (6) + intrinsics (4)
N_CALIBRATION_STATES = 10


class EKF():
    """
    Extended Kalman Filter (EKF) class.

    Only the model is defined here: the state transition function, the
    measurement functions, and their Jacobians. The covariance propagation and
    the Kalman gain are computed at runtime with Eigen (see ekf_wrapper.cpp),
    so P, Q and R are deliberately not part of the generated code.
    """

    def f_continuous(self, X, input_acc, input_angular_velocity):
        """
        Continuous time state transition function.

        :param X: State vector.
        :param input_acc: Input acceleration vector (measured - bias - noise).
        :param input_angular_velocity: Angular velocity (measured - bias - noise).
        :return: State derivative.
        """
        quaternion = X[IDX_QUATERNION]
        # Derivatives
        p_dot = X[IDX_VELOCITY]  # v, world frame
        # R(q)*(a_meas - b_a) - g. The quaternion is normalized because the
        # intermediate RK4 stages below evaluate this at |q| != 1.
        v_dot = Utils.apply_rotation(
            Utils.normalize_quaternion(quaternion),
            input_acc
        ) - self.g
        q_dot = Utils.quaternion_derivate(
            quaternion,
            input_angular_velocity,
        )
        biases_dot = ca.SX.zeros(6, 1)  # biases are constant
        # The camera calibration is constant in the mean: it is driven only by
        # the random walk terms of the process noise, which are applied in
        # ekf_wrapper.cpp. Its rows of F are therefore exactly the identity, and
        # no navigation derivative depends on it, so the calibration block is
        # decoupled in F. All calibration information must come from the points
        # measurement.
        calibration_dot = ca.SX.zeros(N_CALIBRATION_STATES, 1)
        # New state
        return ca.vertcat(
            p_dot,
            v_dot,
            q_dot,
            biases_dot,
            calibration_dot
        )

    def _reprojection(self, p_world):
        """
        Project a known world point into the camera image.

        :param p_world: The landmark position in the map frame.
        :return: A tuple (pixel, depth), where pixel is [u, v] and depth is the
                 coordinate along the optical axis. The projection divides by
                 depth without guarding it, so the caller must reject points at
                 or behind the camera.
        """
        quaternion = self.X[IDX_QUATERNION]
        position = self.X[IDX_POSITION]
        extrinsic_translation = self.X[IDX_EXTRINSIC_TRANSLATION]
        ephi, etheta, epsi = ca.vertsplit(self.X[IDX_EXTRINSIC_ROTATION])
        fx, fy, cx, cy = ca.vertsplit(self.X[IDX_INTRINSICS])

        # World -> body
        quaternion_inverse = ca.vertcat(
            quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])
        p_body = Utils.apply_rotation(
            Utils.normalize_quaternion(quaternion_inverse),
            p_world - position
        )

        # Body -> camera, about the nominal mount
        c_phi, s_phi = ca.cos(ephi), ca.sin(ephi)
        c_theta, s_theta = ca.cos(etheta), ca.sin(etheta)
        c_psi, s_psi = ca.cos(epsi), ca.sin(epsi)
        rot_x = ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, c_phi, -s_phi),
            ca.horzcat(0, s_phi, c_phi))
        rot_y = ca.vertcat(
            ca.horzcat(c_theta, 0, s_theta),
            ca.horzcat(0, 1, 0),
            ca.horzcat(-s_theta, 0, c_theta))
        rot_z = ca.vertcat(
            ca.horzcat(c_psi, -s_psi, 0),
            ca.horzcat(s_psi, c_psi, 0),
            ca.horzcat(0, 0, 1))
        rotation_extrinsic = self.R_nom @ (rot_z @ rot_y @ rot_x)
        p_camera = rotation_extrinsic.T @ (p_body - extrinsic_translation)

        # Camera frame is forward-right-down; OpenCV wants right-down-forward.
        x_optical = p_camera[1]
        y_optical = p_camera[2]
        depth = p_camera[0]

        # Pinhole projection. Note u and v are linear in the intrinsics, so that
        # Jacobian block is exact: d(u,v)/d(fx,fy,cx,cy) = [[xn,0,1,0],
        # [0,yn,0,1]] with xn, yn the normalized image coordinates.
        u = fx * x_optical / depth + cx
        v = fy * y_optical / depth + cy

        return ca.vertcat(u, v), depth

    def __init__(self):
        """
        Initialize the EKF.
        """
        # Time step
        self.dt = ca.SX.sym('dt')
        self.g = ca.SX.sym('g', 3)  # Gravity vector (3D)

        # Nominal camera mount rotation (camera -> body), a runtime parameter.
        # The estimated extrinsic angles are deltas about this nominal, which
        # keeps them near zero. Without it, Re = Rz Ry Rx gimbal locks at
        # etheta = -pi/2, which is exactly a nadir pointing camera: the optical
        # axis is camera frame x (see the axis permutation below), so a nadir
        # mount needs Re * e_x = (0, 0, 1), i.e. etheta = -pi/2. Factoring the
        # mount out removes the singularity for any orientation.
        self.R_nom = ca.SX.sym('R_nom', 3, 3)

        # State vector
        # x, y, z, vx, vy, vz, qw, qx, qy, qz, abx, aby, abz, wbx, wby, wbz,
        # ex, ey, ez, ephi, etheta, epsi, fx, fy, cx, cy
        self.X = ca.SX.sym('X', N_STATES)
        state_position = self.X[IDX_POSITION]
        state_velocity = self.X[IDX_VELOCITY]
        state_quaternion = self.X[IDX_QUATERNION]
        state_bias = ca.vertcat(
            self.X[IDX_ACC_BIAS],
            self.X[IDX_GYRO_BIAS]
        )

        # Inputs
        # axm, aym, azm, wxm, wym, wzm
        self.U = ca.SX.sym('U', 6)

        # Inputs noise
        # axw, ayw, azw, wxw, wyw, wzw
        self.W = ca.SX.sym('W', 6)

        # Inputs without bias and noise
        self.IN = self.U - state_bias - self.W
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

        # Output function pose
        # x, y, z, qw, qx, qy, qz
        self.h_pose = ca.vertcat(
            state_position,
            state_quaternion,
        )

        # Output function velocity
        # vx, vy, vz
        self.h_velocity = ca.vertcat(
            state_velocity
        )

        # Output function for a single reprojected landmark.
        # The landmark is a known map point supplied as a parameter, not a
        # state: this is calibration against a known map, not SLAM.
        self.p_w = ca.SX.sym('p_w', 3)
        self.h_point, self.depth = self._reprojection(self.p_w)

        # Jacobians
        self.F = ca.jacobian(self.f, self.X)
        self.L = ca.jacobian(self.f, self.W)
        self.H_pose = ca.jacobian(self.h_pose, self.X)
        self.H_velocity = ca.jacobian(self.h_velocity, self.X)
        self.H_point = ca.jacobian(self.h_point, self.X)

        # Substitute W with 0
        self.f = ca.substitute(self.f, self.W, 0)
        self.F = ca.substitute(self.F, self.W, 0)
        self.L = ca.substitute(self.L, self.W, 0)
        self.H_pose = ca.substitute(self.H_pose, self.W, 0)
        self.H_velocity = ca.substitute(self.H_velocity, self.W, 0)

        # Functions.
        # Every matrix output is densified: CasADi would otherwise emit only the
        # structurally non-zero entries in compressed-column form, and the C++
        # side maps these buffers as plain dense column-major arrays.
        self.predict_function = ca.Function(
            'calib_predict_function',
            [self.X, self.U, self.dt, self.g],
            [self.f, ca.densify(self.F), ca.densify(self.L)],
            ['X', 'U', 'dt', 'g'],
            ['f', 'F', 'L']
        )
        self.pose_function = ca.Function(
            'calib_pose_function',
            [self.X],
            [self.h_pose, ca.densify(self.H_pose)],
            ['X'],
            ['h_pose', 'H_pose']
        )
        self.velocity_function = ca.Function(
            'calib_velocity_function',
            [self.X],
            [self.h_velocity, ca.densify(self.H_velocity)],
            ['X'],
            ['h_velocity', 'H_velocity']
        )
        # One landmark per call: the C++ side loops and stacks the rows, so the
        # generated code size does not depend on the maximum landmark count and
        # the active count is free to vary at runtime. h, H and depth share the
        # whole reprojection chain through common subexpression elimination.
        self.point_function = ca.Function(
            'calib_point_function',
            [self.X, self.p_w, self.R_nom],
            [self.h_point, ca.densify(self.H_point), self.depth],
            ['X', 'p_w', 'R_nom'],
            ['h_point', 'H_point', 'depth']
        )
