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

"""Casadi EKF C Code Generator."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import casadi as ca
import shutil
import sys
import tempfile
from pathlib import Path
project_src = Path(__file__).resolve().parents[1]
print(f'Adding {project_src} to sys.path')
sys.path.insert(0, str(project_src))
from ekf_definition.ekf import EKF

CODE_PREFIX = 'calib_ekf_c_code'


def main():
    """
    Main function to generate C code for the EKF.
    """
    parser = argparse.ArgumentParser(description='Generate EKF C code.')
    parser.add_argument('--force', action='store_true',
                        help='Overwrite existing generated code without asking.')
    args = parser.parse_args()

    src_dst = project_src / 'src' / f'{CODE_PREFIX}.cpp'
    hdr_dst = project_src / 'include' / 'ekf_calib' / f'{CODE_PREFIX}.h'

    if src_dst.exists() and not args.force:
        confirm = input(
            'Previous C code found. Do you want to overwrite it? (y/N): ')
        if confirm.lower() != 'y':
            print('C code generation aborted.')
            return

    # Create an instance of the EKF
    ekf = EKF()

    # Define the options for the code generation
    opts = {
        'with_header': True,
        'verbose': True,
        'cpp': True,
    }

    # Generate C code into a temporary directory
    with tempfile.TemporaryDirectory() as tmp:
        c = ca.CodeGenerator(f'{CODE_PREFIX}', opts)
        c.add(ekf.predict_function)
        c.add(ekf.pose_function)
        c.add(ekf.velocity_function)
        c.add(ekf.point_function)
        c.generate(f'{tmp}/')

        hdr_dst.parent.mkdir(parents=True, exist_ok=True)
        src_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(f'{tmp}/{CODE_PREFIX}.cpp', src_dst)
        shutil.copy(f'{tmp}/{CODE_PREFIX}.h', hdr_dst)

    print(f'C code generated successfully:\n  {src_dst}\n  {hdr_dst}')


if __name__ == '__main__':
    main()
