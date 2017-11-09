#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (C) 2013, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Run a specified linter in a subprocess, checking its return status.

This is a unit test script for roslint.
"""

from __future__ import print_function

import sys
import subprocess

def main(returnstatus, linter, files):
    """ Main program.

    :param: returnstatus expected return status
    :param: linter command string for running the linter with its options.
    :param: files list of file names to process

    :returns: 0 if test successful, 1 otherwise.
    """
    cmd = linter.split() + files
    print('lint command: ' + ' '.join(cmd))
    sub_rc = subprocess.call(cmd)
    if sub_rc != int(returnstatus):
        return 1
    else:
        return 0


if __name__ == '__main__':

    # for some reason, pylint wants this variable name to be uppercase:
    #pylint: disable=C0103
    ret_status = -9             # return status for missing args
    if len(sys.argv) > 3:
        ret_status = main(sys.argv[1], sys.argv[2], sys.argv[3:])
    else:
        print('usage: runlint returnstatus linter file1 [ file2 ... ]')
    sys.exit(ret_status)
