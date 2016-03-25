# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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


class MessageList(object):

    def __init__(self):
        super(MessageList, self).__init__()
        self._messages = []

    def __getitem__(self, key):
        return self._messages[len(self._messages) - key - 1]

    def __delitem__(self, key):
        if isinstance(key, slice):
            assert key.step is None or key.step == 1, 'MessageList.__delitem__ not implemented for slices with step argument different than 1'
            del self._messages[len(self._messages) - key.stop:len(self._messages) - key.start]
        else:
            del self._messages[len(self._messages) - key - 1]

    def __iter__(self):
        return reversed(self._messages)

    def __reversed__(self):
        return iter(self._messages)

    def __contains__(self, item):
        return item in self._messages

    def __len__(self):
        return len(self._messages)

    def extend(self, item):
        self._messages.extend(item)
