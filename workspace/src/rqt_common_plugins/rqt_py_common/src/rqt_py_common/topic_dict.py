# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
import rospy


class TopicDict(object):

    def __init__(self):
        self.update_topics()

    def get_topics(self):
        return self.topic_dict

    def update_topics(self):
        self.topic_dict = {}
        topic_list = rospy.get_published_topics()
        for topic_name, topic_type in topic_list:
            message = roslib.message.get_message_class(topic_type)()
            self.topic_dict.update(self._recursive_create_field_dict(topic_name, message))

    def _recursive_create_field_dict(self, topic_name, field):
        field_dict = {}
        field_dict[topic_name] = {
            'type': type(field),
            'children': {},
        }

        if hasattr(field, '__slots__'):
            for slot_name in field.__slots__:
                field_dict[topic_name]['children'].update(self._recursive_create_field_dict(slot_name, getattr(field, slot_name)))

        return field_dict


if __name__ == '__main__':
    import pprint
    pprint.pprint(TopicDict().get_topics())
