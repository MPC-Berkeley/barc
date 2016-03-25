# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

import threading
import time


class IndexCacheThread(threading.Thread):
    """
    Updates invalid caches.
    One thread per timeline.
    """
    def __init__(self, timeline):
        threading.Thread.__init__(self)
        self.timeline = timeline
        self._stop_flag = False
        self.setDaemon(True)
        self.start()

    def run(self):
        while not self._stop_flag:
            with self.timeline.index_cache_cv:
                # Wait until the cache is dirty
                while len(self.timeline.invalidated_caches) == 0:
                    self.timeline.index_cache_cv.wait()
                    if self._stop_flag:
                        return
                # Update the index for one topic
                total_topics = len(self.timeline.topics)
                update_step = max(1, total_topics / 100)
                topic_num = 1
                progress = 0
                updated = False
                for topic in self.timeline.topics:
                    if topic in self.timeline.invalidated_caches:
                        updated = (self.timeline._update_index_cache(topic) > 0)
                    if topic_num % update_step == 0 or topic_num == total_topics:
                        new_progress = int(100.0 * (float(topic_num) / total_topics))
                        if new_progress != progress:
                            progress = new_progress
                            if not self._stop_flag:
                                self.timeline.scene().background_progress = progress
                                self.timeline.scene().status_bar_changed_signal.emit()
                    topic_num += 1

            if updated:
                self.timeline.scene().background_progress = 0
                self.timeline.scene().status_bar_changed_signal.emit()
                self.timeline.scene().update()
                # Give the GUI some time to update
                time.sleep(1.0)

    def stop(self):
        self._stop_flag = True
        cv = self.timeline.index_cache_cv
        with cv:
            cv.notify()
