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

import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QUrl
from python_qt_binding.QtGui import QCompleter, QIcon, QWidget
from python_qt_binding.QtWebKit import QWebPage, QWebView


class WebWidget(QWidget):
    def __init__(self, url=None):
        """
        Class to load a webpage in a widget.

        :param url: If url is empty then a navigation bar is shown otherwise the url is loaded and the navigation bar is hidden, ''str''
        """
        super(WebWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_web'), 'resource', 'web_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('WebWidget')

        self._loading = False
        self._stop_icon = QIcon.fromTheme('process-stop')
        self._reload_icon = QIcon.fromTheme('view-refresh')
        self._working_icon = QIcon.fromTheme('process-working')

        self._completer_word_list = ['']
        self._view = QWebView()
        self.verticalLayout.addWidget(self._view)
        if url is None:
            self.set_url("http://ros.org", True)
        else:
            self.set_url(url, False)

        self.url_lineedit.returnPressed.connect(self._handle_url_change)
        self._view.loadFinished[bool].connect(self._handle_load_finished)
        self.reload_button.clicked.connect(self._handle_reload_clicked)
        self._view.linkClicked.connect(self._handle_link_clicked)
        self._view.urlChanged[QUrl].connect(self._handle_url_changed)

    def set_url(self, url, showinput=False):
        """
        Sets the url and begins loading that page
        :param url: url to load in the webview, ''str or QUrl''
        :param showinput: if true the input bar will be shown, else hidden, ''bool''
        """
        if url is not None:
            self._url = QUrl(url)
            self.set_show_url_input(showinput)
            self._view.setUrl(self._url)

    def set_show_url_input(self, showinput):
        """
        Sets the value of the show_url_input flag and hides/shows the widgets as required
        :param showinput: true - show inputbar false - hide , ''bool''
        """
        self._show_url_input = showinput
        self.url_lineedit.setVisible(self._show_url_input)
        self.reload_button.setVisible(self._show_url_input)
        if self._show_url_input:
            self._view.page().setLinkDelegationPolicy(QWebPage.DelegateAllLinks)
        else:
            self._view.page().setLinkDelegationPolicy(QWebPage.DontDelegateLinks)

    def save_settings(self, settings):
        settings.set_value('url_completion', self._pack(self._completer_word_list))
        settings.set_value('url_current', self._url.toString())

    def restore_settings(self, settings):
        self._completer_word_list += self._unpack(settings.value('url_completion'))
        self._completer_word_list = list(set(self._completer_word_list))
        url = settings.value('url_current')
        if url:
            self.set_url(url, self._show_url_input)

    def _handle_url_change(self):
        self.set_url(self.url_lineedit.text(), True)

    def _handle_link_clicked(self, url):
        self.set_url(url, True)

    def _handle_reload_clicked(self):
        if self._loading:
            self._view.stop()
            self._loading = False
            self.reload_button.setIcon(self._reload_icon)
        else:
            self._view.reload()
            self._loading = True
            self.reload_button.setIcon(self._stop_icon)

    def _handle_url_changed(self, url):
        # set text to the current loading item
        self.url_lineedit.setText(url.toString())
        self.reload_button.setIcon(self._stop_icon)
        self._loading = True

    def _handle_load_finished(self, ok):
        self._loading = False
        self.reload_button.setIcon(self._reload_icon)
        if ok:
            self._add_completer_list_item(self._url.toString())
        else:
            # need to disconnect or we will resend the signal once the error page loads
            self._view.loadFinished[bool].disconnect(self._handle_load_finished)
            self._view.page().currentFrame().setHtml('<html><h2>The url you entered seems to be faulty.</h2></html>')
            self._view.loadFinished[bool].connect(self._handle_load_finished)

    def _add_completer_list_item(self, url):
        self._completer_word_list.append(self.url_lineedit.text())
        self._completer_word_list = list(set(self._completer_word_list))
        self._completer = QCompleter(self._completer_word_list)
        self._completer.setCaseSensitivity(Qt.CaseInsensitive)
        self._completer.setCompletionMode(QCompleter.PopupCompletion)
        self.url_lineedit.setCompleter(self._completer)

    @staticmethod
    def _pack(data):
        """
        Packs 'data' into a form that can be easily and readably written to an ini file
        :param data: A list of strings to be flattened into a string ''list''
        :return: A string suitable for output to ini files ''str''
        """
        if len(data) == 0:
            return ''
        if len(data) == 1:
            return data[0]
        return data

    @staticmethod
    def _unpack(data):
        """
        Unpacks the values read from an ini file
        :param data: An entry taken from an ini file ''list or string''
        :return: A list of strings ''list''
        """
        if data is None or data == '':
            data = []
        elif isinstance(data, basestring):
            data = [data]
        return data
