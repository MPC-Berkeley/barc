#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz
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

from python_qt_binding.QtCore import Slot, Qt, qWarning, Signal
from python_qt_binding.QtGui import QColor, QVBoxLayout, QWidget

from pyqtgraph import PlotWidget, mkPen, mkBrush
import numpy


class PyQtGraphDataPlot(QWidget):

    limits_changed = Signal()

    def __init__(self, parent=None):
        super(PyQtGraphDataPlot, self).__init__(parent)
        self._plot_widget = PlotWidget()
        self._plot_widget.getPlotItem().addLegend()
        self._plot_widget.setBackground((255, 255, 255))
        self._plot_widget.setXRange(0, 10, padding=0)
        vbox = QVBoxLayout()
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)
        self._plot_widget.getPlotItem().sigRangeChanged.connect(self.limits_changed)

        self._curves = {}
        self._current_vline = None

    def add_curve(self, curve_id, curve_name, curve_color=QColor(Qt.blue), markers_on=False):
        pen = mkPen(curve_color, width=1)
        symbol = "o"
        symbolPen = mkPen(QColor(Qt.black))
        symbolBrush = mkBrush(curve_color)
        # this adds the item to the plot and legend
        if markers_on:
            plot = self._plot_widget.plot(name=curve_name, pen=pen, symbol=symbol, symbolPen=symbolPen, symbolBrush=symbolBrush, symbolSize=4)
        else:
            plot = self._plot_widget.plot(name=curve_name, pen=pen)
        self._curves[curve_id] = plot

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._plot_widget.removeItem(self._curves[curve_id])
            del self._curves[curve_id]
            self._update_legend()
           
    def _update_legend(self):
        # clear and rebuild legend (there is no remove item method for the legend...)
        self._plot_widget.clear()
        self._plot_widget.getPlotItem().legend.items = []
        for curve in self._curves.values():
            self._plot_widget.addItem(curve)
        if self._current_vline:
            self._plot_widget.addItem(self._current_vline)
 
    def redraw(self):
        pass

    def set_values(self, curve_id, data_x, data_y):
        curve = self._curves[curve_id]
        curve.setData(data_x, data_y)

    def vline(self, x, color):
        if self._current_vline:
            self._plot_widget.removeItem(self._current_vline)
        self._current_vline = self._plot_widget.addLine(x=x, pen=color)

    def set_xlim(self, limits):
        # TODO: this doesn't seem to handle fast updates well
        self._plot_widget.setXRange(limits[0], limits[1], padding=0)

    def set_ylim(self, limits):
        self._plot_widget.setYRange(limits[0], limits[1], padding=0)

    def get_xlim(self):
        x_range, _ = self._plot_widget.viewRange()
        return x_range

    def get_ylim(self):
        _, y_range = self._plot_widget.viewRange()
        return y_range
