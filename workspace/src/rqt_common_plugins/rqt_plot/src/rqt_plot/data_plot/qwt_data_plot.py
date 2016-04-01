#!/usr/bin/env python

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

# -*- coding: utf-8 -*-
from __future__ import division
import math
import sys

from python_qt_binding.QtCore import QEvent, QSize, QPointF, Qt, SIGNAL, Signal, Slot, qWarning
from python_qt_binding.QtGui import QColor, QPen, QBrush, QVector2D
import Qwt

from numpy import arange, zeros, concatenate


# create real QwtDataPlot class
class QwtDataPlot(Qwt.QwtPlot):
    mouseCoordinatesChanged = Signal(QPointF)
    _num_value_saved = 1000
    _num_values_ploted = 1000

    limits_changed = Signal()

    def __init__(self, *args):
        super(QwtDataPlot, self).__init__(*args)
        self.setCanvasBackground(Qt.white)
        self.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.BottomLegend)

        self._curves = {}

        # TODO: rejigger these internal data structures so that they're easier
        # to work with, and easier to use with set_xlim and set_ylim
        #  this may also entail rejiggering the _time_axis so that it's
        #  actually time axis data, or just isn't required any more
        # new data structure
        self._x_limits = [0.0, 10.0]
        self._y_limits = [0.0, 10.0]

        # old data structures
        self._last_canvas_x = 0
        self._last_canvas_y = 0
        self._pressed_canvas_y = 0
        self._pressed_canvas_x = 0
        self._last_click_coordinates = None

        marker_axis_y = Qwt.QwtPlotMarker()
        marker_axis_y.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        marker_axis_y.setLineStyle(Qwt.QwtPlotMarker.HLine)
        marker_axis_y.setYValue(0.0)
        marker_axis_y.attach(self)

        self._picker = Qwt.QwtPlotPicker(
            Qwt.QwtPlot.xBottom, Qwt.QwtPlot.yLeft, Qwt.QwtPicker.PolygonSelection,
            Qwt.QwtPlotPicker.PolygonRubberBand, Qwt.QwtPicker.AlwaysOn, self.canvas()
        )
        self._picker.setRubberBandPen(QPen(Qt.blue))
        self._picker.setTrackerPen(QPen(Qt.blue))

        # Initialize data
        self.rescale()
        #self.move_canvas(0, 0)
        self.canvas().setMouseTracking(True)
        self.canvas().installEventFilter(self)

    def eventFilter(self, _, event):
        if event.type() == QEvent.MouseButtonRelease:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            self._last_click_coordinates = QPointF(x, y)
            self.limits_changed.emit()
        elif event.type() == QEvent.MouseMove:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            coords = QPointF(x, y)
            if self._picker.isActive() and self._last_click_coordinates is not None:
                toolTip = 'origin x: %.5f, y: %.5f' % (self._last_click_coordinates.x(), self._last_click_coordinates.y())
                delta = coords - self._last_click_coordinates
                toolTip += '\ndelta x: %.5f, y: %.5f\nlength: %.5f' % (delta.x(), delta.y(), QVector2D(delta).length())
            else:
                toolTip = 'buttons\nleft: measure\nmiddle: move\nright: zoom x/y\nwheel: zoom y'
            self.setToolTip(toolTip)
            self.mouseCoordinatesChanged.emit(coords)
        return False

    def log(self, level, message):
        self.emit(SIGNAL('logMessage'), level, message)

    def resizeEvent(self, event):
        Qwt.QwtPlot.resizeEvent(self, event)
        self.rescale()

    def add_curve(self, curve_id, curve_name, curve_color=QColor(Qt.blue), markers_on=False):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            return
        curve_object = Qwt.QwtPlotCurve(curve_name)
        curve_object.attach(self)
        curve_object.setPen(curve_color)
        if markers_on:
            curve_object.setSymbol(Qwt.QwtSymbol(Qwt.QwtSymbol.Ellipse, QBrush(curve_color), QPen(Qt.black), QSize(4,4)))
        self._curves[curve_id] = curve_object

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id].hide()
            self._curves[curve_id].attach(None)
            del self._curves[curve_id]

    def set_values(self, curve_id, data_x, data_y):
        curve = self._curves[curve_id]
        curve.setData(data_x, data_y)

    def redraw(self):
        self.replot()

    # ----------------------------------------------
    # begin qwtplot internal methods
    # ----------------------------------------------
    def rescale(self):
        self.setAxisScale(Qwt.QwtPlot.yLeft,
                          self._y_limits[0],
                          self._y_limits[1])
        self.setAxisScale(Qwt.QwtPlot.xBottom,
                          self._x_limits[0],
                          self._x_limits[1])

        self._canvas_display_height = self._y_limits[1] - self._y_limits[0]
        self._canvas_display_width  = self._x_limits[1] - self._x_limits[0]
        self.redraw()

    def rescale_axis_x(self, delta__x):
        """
        add delta_x to the length of the x axis
        """
        x_width = self._x_limits[1] - self._x_limits[0]
        x_width += delta__x
        self._x_limits[1] = x_width + self._x_limits[0]
        self.rescale()

    def scale_axis_y(self, max_value):
        """
        set the y axis height to max_value, about the current center
        """
        canvas_display_height = max_value
        canvas_offset_y = (self._y_limits[1] + self._y_limits[0])/2.0
        y_lower_limit = canvas_offset_y - (canvas_display_height / 2)
        y_upper_limit = canvas_offset_y + (canvas_display_height / 2)
        self._y_limits = [y_lower_limit, y_upper_limit]
        self.rescale()

    def move_canvas(self, delta_x, delta_y):
        """
        move the canvas by delta_x and delta_y in SCREEN COORDINATES
        """
        canvas_offset_x = delta_x * self._canvas_display_width / self.canvas().width()
        canvas_offset_y = delta_y * self._canvas_display_height / self.canvas().height()
        self._x_limits = [ l + canvas_offset_x for l in self._x_limits]
        self._y_limits = [ l + canvas_offset_y for l in self._y_limits]
        self.rescale()

    def mousePressEvent(self, event):
        self._last_canvas_x = event.x() - self.canvas().x()
        self._last_canvas_y = event.y() - self.canvas().y()
        self._pressed_canvas_y = event.y() - self.canvas().y()

    def mouseMoveEvent(self, event):
        canvas_x = event.x() - self.canvas().x()
        canvas_y = event.y() - self.canvas().y()
        if event.buttons() & Qt.MiddleButton:  # middle button moves the canvas
            delta_x = self._last_canvas_x - canvas_x
            delta_y = canvas_y - self._last_canvas_y
            self.move_canvas(delta_x, delta_y)
        elif event.buttons() & Qt.RightButton:   # right button zooms
            zoom_factor = max(-0.6, min(0.6, (self._last_canvas_y - canvas_y) / 20.0 / 2.0))
            delta_y = (self.canvas().height() / 2.0) - self._pressed_canvas_y
            self.move_canvas(0, zoom_factor * delta_y * 1.0225)
            self.scale_axis_y(max(0.005, self._canvas_display_height - (zoom_factor * self._canvas_display_height)))
            self.rescale_axis_x(self._last_canvas_x - canvas_x)
        self._last_canvas_x = canvas_x
        self._last_canvas_y = canvas_y

    def wheelEvent(self, event):  # mouse wheel zooms the y-axis
        # y position of pointer in graph coordinates
        canvas_y = event.y() - self.canvas().y()

        zoom_factor = max(-0.6, min(0.6, (event.delta() / 120) / 6.0))
        delta_y = (self.canvas().height() / 2.0) - canvas_y
        self.move_canvas(0, zoom_factor * delta_y * 1.0225)

        self.scale_axis_y(max(0.0005, self._canvas_display_height - zoom_factor * self._canvas_display_height))
        self.limits_changed.emit()


    def vline(self, x, color):
        qWarning("QwtDataPlot.vline is not implemented yet")

    def set_xlim(self, limits):
        self.setAxisScale(Qwt.QwtPlot.xBottom, limits[0], limits[1])
        self._x_limits = limits

    def set_ylim(self, limits):
        self.setAxisScale(Qwt.QwtPlot.yLeft, limits[0], limits[1])
        self._y_limits = limits

    def get_xlim(self):
        return self._x_limits

    def get_ylim(self):
        return self._y_limits


if __name__ == '__main__':
    from python_qt_binding.QtGui import QApplication

    app = QApplication(sys.argv)
    plot = QwtDataPlot()
    plot.resize(700, 500)
    plot.show()
    plot.add_curve(0, '(x/500)^2')
    plot.add_curve(1, 'sin(x / 20) * 500')
    for i in range(plot._num_value_saved):
        plot.update_value(0, (i / 500.0) * (i / 5.0))
        plot.update_value(1, math.sin(i / 20.0) * 500)

    sys.exit(app.exec_())
