#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Intelligent Robotics Core S.L.
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
#
# Author: Francisco Martin Rico - fmrico at gmail.com


from __future__ import division
import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget
from python_qt_binding.QtSvg import QSvgGenerator

import rclpy

from ros2_knowledge_graph_viewer.ros2_knowledge_graph2_impl import Ros2KnowledgeGraphImpl

from qt_dotgraph.dot_to_qt import DotToQtGenerator
# pydot requires some hacks
from qt_dotgraph.pydotfactory import PydotFactory
from rqt_gui_py.plugin import Plugin
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory

from .dotcode import \
    Ros2KnowledgeGraphDotcodeGenerator
from .interactive_graphics_view import InteractiveGraphicsView

from .ros2_knowledge_graph2_impl import Ros2KnowledgeGraphImpl

from PyQt5 import QtGui, QtCore

try:
    unicode
    # we're on python2, or the "unicode" function has already been defined elsewhere
except NameError:
    unicode = str
    # we're on python3


class Ros2KnowledgeGraph(Plugin):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(Ros2KnowledgeGraph, self).__init__(context)

        self._node = context.node
        self._logger = self._node.get_logger().get_child('ros2_knowledge_graph_viewer.ros2_knowledge_graph.Ros2KnowledgeGraph')
        self.setObjectName('Ros2KnowledgeGraph')

        self._ros2_knowledge_graph = Ros2KnowledgeGraphImpl()
        self._current_dotcode = None

        self._widget = QWidget()

        self.dotcode_factory = PydotFactory()
        self.dotcode_generator = Ros2KnowledgeGraphDotcodeGenerator()
        self.dot_to_qt = DotToQtGenerator()

        _, package_path = get_resource('packages', 'ros2_knowledge_graph_viewer')
        ui_file = os.path.join(package_path, 'share', 'ros2_knowledge_graph_viewer', 'resource', 'Ros2KnowledgeGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('Ros2KnowledgeGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        self._widget.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_as_svg_push_button.pressed.connect(self._save_svg)
        self._widget.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self._widget.save_as_image_push_button.pressed.connect(self._save_image)

        self._update_ros2_knowledge_graph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

        self._updateTimer = QtCore.QTimer()
        self._updateTimer.timeout.connect(self.do_update)
        self._updateTimer.start(10)

    def do_update(self):
        # print("Spinnning")
        rclpy.spin_once(self._ros2_knowledge_graph, timeout_sec=0.01)
        # print("Spinned")

        self._update_ros2_knowledge_graph()

        self._updateTimer = QtCore.QTimer()
        self._updateTimer.timeout.connect(self.do_update)
        self._updateTimer.start(10)


    def _update_ros2_knowledge_graph(self):
        self._refresh_ros2_knowledge_graph()

    def _refresh_ros2_knowledge_graph(self):
        # print("_refresh_ros2_knowledge_graph")
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        return self.dotcode_generator.generate_dotcode(
            ros2_knowledge_graphinst=self._ros2_knowledge_graph,
            dotcode_factory=self.dotcode_factory)

    def _update_graph_view(self, dotcode):
        # print("_update_graph_view")
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _redraw_graph_view(self):
        # print("_redraw_graph_view")
        self._scene.clear()

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=3,
                                                            same_label_siblings=True,
                                                            scene=self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        self._fit_in_view()

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr('Save as SVG'), 'rosgraph.svg',
            self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr('Save as image'), 'rosgraph.png',
            self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0)
                     .toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)
