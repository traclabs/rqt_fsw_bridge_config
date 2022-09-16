#!/usr/bin/env python3

from __future__ import division
import os
import ast
import ntpath
import yaml

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Slot, Qt
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QTableWidgetItem
from PyQt5 import QtCore, QtWidgets

import rclpy
from ament_index_python import get_resource

from fsw_ros2_bridge_msgs.srv import GetMessageInfo, SetMessageInfo, GetPluginInfo

from .config_info import ConfigInfo
from .confirm_dialog import ConfirmDialog


class BridgeConfigWidget(QWidget):

    _column_names = ['key', 'val']

    def __init__(self, node, plugin):
        super(BridgeConfigWidget, self).__init__()

        self._node = node
        self._plugin = plugin
        self._logger = self._node.get_logger().get_child('rqt_fsw_bridge_config.BridgeConfigWidget')
        self._connected_to_bridge = False
        self._plugin_pkg_name = ""
        self._plugin_name = ""
        self._config_file_map = {}

        # set up UI
        _, package_path = get_resource('packages', 'rqt_fsw_bridge_config')
        ui_file = os.path.join(package_path, 'share', 'rqt_fsw_bridge_config',
                               'resource', 'BridgeConfigWidget.ui')
        loadUi(ui_file, self)
        self.setup_ui_connections()

        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        # bridge info
        self._plugin_info = None
        self._config_info = ConfigInfo(self._node)

        # ros clients
        self.plugin_info_client =\
            self._node.create_client(GetPluginInfo, '/fsw_ros2_bridge/get_plugin_info')

        # connection timer
        self._timer_wait_for_bridge = QTimer(self)
        self._timer_wait_for_bridge.timeout.connect(self.wait_for_plugin)

    def start(self):
        self._timer_wait_for_bridge.start(1000)

    def shutdown_plugin(self):
        self._timer_wait_for_bridge.stop()

    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.config_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.config_tree_widget.header().restoreState(header_state):
                self._logger.warn('rqt_fsw_bridge_config: Failed to restore header state.')

    def setup_ui_connections(self):
        self.config_tree_widget.itemClicked.connect(self._on_msg_item_clicked)
        self.save_config_button.clicked.connect(self.save_config_pressed)
        self.reload_config_button.clicked.connect(self.reload_config_pressed)

    def send_plugin_info_request(self):
        req = GetPluginInfo.Request()
        future = self.plugin_info_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    @Slot()
    def wait_for_plugin(self):
        if (not self._connected_to_bridge):
            self._node.get_logger().info("Trying to connect to FSW bridge...")
            if self.plugin_info_client.wait_for_service(timeout_sec=1.0):
                if self._plugin_info is None:
                    self._plugin_info = self.send_plugin_info_request()
                    self._plugin_name = self._plugin_info.plugin_name
                    self._plugin_pkg_name = self._plugin_name.split('.')[0]
                    self.parse_config_files(self._plugin_info.config_files)

                    if self._config_file_map:
                        self.parse_config_file(str(self.config_file_combo_box.currentText()))

                    self._node.get_logger().info("setting plugin: " + self._plugin_name)
                    self._node.get_logger().info("setting plugin pkg: " + self._plugin_pkg_name)

                    self.plugin_pkg_label.setText(self._plugin_pkg_name)
                    self.plugin_name_label.setText(self._plugin_name)
                    self._connected_to_bridge = True

    # def dictionary_check(self, input):
    #     for key,value in input.items():
    #         if isinstance(value, dict):
    #             self.dictionary_check(value)
    #             self._node.get_logger().info(key)
    #             item = QTreeWidgetItem(key)
    #         else:
    #             self._node.get_logger().info(key + ": " + str(value))

    def build_config_tree(self, data, par=None):
        items = []
        item = None

        # data = ast.literal_eval(data)
        for key in data.keys():
            if par is None:
                item = QTreeWidgetItem([key])
            else:
                item = QTreeWidgetItem([key, par])

            # item.setText(self._column_index['val'], data[key])
            item.setText(self._column_index['key'], key)


            if isinstance(data[key], dict):
                # child = self.build_config_tree(data[key], key)
                # item.addChild(child)
                children = self.build_config_tree(data[key], key)
                item.addChildren(children)
                item.setText(self._column_index['val'], "")
                self._node.get_logger().info("found child: " + key)
            else:
                self._node.get_logger().info("found leaf: " + key + " : " + data[key])
                item.setText(self._column_index['val'], data[key])
                item.setFlags(Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsSelectable)

            items.append(item)

        if par is None:
            self.config_tree_widget.insertTopLevelItems(0, items)

        return items


    def parse_config_file(self, config_file):
        fname = self._config_file_map[config_file]
        try:
            with open(fname, "r") as infile:
                config_dict = yaml.safe_load(infile)
        except FileNotFoundError:
            self._node.get_logger().error("Couldnt open " + fname + " for editing")
            return
        self._node.get_logger().info("config dict:\n" + str(config_dict))

        self.build_config_tree(config_dict)
        self.config_tree_widget.resizeColumnToContents(0)

        # items = []
        # for key, values in data.items():
        #     item = QTreeWidgetItem([key])
        #     for value in values:
        #         ext = value.split(".")[-1].upper()
        #         child = QTreeWidgetItem([value, ext])
        #         item.addChild(child)
        #     items.append(item)

        # self.config_tree_widget.clear()
        # self.config_tree_widget.insertTopLevelItems(0, items)
        # self.config_tree_widget.expandAll()

    # def build_msg_struct_tree(self, data, par=None):
    #     items = []
    #     item = None

    #     data = ast.literal_eval(data)
    #     for key in data.keys():
    #         if par is None:
    #             item = QTreeWidgetItem([key])
    #         else:
    #             item = QTreeWidgetItem([key, par])

    #         item.setText(self._column_index['structure'], key)
    #         item.setText(self._column_index['type'], data[key])

    #         if not self.is_primitive(data[key]):
    #             [pkg_name, msg_type] = data[key].split("/")
    #             m = self._config_info.get_message_struct(msg_type)
    #             child = self.build_msg_struct_tree(m, data[key])
    #             item.addChild(child)
    #         items.append(item)

    #     if par is None:
    #         self.msg_struct_tree.insertTopLevelItems(0, items)

    #     return item

    def parse_config_files(self, config_files):
        self.config_file_combo_box.clear()
        for f in config_files:
            bn = ntpath.basename(f)
            self._config_file_map[bn] = f
            self.config_file_combo_box.addItem(bn)
        self._node.get_logger().info('config file map: -- ' + str(self._config_file_map))

    @QtCore.pyqtSlot(QtWidgets.QTreeWidgetItem, int)
    def _on_msg_item_clicked(self, it, col):
        return

    @QtCore.pyqtSlot()
    def save_config_pressed(self):
        return

    @QtCore.pyqtSlot()
    def reload_config_pressed(self):
        return
