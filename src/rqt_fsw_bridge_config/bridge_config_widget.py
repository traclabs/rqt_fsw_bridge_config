#!/usr/bin/env python3

from __future__ import division
import os
import ast
import ntpath
import yaml
import collections

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Slot, Qt
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QTableWidgetItem, QComboBox
from PyQt5 import QtCore, QtWidgets

import rclpy
from ament_index_python import get_resource
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
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
        self._plugin_node_name = ""
        self._config_file_map = {}
        self._config_dict = {}

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

        # need to set these on connection when we know the name of the plugin
        self.plugin_params_client = None
        self.plugin_param_client = None

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
        self.save_config_button.clicked.connect(self.save_config_pressed)
        self.reload_config_button.clicked.connect(self.reload_config_pressed)
        self.send_config_button.clicked.connect(self.send_config_pressed)
        self.config_tree_widget.itemDoubleClicked.connect(self.on_config_item_clicked)
        self.config_file_combo_box.currentIndexChanged.connect(self.config_file_selected)
        self.config_tree_widget.itemChanged.connect(self.on_config_item_changed)
        self.update_param_checkbox.stateChanged.connect(self.update_checkbox_changed)

    def send_plugin_info_request(self):
        req = GetPluginInfo.Request()
        future = self.plugin_info_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    def send_parameter_set_request(self, pname, pvalue):
        self._logger.warn('trying to set \'' + pname + '\' as: ' + str(pvalue))
        req = SetParametersAtomically.Request()
        param = Parameter()
        param.name = pname
        param.value = self.parse_param_val(pvalue)
        req.parameters.append(param)
        future = self.plugin_param_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    def send_parameters_set_request(self):
        req = SetParameters.Request()
        self._node.get_logger().info('config dict: ' + str(self._config_dict[self._plugin_node_name]["ros__parameters"]))
        flat_dict = self.flatten(self._config_dict[self._plugin_node_name]["ros__parameters"])
        self._node.get_logger().info('flat_dict: ' + str(flat_dict))
        for key, val in flat_dict.items():
            param = Parameter()
            param.name = key
            param.value = self.parse_param_val(val)
            req.parameters.append(param)
        future = self.plugin_params_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()


    def parse_param_val(self, value):
        pval = ParameterValue()
        try:
            int_param = int(value)
            self._logger.info("parse_param_val() -- could be an int: " + value)
            pval.integer_value = int_param
            pval.type = ParameterType.PARAMETER_INTEGER
            return pval
        except ValueError:
            pass

        try:
            float_param = float(value)
            self._logger.info("parse_param_val() -- could be a float: " + value)
            pval.double_value = float_param
            pval.type = ParameterType.PARAMETER_DOUBLE
            return pval
        except ValueError:
            pass

        try:
            if value.lower() == "true":
                self._logger.info("parse_param_val() -- could be a bool: " + value)
                pval.bool_value = True
                pval.type = ParameterType.PARAMETER_BOOL
                return pval
            elif value.lower() == "false":
                pval.bool_value = False
                self._logger.info("parse_param_val() -- could be a bool: " + value)
                pval.type = ParameterType.PARAMETER_BOOL
                return pval
        except ValueError:
            pass

        str_param = value
        self._logger.info("parse_param_val() -- probably a string: " + value)
        pval.string_value = str_param
        pval.type = ParameterType.PARAMETER_STRING
        return pval

    @Slot()
    def wait_for_plugin(self):
        if (not self._connected_to_bridge):
            self._node.get_logger().info("Trying to connect to FSW bridge...")
            if self.plugin_info_client.wait_for_service(timeout_sec=1.0):
                if self._plugin_info is None:
                    self._plugin_info = self.send_plugin_info_request()
                    self._plugin_node_name = self._plugin_info.node_name
                    self._plugin_name = self._plugin_info.plugin_name
                    self._plugin_pkg_name = self._plugin_name.split('.')[0]
                    self.parse_config_files(self._plugin_info.config_files)
                    self.setup_parameter_clients()

                    if self._config_file_map:
                        self.parse_config_file(str(self.config_file_combo_box.currentText()))

                    self._node.get_logger().info("setting plugin: " + self._plugin_name)
                    self._node.get_logger().info("setting plugin pkg: " + self._plugin_pkg_name)

                    self.plugin_pkg_label.setText(self._plugin_pkg_name)
                    self.plugin_name_label.setText(self._plugin_name)

                    self._connected_to_bridge = True

    def setup_parameter_clients(self):
        if self.plugin_params_client is None:
            n = '/' + self._plugin_node_name + '/set_parameters'
            self._node.get_logger().info("setting up params client: " + n)
            self.plugin_params_client = self._node.create_client(SetParameters, n)

        if self.plugin_param_client is None:
            n = '/' + self._plugin_node_name + '/set_parameters_atomically'
            self._node.get_logger().info("setting up param client: " + n)
            self.plugin_param_client = self._node.create_client(SetParametersAtomically, n)

    def build_config_tree(self, item, value):
        item.setExpanded(True)
        if type(value) is dict:
            for key, val in sorted(value.items()):
                child = QTreeWidgetItem()
                child.setText(self._column_index['key'], str(key))
                item.addChild(child)
                self.build_config_tree(child, val)
        else:
            item.setText(self._column_index['val'], str(value))

    def parse_config_file(self, config_file):
        self._node.get_logger().info("parsing config file...")
        fname = self._config_file_map[config_file]
        try:
            with open(fname, "r") as infile:
                self._config_dict = yaml.safe_load(infile)
        except FileNotFoundError:
            self._node.get_logger().error("Couldnt open " + fname + " for editing")
            return
        self.config_tree_widget.clear()
        self.build_config_tree(self.config_tree_widget.invisibleRootItem(), self._config_dict)
        self._node.get_logger().info("updated dict: " + str(self._config_dict   ))

    def parse_config_files(self, config_files):
        self.config_file_combo_box.clear()
        for f in config_files:
            bn = ntpath.basename(f)
            self._config_file_map[bn] = f
            self.config_file_combo_box.addItem(bn)
        self._node.get_logger().info('config file map: -- ' + str(self._config_file_map))

    def roll_out_tree_as_list(self, it, tl):
        tl.append(it.text(0))
        if it.parent():
            pit = self.roll_out_tree_as_list(it.parent(), tl) 

    def set_config_data(self, tl, val):
        if not tl:
            return
        if len(tl) == 1:
            self._config_dict[tl[0]] = val
        elif len(tl) == 2:
            self._config_dict[tl[0]][tl[1]] = val
        elif len(tl) == 3:
            self._config_dict[tl[0]][tl[1]][tl[2]] = val
        elif len(tl) == 4:
            self._config_dict[tl[0]][tl[1]][tl[2]][tl[3]] = val
        elif len(tl) == 5:
            self._config_dict[tl[0]][tl[1]][tl[2]][tl[3]][tl[4]] = val
        elif len(tl) == 6:
            self._config_dict[tl[0]][tl[1]][tl[2]][tl[3]][tl[4]][tl[5]] = val
        elif len(tl) == 7:
            self._config_dict[tl[0]][tl[1]][tl[2]][tl[3]][tl[4]][tl[5]][tl[6]] = val
        else:
            self._node.get_logger().error('can not set parameters at depth ' + len(tl))
            self._node.get_logger().error('maybe cool it with the complexity')

    def flatten(self, d, parent_key='', sep='.'):
        items = []
        for k, v in d.items():
            new_key = parent_key + sep + k if parent_key else k
            if isinstance(v, collections.MutableMapping):
                items.extend(self.flatten(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def set_parameter(self, tl, val):
        if len(tl) < 3:
            self._node.get_logger().error('problem setting ros2 param from: -- ' + str(tl))
            return
        if tl[1] != "ros__parameters":
            self._node.get_logger().error('problem parsing ros_parameters from: -- ' + str(tl))
            return

        pname = tl[2]
        for idx in range(3, len(tl)):
            pname = pname + "." + tl[idx]
            self._node.get_logger().info('final param name: -- ' + pname)
            self.send_parameter_set_request(pname, val)

    @QtCore.pyqtSlot(QtWidgets.QTreeWidgetItem, int)
    def on_config_item_clicked(self, it, col):
        if col == 0:
            it.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        else:
            if str(it.text(1)):
                it.setFlags(Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsSelectable)

    @QtCore.pyqtSlot(QtWidgets.QTreeWidgetItem, int)
    def on_config_item_changed(self, it, col):
        if col == 1:
            tl = []
            self.roll_out_tree_as_list(it, tl)
            tl.reverse()
            self.set_config_data(tl, str(it.text(1)))
            if self.update_param_checkbox.isChecked():
                self.set_parameter(tl, it.text(1))

    @QtCore.pyqtSlot(int)
    def config_file_selected(self, idx):
        self.parse_config_file(str(self.config_file_combo_box.currentText()))

    @QtCore.pyqtSlot()
    def save_config_pressed(self):
        config_file = self.config_file_combo_box.currentText()
        fname = self._config_file_map[config_file]
        self._node.get_logger().info('saving to: ' + fname)
        try:
            with open(fname, "w") as outfile:
                r = yaml.dump(self._config_dict, outfile, default_flow_style=False)
        except FileNotFoundError:
            pass

    @QtCore.pyqtSlot()
    def reload_config_pressed(self):
        self.parse_config_file(str(self.config_file_combo_box.currentText()))

    @QtCore.pyqtSlot()
    def send_config_pressed(self):
        self.send_parameters_set_request()

    @QtCore.pyqtSlot(int)
    def update_checkbox_changed(self, s):
        if s == 2:
            self._node.get_logger().info("Hey! we're sending everything when you check this!")
            self.send_parameters_set_request()
        



