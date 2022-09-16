#!/usr/bin/env python3

from fsw_ros2_bridge_msgs.msg import MessageInfo


class ConfigInfo:
    def __init__(self, node):
        self._node = node
        self._message_info_list = []
        self._msg_dict = {"commands": [], "telemetry": [], "helper": []}
        self._msg_struct = {}
        self._msg_info = {}
        self._msg_pkg = ""
        self._plugin_pkg = ""

    def init(self, plugin_pkg, msg_pkg, message_info_list):
        self._plugin_pkg = plugin_pkg
        self._msg_pkg = msg_pkg
        self._message_info_list = message_info_list
        return self.set_message_info()

    def set_message_info(self):
        for m in self._message_info_list:
            if m.msg_type is MessageInfo.TELEMETRY:
                self._msg_dict["telemetry"].append(m.msg_name)
            elif m.msg_type is MessageInfo.COMMAND:
                self._msg_dict["commands"].append(m.msg_name)
            else:
                self._msg_dict["helper"].append(m.msg_name)
            self._msg_struct[m.msg_name] = m.json
            self._msg_info[m.msg_name] = m.info

        n_cmd = len(self._msg_dict["commands"])
        n_tlm = len(self._msg_dict["telemetry"])
        n_hlp = len(self._msg_dict["helper"])

        self._node.get_logger().info("Message Config:")
        self._node.get_logger().info("  found " + str(n_cmd) + " command msgs")
        self._node.get_logger().info("  found " + str(n_tlm) + " telemetry msgs")
        self._node.get_logger().info("  found " + str(n_hlp) + " helper msgs")
        return self._msg_dict

    def get_message_type(self, msg_name):
        if msg_name in self._msg_dict["telemetry"]:
            return "TELEMETRY"
        if msg_name in self._msg_dict["commands"]:
            return "COMMAND"
        if msg_name in self._msg_dict["helper"]:
            return "HELPER"
        return "UNKNOWN"

    def get_message_struct(self, msg_name):
        if msg_name in self._msg_struct.keys():
            return self._msg_struct[msg_name]
        return {}

    def get_message_info(self, msg_name):
        if msg_name in self._msg_struct.keys():
            return self._msg_info[msg_name]
        return ""

    def save_message_info(self, msg_name, info):
        self._msg_info[msg_name] = info
