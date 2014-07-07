#!/usr/bin/env python3
import sys
import BlenderUtils
import Utils

__doc__ = """
outputs.py is a module that holds classes and their instances that can, on
command, send parts of the current Blender state to ROS network.

E.g. The instance 'neck_euler' of class ParametersOut will, when transmit()
method is called, send a message containing the current neck joint positions
in Blender space to basic_head_api as yaw, pitch and roll rotations.

See default config: config/outputs.yaml for available instances.
"""

class Shelf:

  class ParametersOut:

    def build_msg_setter(self, singlebind_conf):
      """
      Returns a processor function, which will, on the given msg, set the
      parameter specified by 'keychain' to the value from blender data
      returned by 'blendergetter'.
      """
      keychain = Utils.DictKeyChain(singlebind_conf["varpath"].split(":"))

      if singlebind_conf.has_key("bonerot"):
        def blendergetter():
          return BlenderUtils.get_bones_rotation_rad(
            *singlebind_conf["bonerot"].split(":")
          )

      def processor(msg):
        keychain.set_on(msg, blendergetter())
      return processor

    def transmit(self):
      msg = self.msgtype()
      for processor in self.processors:
        processor(msg)
      self.pub.publish(msg)

    def __init__(self, confentry):
      __import__(confentry["msg"])
      self.msgtype = sys.modules[confentry["msg"]]
      self.pub = rospy.Publisher(confentry["pubtopic"], self.msgtype)

      self.processors = []
      for singlebind in confentry["binding"]:
        self.processors.append(build_msg_setter(singlebind))


def build_single(confentry):
  clazz = getattr(Shelf, confentry["inputclass"])
  return clazz(confentry)

def initialize(fullconfig):
  self.instances = {
    confentry["name"]: build_single(confentry) for confentry in fullconfig
  }

def get_instance(name):
  return self.instances[name]
