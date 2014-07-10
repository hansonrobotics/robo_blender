#!/usr/bin/env python3
import sys
import rospy
import Utils, BlenderUtils, ShapekeyStore
import bpy

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

    def build_msg_setter(self, bindentry):
      """
      Returns a processor function, which will, on the given msg, set the
      parameter specified by 'keychain' to the value from blender data
      returned by 'blendergetter'.
      """
      keychain = Utils.DictKeyChain(bindentry["varpath"].split(":"))

      if "bonerot" in bindentry:
        def blendergetter():
          return BlenderUtils.get_bones_rotation_rad(
            *bindentry["bonerot"].split(":")
          )
      elif "shkey_mesh" in bindentry:
        def blendergetter():
          # Dictionary {shapekey: key_block object}.
          key_blocks = (
            bpy.data.meshes[bindentry["shkey_mesh"]].shape_keys.key_blocks
          )
          # List of shapekey coefficients.
          coeffs = [
            key_blocks[shapekey].value
            for shapekey in ShapekeyStore.getIter()
          ]
          return coeffs

      def processor(msg):
        keychain.set_on(msg, blendergetter())
      return processor

    def transmit(self):
      msg = self.msgtype()
      for processor in self.processors:
        processor(msg)
      self.pub.publish(msg)

    def __init__(self, confentry):
      self.msgtype = Utils.import_member(confentry["msg"])
      self.pub = rospy.Publisher(confentry["pubtopic"], self.msgtype, queue_size=2)

      self.processors = []
      for singlebind in confentry["binding"]:
        self.processors.append(self.build_msg_setter(singlebind))

  class CompositeOut:

    # The inside of this function will only be executed once.
    # self.outputs cannot be built inside __init__, because other outputs may
    # not have been built at that point.
    def transmit(self):
      self.outputs = [
        get_instance(output_name)
        for output_name in self.confentry["outputs"]
      ]

      # Set _transmit() to be executed when transmit() is called from this
      # point on
      self.transmit = self._transmit
      self.transmit()

    def _transmit(self):
      for output in self.outputs:
        output.transmit()

    def __init__(self, confentry):
      self.confentry = confentry


def build_single(confentry):
  clazz = getattr(Shelf, confentry["inputclass"])
  return clazz(confentry)

def initialize(fullconfig):
  global instances
  instances = {
    confentry["name"]: build_single(confentry) for confentry in fullconfig
  }

def get_instance(name):
  return instances[name]
