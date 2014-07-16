import inspect
import Utils, BlenderUtils, ShapekeyStore
import bpy
import rospy
from . import StateOutputBase

class BlenderGetterFactory:

  @staticmethod
  def bonerot(bindentry):
    args = bindentry["bonerot"].split(":")
    return lambda: BlenderUtils.get_bones_rotation_rad(*args)

  @staticmethod
  def shkey_mesh(bindentry):
    mesh = bindentry["shkey_mesh"]
    def func():
      # Dictionary {shapekey: key_block object}.
      key_blocks = (
        bpy.data.meshes[mesh].shape_keys.key_blocks
      )
      # List of shapekey coefficients.
      coeffs = [
        key_blocks[shapekey].value
        for shapekey in ShapekeyStore.getIter()
      ]
      return coeffs
    return func

class ParametersOut(StateOutputBase):

  def build_processor(self, bindentry):
    """
    Returns a processor function, which will, on the given msg, set the
    parameter specified by 'keychain' to the value from blender data
    returned by 'blendergetter'.
    """
    try:
      blendergetter_builder = [
        getattr(BlenderGetterFactory, key) for key in bindentry
        if hasattr(BlenderGetterFactory, key) and key[0] != "_"
      ][0]
    except Exception:
      raise NameError("Unable to bind %s" % bindentry)

    keychain = Utils.DictKeyChain(bindentry["varpath"].split(":"))
    blendergetter = blendergetter_builder(bindentry)
    def processor(msg):
      keychain.set_on(msg, blendergetter())
    return processor

  def transmit(self):
    msg = self.build_msg()
    self.pub.publish(msg)

  def build_msg(self):
    msg = self.msgtype()
    for processor in self.processors:
      processor(msg)
    return msg

  def __init__(self, confentry):
    self.msgtype = Utils.import_member(confentry["msg"])
    self.pub = rospy.Publisher(confentry["pubtopic"], self.msgtype, queue_size=2)

    self.processors = []
    for singlebind in confentry["binding"]:
      self.processors.append(self.build_processor(singlebind))