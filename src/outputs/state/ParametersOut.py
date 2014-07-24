import inspect
import Utils, BlenderUtils, ShapekeyStore, ShapekeyStore2
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

    def get_coeffs(shkeystore, key_blocks):
      iterator = shkeystore.getIter()
      try:
        return [key_blocks[shapekey].value for shapekey in iterator]
      except KeyError as err:
        rospy.loginfo("%s not compatible: shapekey %s in mesh %s",
          shkeystore.__name__, err, mesh
        )
        return None

    # Pick which ShapekeyStore to use
    _shkeystore = [
      store for store in [ShapekeyStore, ShapekeyStore2]
      if get_coeffs(store, bpy.data.meshes[mesh].shape_keys.key_blocks)
    ][0]
    rospy.loginfo("Using %s for %s output", _shkeystore.__name__, mesh)

    def func():
      # Dictionary {shapekey: key_block object}.
      key_blocks = (
        bpy.data.meshes[mesh].shape_keys.key_blocks
      )
      # List of shapekey coefficients.
      return get_coeffs(_shkeystore, key_blocks)
    return func

  @staticmethod
  def childbones_name(bindentry):
    armature = bindentry["childbones_name"]
    return lambda: [bone for bone in bpy.data.objects[armature].pose.bones.keys()]

  @staticmethod
  def childbones_rot(bindentry):
    armature, axis = bindentry["childbones_rot"].split(":")
    return lambda: [
      BlenderUtils.get_bones_rotation_rad(armature, bone, axis)
      for bone in bpy.data.objects[armature].pose.bones.keys()
    ]

class ParametersOut(StateOutputBase):

  def build_processor(self, bindentry):
    """
    Returns a processor function, which will, on the given msg, set the
    parameter specified by 'keychain' to the value from blender data
    returned by 'blendergetter'.
    """

    # Find a function in BlenderGetterFactory whose name is in the bindentry.
    # I.e. if "bonerot" in bindentry:
    #        blendergetter_builder = BlenderGetterFactory.bonerot
    # etc.
    try:
      blendergetter_builder = [
        getattr(BlenderGetterFactory, key) for key in bindentry
        if hasattr(BlenderGetterFactory, key) and key[0] != "_"
      ][0]
    except Exception:
      raise NameError("Unable to bind %s" % bindentry)

    keychain = Utils.RosMsgKeyChain(bindentry["varpath"].split(":"))
    blendergetter = blendergetter_builder(bindentry)
    def processor(msg):
      keychain.hard_set_on(msg, blendergetter())
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
