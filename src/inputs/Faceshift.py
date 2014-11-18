import rospy
import bpy
from .bases import InputBase
from pau2motors.msg import pau
import ShapekeyStore

class Faceshift(InputBase):
  """ 
  Shapekey application on mesh - tested and working. Singlebinds - not tested.

  TODO: Figure out another approach instead of setting the shapekeys
  directly to the primary mesh (this way no chance is given for the
  controller to select and combine the faceshift input). Maybe we should map
  faceshift input to another mesh placed next to the primary one.
  """

  def build_set_bone_position(self, singlebind_dict):
    keychain = Utils.DictKeyChain(singlebind_dict["varpath"].split(":"))
    bone_parent, bone, axis = singlebind_dict["bonerot"].split(":")
    rot_index = {"x": 0, "y": 1, "z": 2}[axis]

    def processor(msg):
      bpy.data.objects[bone_parent].pose.bones[bone].rotation_mode = 'XYZ'
      rot = bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler
      rot[rot_index] = keychain.get_from(msg)
      bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler = rot

    return processor

  def handle_source(self, msg):
    if not self.confentry["enabled"]:
      return

    # Apply shapekeys
    meshname = self.confentry["binding"]["meshname"]
    for i in range(len(msg.m_coeffs)):
      shapename = ShapekeyStore.getString(i)
      bpy.data.meshes[meshname].shape_keys.key_blocks[shapename].value = msg.m_coeffs[i]

    # Process single binds
    for processor in self.singlebind_processors:
      processor(msg)
    

  def __init__(self, confentry):
    self.confentry = confentry

    # Build processors for entries in confentry["binding"]["singles"]
    self.singlebind_processors = []
    for singlebind in self.confentry["binding"]["singles"]:
      self.singlebind_processors.append(
        self.build_set_bone_position(singlebind)
      )

    rospy.Subscriber(confentry["sourcetopic"], pau, self._pend_msg(self.handle_source))