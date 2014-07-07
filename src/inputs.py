#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from ros_nmpt_saliency.msg import targets
from ros_faceshift.msg import fsMsgTrackingState
import ShapekeyStore
import Utils

__doc__ = """
inputs.py is a module that holds classes and their instances that are
responsible for transferring input data (like from sensors) to Blender space.

E.g. PiVision and Saliency classes control small cubes to represent the
point received.

Default config: config/inputs.yaml
"""

class _ObjectInBlender:
  """A base class that can be used to move an object in Blender space."""

  def set_object_location(self, point):
    self._confirm_object()
    # Go straight to _set_object_location() next time.
    self.set_object_location = self._set_object_location
    self.set_object_location(point)

  def _set_object_location(self, point):
    if confentry["enabled"]:
      binding = confentry["binding"]["objectpos"]
      self.location = Vector(binding["offset"]) + point * binding["scale"]
      bpy.data.objects[binding["name"]].location = self.location

  def _confirm_object(self):
    """Create object with name specified in config if it's not there."""
    if not bpy.data.objects.has_key(binding["name"]):
      bpy.ops.mesh.primitive_cube_add(radius=0.024)
      bpy.context.selected_objects[0].name = binding["name"]

  def __init__(self, confentry):
    self.confentry = confentry
    self.location = Vector()


class Shelf:

  class PiVision(_ObjectInBlender):

    def roi2point(self, msg):
      """
      Returns a normalized point at the center of the given RegionOfInterest
      message.
      """
      p = Vector([0,0,0])
      if self.camerainfo.width > 0:
        p.x =  0.5 - (msg.x_offset+(msg.width/2.0))/self.camerainfo.width
      if self.camerainfo.height > 0:
        p.z =  0.5 - (msg.y_offset+(msg.height/2.0))/self.camerainfo.height
      return p

    def handle_source(self, msg):
      point = self.roi2point(msg)
      self.set_object_location(point)
      pass
    
    def __init__(self, confentry):
      rospy.Subscriber(confentry["sourcetopic"], RegionOfInterest, self.handle_source)

      def handle_camerainfo(msg):
        self.camerainfo = msg
        rospy.loginfo("msg")
        ci_sub.unregister()
      ci_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, handle_camerainfo)

      super(PiVision, self).__init__(confentry)


  class NmptSaliency(_ObjectInBlender):

    def handle_source(self, msg):
      point = msg.positions[0]
      self.set_object_location(point)
    
    def __init__(self, confentry):
      rospy.Subscriber(confentry["sourcetopic"], targets, self.handle_source)
      super(NmptSaliency, self).__init__(confentry)


  class Faceshift:

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
      if not confentry["enabled"]:
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

      rospy.Subscriber(confentry["sourcetopic"], fsMsgTrackingState, self.handle_source)


def build_single(confentry):
  """
  Build an instance of a class from the Shelf with the name specified by
  confentry["inputclass"]
  """
  clazz = getattr(Shelf, confentry["inputclass"])
  return clazz(confentry)

def initialize(fullconfig):
  self.instances = {
    confentry["inputclass"]: build_single(confentry) for confentry in fullconfig
  }

def get_instance(classname):
  return self.instances[classname]
