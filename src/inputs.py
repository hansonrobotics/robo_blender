#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from ros_nmpt_saliency.msg import targets
from ros_faceshift.msg import fsMsgTrackingState

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
      location = Vector(binding["offset"]) + point * binding["scale"]
      bpy.data.objects[binding["name"]].location = location

  def _confirm_object(self):
    """Create object with name specified in config if it's not there."""
    if not bpy.data.objects.has_key(binding["name"]):
      bpy.ops.mesh.primitive_cube_add(radius=0.024)
      bpy.context.selected_objects[0].name = binding["name"]

  def __init__(self, confentry):
    self.confentry = confentry


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

    def handle_source(self, msg):
      pass

    def __init__(self, confentry):
      rospy.Subscriber(confentry["sourcetopic"], fsMsgTrackingState, self.handle_source)


def build_single(confentry):
  """
  Build an instance of a class from the Shelf with the name specified by
  confentry["classname"]
  """
  clazz = getattr(Shelf, confentry["inputclass"])
  return clazz(confentry)