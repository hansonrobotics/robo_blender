import rospy
from . import bases
from .bases.ObjectInBlender import ObjectInBlender
import sensor_msgs.msg
from mathutils import Vector

class RegionOfInterest(ObjectInBlender):

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
  
  def __init__(self, confentry):
    rospy.Subscriber(
    	confentry["sourcetopic"],
    	sensor_msgs.msg.RegionOfInterest,
    	self._pend_msg(self.handle_source)
    )

    def handle_camerainfo(msg):
      self.camerainfo = msg
      ci_sub.unregister()
    ci_sub = rospy.Subscriber(
      '/camera/camera_info',
      sensor_msgs.msg.CameraInfo,
      handle_camerainfo
    )

    super(RegionOfInterest, self).__init__(confentry)