import rospy
from .bases import InputBase
from .bases.ObjectInBlender import ObjectInBlender
import sensor_msgs.msg, eva_behavior.msg
from mathutils import Vector
import copy
import posixpath

class Face:

  def __init__(self, face_id, confentry, get_camerainfo):
    face_id = int(face_id)

    #Generate name in binding config
    source_topic = posixpath.join(
      confentry["rosbind"]["namespace"],
      confentry["rosbind"]["sourcetopic"] % face_id
    )
    blenderbind = copy.deepcopy(confentry["blenderbind"])
    blenderbind["objectpos"]["name"] = source_topic

    #Nest an ObjectInBlender instance
    self.obj_in_blender = ObjectInBlender.from_binding(blenderbind)

    self.id = face_id
    self.get_camerainfo = get_camerainfo

    #Subscribe to generated topic
    self.topic = rospy.Subscriber(
      source_topic,
      sensor_msgs.msg.RegionOfInterest,
      self.obj_in_blender._pend_msg(self._handle_source)
    )

  def destroy(self):
    self.topic.unregister()
    self.obj_in_blender.destroy()

  def _roi2point(self, msg):
    """
    Returns a normalized point at the center of the given RegionOfInterest
    message.
    """
    camerainfo = self.get_camerainfo()
    p = Vector([0,0,0])
    if camerainfo.width > 0:
      p.x =  0.5 - (msg.x_offset+(msg.width/2.0))/camerainfo.width
    if camerainfo.height > 0:
      p.z =  0.5 - (msg.y_offset+(msg.height/2.0))/camerainfo.height
    return p

  def _handle_source(self, roi):
    point = self._roi2point(roi)
    self.obj_in_blender.set_object_location(point)

class NRegionsOfInterest(InputBase):
  """
  Dynamically subscribe and unregister from topics according to events sent to
  the 'eventtopic' (specified in config).
  """
  def __init__(self, confentry):
    #Make sure namespace parameter is there, even if it's empty
    if not "namespace" in confentry["rosbind"]:
      confentry["rosbind"]["namespace"] = ""

    self.confentry = confentry
    self.topic = rospy.Subscriber(
      posixpath.join(
        confentry["rosbind"]["namespace"],
        confentry["rosbind"]["eventtopic"]
      ),
      eva_behavior.msg.event,
      self._pend_msg(self._handle_event)
    )

    #A dictionary {face id: Face instance} of which pi_vision is currently tracking
    self.faces = {}

    def handle_camerainfo(msg):
      self.camerainfo = msg
      ci_sub.unregister()
    ci_sub = rospy.Subscriber(
      posixpath.join(confentry["rosbind"]["namespace"], "camera/camera_info"),
      sensor_msgs.msg.CameraInfo,
      handle_camerainfo
    )

  def get_camerainfo(self):
    return self.camerainfo

  def _handle_event(self, msg):
    if msg.event == "new_face":
      self._add_face(int(msg.param))
    elif msg.event == "exit":
      self._remove_face(int(msg.param))

  def _add_face(self, face_id):
    self.faces[face_id] = Face(
      face_id,
      self.confentry,
      self.get_camerainfo
    )

  def _remove_face(self, face_id):
    if face_id in self.faces:
      self.faces[face_id].destroy()
      del self.faces[face_id]
