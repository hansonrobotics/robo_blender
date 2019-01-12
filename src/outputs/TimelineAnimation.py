#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import rospy
import Utils

class FrameMap:
  """ Represents the position of an animation in the timeline. """

  def __iter__(self):
    return iter(range(self.frame_start, self.frame_end))

  def set_duration(self, val):
    self.duration = val

  def get_frame_duration(self):
    return float(self.frame_start - self.frame_end)/self.duration

  @classmethod
  def from_string(cls, str):
    """ Alternative constructor method. """
    # 'cls' hold the FrameMap class
    # Asterix below means the list str.split(":") will be expanded into
    # arguments frame_start, frame_end, min_duration
    return cls(*str.split(":"))

  def __init__(self, frame_start, frame_end, min_duration):
    self.frame_start = frame_start
    self.frame_end = frame_end
    self.min_duration = min_duration

class TimelineAnimation:
  """
  This output can build and send an animation out, if you give the
  animation's location (FrameMap instance).
  """

  def send(self, frame_map):
    """
    Call this from your controller.
    Will iterate throught frames, build trajectory messages and send them.
    """
    # __iter__ method in FrameMap will allow you to iterate like this:
    #
    #for frame in frame_map:
    #  print("Current frame: %s" % frame)
    raise NotImplementedError

  def __init__(self, confentry):
    # Motor config and categories can be saved here (outputs/anim_zeno.yaml)
    # until we find a better location for it.
    self.config = Utils.read_yaml("anim_zeno.yaml", __file__)
    raise NotImplementedError