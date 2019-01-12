#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import sys, Utils, inspect, traceback, rospy

__doc__ = """
inputs.py is a module that holds classes and their instances that are
responsible for transferring input data (like from sensors) to Blender space.

E.g. 'pivision' and 'nmpt_saliency' inputs are instances of RegionOfInterest, which
inherits from ObjectInBlender and control small cubes in Blender space
according to received ROS messages.

Input instances can be reached by either:
inputs.store.<instancename> or inputs.get_instance(<instancename>),
where <instancename> can be pivision, nmpt_saliency, etc.

See config/inputs.yaml for available input instances.

All input classes should inherit from inputs.bases.InputBase
"""

class PendingMsg:
  """
  A massage and its handler stored to be executed later in a Blender callback
  for thread safety.

  Takes the role of what in a previous implementation was called an "op" in
  the list "ops".
  """

  def execute(self):
    """ Should be called from within a Blender callback only. """
    self.handler(self.msg)

  def pend(self):
    """Call this to enqueue the message for the next Blender callback. """
    pend_msg(self)

  def __init__(self, msg, handler):
    self.msg = msg
    self.handler = handler

ops = []
def pend_msg(msg):
  global ops
  ops.append(msg)

def execute_pending():
  """ Should be called from within a Blender callback only. """
  global ops
  for op in ops:
    op.execute()
  ops = []



class InstanceStore:
  """ Here is where instances of outputs are stored. """

  @staticmethod
  def _get_class(name):
    module_name = ".".join([__name__, name])
    member_name = ".".join([__name__] + [name]*2)
    try:
      return Utils.import_member(member_name)
    except ImportError as err:
      if err.name == module_name:
        raise NameError("Couldn't find module %s" % module_name)
      else:
        raise
    except AttributeError:
      raise NameError("Couldn't load class %s" % member_name)

  @classmethod
  def _build_single(cls, confentry):
    return cls._get_class(confentry["class"])(confentry)

  def _store_instance(self, name, instance):
    self._instances[name] = instance
    setattr(self, name, instance)

  def __init__(self, fullconfig):
    # Reserve member names
    self.orig_members = [member[0] for member in inspect.getmembers(self)]

    # Build and store outputs out of the config
    self._instances = {}
    for confentry in fullconfig:
      try:
        if confentry["name"][0] == "_":
          raise NameError("Input name %s can't start with _" % confentry["name"])
        if confentry["name"] in self.orig_members:
          raise NameError("Input name %s is reserved" % confentry["name"])
        self._store_instance(confentry["name"], self._build_single(confentry))
      except:
        print("Exception during input '%s' load:" % confentry["name"])
        traceback.print_exc()
        rospy.logwarn("Couldn't load input '%s'" % confentry["name"])

def initialize(fullconfig):
  global store
  store = InstanceStore(fullconfig)

def get_instance(name):
  return store._instances[name]
