import Utils
import inspect
import sys
import rospy

__doc__ = """
'outputs' is a package that holds classes and their instances that can, on
command, send parts of the current (or stored?) Blender state to ROS network.

Output instances can be reached by either:
outputs.store.<instancename> or outputs.get_instance(<instancename>),
where <instancename> can be neck_euler, eyes, etc.

E.g. The instance outputs.store.neck_euler of class ParametersOut will, when
transmit() method is called, send a message containing the current neck joint
positions in Blender space to basic_head_api as yaw, pitch and roll rotations.

See default config: config/outputs.yaml for available instances.
"""

class DummyOutput:

  def __getattr__(self, attrname):
    """ Return a dummy function that doesn't do anything. """
    rospy.logwarn("Output %s is disabled." % self.name)
    return lambda *args: None

  def __init__(self, name):
    self.name = name


class InstanceStore:
  """ Here is where instances of outputs are stored. """

  @staticmethod
  def _get_class(relative_path):
    #E.g. ParametersOut
    module_name = relative_path.split(".")[-1]
    #E.g. outputs.state.ParametersOut
    module_path = ".".join([__name__, relative_path])
    #E.g. outputs.state.ParametersOut.ParametersOut
    member_path = ".".join([__name__, relative_path, module_name])
    try:
      return Utils.import_member(member_path)
    except ImportError as err:
      if err.name == module_name:
        raise NameError("Couldn't find module %s" % module_path)
      else:
        raise
    except AttributeError:
      raise NameError("Couldn't load class %s" % member_path)

  @classmethod
  def _build_single(cls, confentry):
    if confentry["enabled"]:
      return cls._get_class(confentry["class"])(confentry)
    else:
      return DummyOutput(confentry["name"])

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
          raise NameError("Output name %s can't start with _" % confentry["name"])
        if confentry["name"] in self.orig_members:
          raise NameError("Output name %s is reserved" % confentry["name"])
        self._store_instance(confentry["name"], self._build_single(confentry))
      except:
        traceback.print_exc()
        rospy.logwarn("Couldn't load input %s" % confentry["name"])

def initialize(fullconfig):
  global store
  store = InstanceStore(fullconfig)

def get_instance(name):
  return store._instances[name]