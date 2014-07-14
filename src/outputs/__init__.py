import Utils
import inspect
import sys

__doc__ = """
'outputs' is a package that holds classes and their instances that can, on
command, send parts of the current Blender state to ROS network. You can
access output.yaml config entries as members of outputs package.

E.g. The instance outputs.neck_euler of class ParametersOut will, when
transmit() method is called, send a message containing the current neck joint
positions in Blender space to basic_head_api as yaw, pitch and roll rotations.

See default config: config/outputs.yaml for available instances.
"""

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
    raise NameError("Couldn't find class %s" % member_name)

def _build_single(confentry):
  return _get_class(confentry["class"])(confentry)

def _store_instance(name, instance):
  global instances
  instances[name] = instance
  setattr(sys.modules[__name__], name, instance)

def initialize(fullconfig):
  global instances
  instances = {}
  for confentry in fullconfig:
    if confentry["name"][0] == "_":
      raise NameError("Output name %s can't start with _" % confentry["name"])
    if confentry["name"] in orig_members:
      raise NameError("Output name %s is reserved" % confentry["name"])
    _store_instance(confentry["name"], _build_single(confentry))

def get_instance(name):
  return instances[name]

# Reserve member names
orig_members = [member[0] for member in inspect.getmembers(sys.modules[__name__])]