import Utils

__doc__ = """
'outputs' is a package that holds classes and their instances that can, on
command, send parts of the current Blender state to ROS network.

E.g. The instance 'neck_euler' of class ParametersOut will, when transmit()
method is called, send a message containing the current neck joint positions
in Blender space to basic_head_api as yaw, pitch and roll rotations.

See default config: config/outputs.yaml for available instances.
"""

def _get_class(name):
  module_name = ".".join([__name__, name])
  member_name = ".".join([__name__] + [name]*2)
  try:
    return Utils.import_member(member_name)
  except ImportError:
    raise NameError("Couldn't find module %s" % module_name)
  except AttributeError:
    raise NameError("Couldn't find class %s" % member_name)

def _build_single(confentry):
  return _get_class(confentry["class"])(confentry)

def initialize(fullconfig):
  global instances
  instances = {
    confentry["name"]: _build_single(confentry) for confentry in fullconfig
  }

def get_instance(name):
  return instances[name]