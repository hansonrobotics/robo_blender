import sys
import importlib
import inspect
import rospy
import Utils

__doc__ = """
--Description--
Package 'modes' holds modules that control the main flow of robot behavior.
When a module is set active, its step() method is called at the fps set in
config/config.yaml.

A different module can be enabled at any time by calling
the modes.enable() method or sending a ROS message to cmd_blendermode with a
different module name. Currently only one module can be active at a time.

Note: Not sure if enabling the modes repeatedly will cause the memory to leak.
Keep an eye out.

--Creating new modes--
Just create a new python file inside the 'modes' directory. It will be found
automatically.

The new mode file (module) needs to have implemented either two methods on its
top-level: init() and step(dt), or a class with the same name as the module
with a constructor method and step(self, dt).

Look at the example mode SmartTrack2.py for a reference.
"""

active = None

def enable(mode_name):
  # Find the module with the same name in this package.
  module_name = __name__ + "." + mode_name
  try:
    # Search for a class inside the module with the same name.
    member_name = module_name + "." + mode_name
    mode = Utils.import_member(member_name)
    rospy.loginfo("Enabling class %s as the active mode" % member_name)
  except AttributeError:
    # If couldn't find the class, use the module itself.
    mode = importlib.import_module(module_name)
    rospy.loginfo("Enabling module %s as the active mode" % module_name)

  # Taking this out of the try-except block to not catch AttributeErrors
  # during initialization.
  global active
  if inspect.isclass(mode):
    active = mode()
  else:
    mode.init()
    active = mode    

def disable():
  global active
  active = None
  rospy.loginfo("Active mode disabled.")

def step(dt):
  """
  'dt' holds the duration of one frame (the time since the last call) in
  seconds.
  """
  if active != None:
    active.step(dt)
