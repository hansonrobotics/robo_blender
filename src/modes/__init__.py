#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import sys
import importlib
import inspect
import rospy
import Utils
import traceback

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

The new mode file (module) needs to have a class with the same name as the
module with a step(self, dt) method.

Look at the example mode SmartTrack2.py for a reference.
"""

active = None

def enable(mode_name):
  module_name = ".".join([__name__, mode_name])
  member_name = ".".join([__name__] + [mode_name]*2)
  try:
    mode = Utils.import_member(member_name)
    rospy.loginfo("Activating mode %s" % mode_name)
  except ImportError as e:
    traceback.print_exc()
    rospy.logwarn("Couldn't load module %s" % module_name)
    return
  except AttributeError as e:
    traceback.print_exc()
    rospy.logwarn("Couldn't load class %s" % member_name)
    return

  # Taking this out of the try-except block to not catch AttributeErrors
  # during initialization.
  global active
  active = mode()

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
