import sys

__doc__ = """
Package 'modes' holds modules that control the main flow of robot behavior.
When a module is set active, its step() method is called at the fps set in
config/config.yaml.

A different module can be enabled at any time by calling
the modes.enable() method or sending a ROS message to cmd_blendermode with a
different module name. Currently only one module can be active at a time.
"""

active = None

def enable(modulename):
  # Find the module with the specified name in this package and store its
  # reference.
  fullname = __name__ + "." + modulename
  __import__(fullname)
  global active
  active = sys.modules[fullname]

def disable():
  active = None

def step(dt):
  """
  'dt' holds the duration of one frame (the time since the last call) in
  seconds.
  """
  if active != None:
    active.step(dt)
