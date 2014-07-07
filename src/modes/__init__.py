import sys

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
  if active != None:
    active.step(dt)
