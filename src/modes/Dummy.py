#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
from .controllers import primary
from .controllers import demo_loops
import inputs, outputs
import rospy
from .controllers import animate

class Dummy:
  """
  Does Nothing. Stops other blender modes
  """

  def __init__(self):
    self.init = False

  def step(self, dt):
    if not self.init:
        self.anim = animate.Animate('Armature')
        self.anim.stopAnimation()
        self.init = True
    return True
