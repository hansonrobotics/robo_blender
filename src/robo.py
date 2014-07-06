#!/usr/bin/env python3
import rospy
import bpy
import math
import yaml

from bpy.app.handlers import persistent

from inputs import *

class RoboBlender:

  inputs = []

  @staticmethod
  def init_inputs():
    return [PiVision(), NmptSaliency()]

  def execute(self):
    self.input = self.load_inputs()
    rospy.init_node('robo_blender', anonymous=True)

    @persistent
    def load_handler(dummy):
      pass
    bpy.app.handlers.scene_update_pre.append(load_handler)

rospy.loginfo("ROBO: Starting")
robo = RoboBlender()
rospy.loginfo("ROBO: Started")
robo.execute()
