#!/usr/bin/env python3
import rospy
import bpy
import math
import yaml
from std_msgs.msg import String

from bpy.app.handlers import persistent

import modes
import inputs

class RoboBlender:

  def handle_blendermode(self, msg):
    msg = msg.data
    if msg == "off":
      modes.disable()
    else:
      modes.enable(msg.data)

  def execute(self):
    rospy.init_node('robo_blender', anonymous=True)
    rospy.Subscriber('cmd_blendermode', String, handle_blendermode)
    inputs.initialize()

    @persistent
    def load_handler(dummy):
      pass
    bpy.app.handlers.scene_update_pre.append(load_handler)

    # Default mode
    modes.enable("SmartTrack")

rospy.loginfo("ROBO: Starting")
robo = RoboBlender()
rospy.loginfo("ROBO: Started")
robo.execute()
