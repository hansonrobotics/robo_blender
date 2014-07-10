#!/usr/bin/env python3
import rospy
import bpy
import math
import yaml
import time
import os
import Utils
from std_msgs.msg import String
from bpy.app.handlers import persistent
import modes, inputs, outputs

class RoboBlender:

  config_dir = "config"

  def handle_blendermode(self, msg):
    msg = msg.data
    if msg == "disable":
      modes.disable()
    else:
      modes.enable(msg)

  def step(self, dt):
    modes.step(dt)

  def execute(self):
    rospy.init_node('robo_blender', anonymous=True)
    rospy.Subscriber('cmd_blendermode', String, self.handle_blendermode)
    inputs.initialize(
      Utils.read_yaml(os.path.join(self.config_dir, "inputs.yaml"))
    )
    outputs.initialize(
      Utils.read_yaml(os.path.join(self.config_dir, "outputs.yaml"))
    )

    @persistent
    def handle_scene_update(dummy):
      # Limit execution to intervals of self.frame_interval
      t = time.time()
      if t - self.lastframe < self.frame_interval:
        return
      self.lastframe = self.lastframe + self.frame_interval
      # Limited execution starts here.
      self.step(self.frame_interval)

    self.lastframe = time.time()
    bpy.app.handlers.scene_update_pre.append(handle_scene_update)

    # Enable default mode
    modes.enable("ManualHead")

  def __init__(self):
    self.config = Utils.read_yaml(os.path.join(self.config_dir, "config.yaml"))
    self.frame_interval = 1.0/self.config["fps"]

print("ROBO: Starting")
robo = RoboBlender()
robo.execute()
rospy.loginfo("ROBO: Started")
