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

class PersistentParams:

  dictname = "robo_blender"

  @staticmethod
  def _get_prop_group(dictname):
    for name, scene in bpy.data.scenes.items():
      if dictname in scene.keys():
        return scene[dictname]
    return None

  def get(self, key):
    return self.prop_group.get(key)

  def set(self, key, val):
    self.prop_group[key] = val

  def __init__(self):
    prop_group = self._get_prop_group(self.dictname)
    if prop_group == None:
      bpy.data.scenes[0][self.dictname] = {}
      prop_group = bpy.data.scenes[0][self.dictname]

    self.prop_group = prop_group


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
    # Try to shut down if the script is already running.
    blparams = PersistentParams()
    if blparams.get("running"):
      blparams.set("running", False)
      return
    blparams.set("running", True)

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
      # Check whether to shut down
      if not blparams.get("running"):
        bpy.app.handlers.scene_update_pre.remove(handle_scene_update)
        rospy.loginfo("ROBO: Ended")
        return

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
    rospy.loginfo("ROBO: Started")

  def __init__(self):
    self.config = Utils.read_yaml(os.path.join(self.config_dir, "config.yaml"))
    self.frame_interval = 1.0/self.config["fps"]

print("ROBO: Loading")
robo = RoboBlender()
robo.execute()
