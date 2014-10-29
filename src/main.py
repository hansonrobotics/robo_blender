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

# TODO: Cannot use set keys as they can be saved to blender file
class PersistentParams:

  dictname = "robo_blender"

  def get(self, key):
    return bpy.data.scenes[0][self.dictname].get(key)

  def set(self, key, val):
    bpy.data.scenes[0][self.dictname][key] = val

  def __init__(self):
    #If bpy.data.scenes[0] is stored locally, referencing it often causes Blender
    #to crash after Ctrl+Z (undo) is pressed.
    #Referencing the scene only from within bpy seems to work fine though.
    if not self.dictname in bpy.data.scenes[0].keys():
      bpy.data.scenes[0][self.dictname] = {}


class RoboBlender:

  config_dir = "config"

  def handle_blendermode(self, msg):
    msg = msg.data
    if msg == "disable":
      modes.disable()
    else:
      modes.enable(msg)

  def step(self, dt):
    inputs.execute_pending()
    modes.step(dt)

  def execute(self):
    # Try to shut down if the script is already running.
    blparams = PersistentParams()
    if blparams.get("running"):
      blparams.set("running", False)
      return
    blparams.set("running", True)

    rospy.init_node('robo_blender', anonymous=True)
    inputs.initialize(
      Utils.read_yaml(os.path.join(self.config_dir, "inputs.yaml"))
    )
    outputs.initialize(
      Utils.read_yaml(os.path.join(self.config_dir, "outputs.yaml"))
    )
    rospy.Subscriber('cmd_blendermode', String, self.handle_blendermode)
    
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
    modes.enable("Dummy")
    rospy.loginfo("ROBO: Started")

  def __init__(self):
    self.config = Utils.read_yaml(os.path.join(self.config_dir, "config.yaml"))
    self.frame_interval = 1.0/self.config["fps"]

print("ROBO: Loading")
robo = RoboBlender()
robo.execute()
