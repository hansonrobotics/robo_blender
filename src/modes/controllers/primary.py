#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import bpy

def get_head_target():
  return bpy.data.objects["headtarget"]

def get_eyes_target():
  return bpy.data.objects["eyefocus"]