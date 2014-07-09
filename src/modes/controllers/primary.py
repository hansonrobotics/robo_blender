import bpy

def point_head_at(vector3):
  bpy.data.objects["headtarget"].location = vector3

def point_eyes_at(vector3):
  bpy.data.objects["eyefocus"].location = vector3