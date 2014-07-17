from .controllers import tracking
import bpy

class TrackDev:

  def step(self, dt):
    self.ctrl.step(dt)

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl(bpy.data.objects["pivision"])