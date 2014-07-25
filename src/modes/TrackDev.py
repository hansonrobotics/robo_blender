from .controllers import tracking
import bpy
import inputs

class TrackDev:

  def step(self, dt):
    self.ctrl.step(dt)

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl(inputs.store.pivision)