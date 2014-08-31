from .controllers import tracking
import bpy
import inputs, outputs

class TrackDev:

  def step(self, dt):
    self.ctrl.step(dt)
    outputs.store.neck_euler.transmit()
    outputs.store.eyes.transmit()

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl(inputs.store.pivision)
