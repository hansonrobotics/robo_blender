from .controllers import tracking
import bpy
import inputs, outputs
import rospy
from eva_behavior.msg import tracking_action

class TrackDev:

  primary = inputs.store.pivision

  def step(self, dt):
    self.ctrl.step(dt)
    outputs.store.neck_euler.transmit()
    outputs.store.eyes.transmit()

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl(self.primary)

    # Allow only a single subscriber in the class-wide parameter action_topic
    cls = type(self)
    if hasattr(cls, 'action_topic'):
      cls.action_topic.unregister()
    cls.action_topic = rospy.Subscriber('tracking_action', tracking_action, self.action_cb)

  def action_cb(self, action):
    if action.action  == 'track':
      self.primary.change_topic(action.target)
