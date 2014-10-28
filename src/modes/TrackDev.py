from .controllers import tracking
import bpy
import inputs, outputs
import rospy
from eva_behavior.msg import tracking_action

class TrackDev:

  primary = inputs.store.pivision
  secondary = inputs.store.glancetarget

  def step(self, dt):
    self.ctrl.step(dt)
    outputs.store.neck_euler.transmit()
    outputs.store.eyes.transmit()

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl(self.primary)

    # Allow only a single subscriber in the class-wide parameter action_topic
    cls = type(self)
    old_action_topic = getattr(cls, 'action_topic', None)
    cls.action_topic = rospy.Subscriber('tracking_action', tracking_action, self.action_cb)
    if old_action_topic:
      old_action_topic.unregister()

  def action_cb(self, action):
    if action.action  == 'track':
      print(action.target)
      self.primary.change_topic(action.target)
    elif action.action == 'glance':
      self.secondary.change_topic(action.target)
      self.ctrl.glance(self.secondary)
