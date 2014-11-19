from .controllers import tracking
import outputs
import rospy
from eva_behavior.msg import tracking_action

class TrackDev:

  def step(self, dt):
    self.ctrl.step(dt)
    outputs.store.neck_euler.transmit()
    outputs.store.eyes.transmit()

  def __init__(self):
    self.ctrl = tracking.TrackSaccadeCtrl()

    # Allow only a single subscriber in the class-wide parameter action_topic
    cls = type(self)
    old_action_topic = getattr(cls, 'action_topic', None)
    cls.action_topic = rospy.Subscriber('tracking_action', tracking_action, self.action_cb)
    if old_action_topic:
      old_action_topic.unregister()

  def action_cb(self, action):
    if action.action  == 'track':
      self.ctrl.track(action.target)
    elif action.action == 'glance':
      self.ctrl.glance(action.target)
