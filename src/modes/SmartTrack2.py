from .controllers import primary
from .controllers import demo_loops
from .controllers import tracking
import inputs, outputs
import rospy
from basic_head_api.msg import MakeFaceExpr

class SmartTrack2:
  """
  Tracks pivison input and produces random, but smooth expressions using
  basic_head_api.
  """

  def step(self, dt):
    self.tracking_ctrl.step(dt)
    self.exprs.step(dt)
    #outputs.store.neck_euler.transmit()

  def __init__(self):
    self.exprs = demo_loops.SmoothExpressions(
      rospy.Publisher("make_face_expr", MakeFaceExpr, queue_size=2)
    )
    self.tracking_ctrl = tracking.TrackSaccadeCtrl(inputs.store.pivision)