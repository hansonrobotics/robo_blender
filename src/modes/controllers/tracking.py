from . import primary
from docutils.nodes import target
from mathutils import Vector
import random
import rospy
from eva_behavior.msg import event
from eva_behavior.msg import tracking_action
class EmaPoint:
  """Exponential moving average point"""

  def update(self, sample, dt):
    """Update the average with a new sample point."""
    #'alpha' is the standard EMA new sample weight over the previous ones
    alpha = 2.0/(self.N/dt+1)
    self.point = alpha*sample + (1-alpha)*self.point
    return self.point

  def __init__(self, N, init_point=Vector()):
    """Alpha coefficient is caluclated from N (the samples to average over)."""
    #N is the number of seconds to average over. More precisely the number
    #of seconds over which added samples gain ~86% of the total weight.
    self.N = float(N)
    self.point = init_point

class RandomTimer:

  def step(self, dt):
    """ Returns true every time it passes the randomly generated time interval. """
    self.time_idle += dt
    if self.time_idle > self.interval:
      print(self.time_idle)
      self.clear_time()
      return True
    return False

  def clear_time(self):
    self.time_idle = 0.0
    self.interval = random.gauss(*self.mu_sig)

  def __init__(self, mu_sig, initial_trigger=False):
    """
    Arguments:
    * mu_sig: tuple (Average time in which to generate new location, standard deviation of that time)
    * initial_trigger: boolean - whether or not to return True in the first call of step()
    """
    self.mu_sig = mu_sig
    self.clear_time()
    if initial_trigger:
      self.time_idle = self.interval + 1.0

class SaccadePipe:

  offset = Vector((0, 0, 0))

  @staticmethod
  def random_vector(radius):
    return Vector(
      (random.uniform(-1, 1) for i in range(3))
    ).normalized()*random.uniform(0, radius)

  def pipe(self, eyetarget_pos, dt):
    if self.timer.step(dt):
      self.offset = self.random_vector(self.radius)
    return eyetarget_pos + self.offset

  def __init__(self, radius=0.3, interval_mu_sig=(1.5, 0.8)):
    self.radius = radius
    self.timer = RandomTimer(interval_mu_sig, True)

class TrackSaccadeCtrl:
  """
  Initialize with the object to track and as long as step(dt) method is called
  the head and the eyes will smoothly track the object of interest.

  Eyes will saccade if the object is moving slow enough for the eyes to catch
  up with it.
  """
  
  @staticmethod
  def distance(v1, v2):
    return (v1 - v2).magnitude

  def step(self, dt):
    head_target = primary.get_head_target()
    eyes_target = primary.get_eyes_target()

    # Update EMAs
    for ema, target in [
      (self.head_ema, head_target),
      (self.eyes_ema, eyes_target)
    ]:
      target.location = ema.update(self.interest_obj.location, dt)

    # Saccade if eyes caught up with the interest_obj
    if self.distance(
      self.eyes_ema.point, self.interest_obj.location
    ) < self.saccade.radius*0.5:
      eyes_target.location = self.saccade.pipe(eyes_target.location, dt)
    else:
      self.saccade.timer.clear_time()

  def __init__(self, interest_obj, **kwargs):
    """
    Argument 'interest_obj' must have attribute "location".

    Takes the following optional keyword arguments:
    head_seconds=1.0 #Time for the head to catch up with interest_obj
    eyes_seconds=0.1 #Time for the eyes to catch up with interest_obj
    radius=0.1 #Maximum distance for the eyes to sacacde around interest_obj

    #Arguments for the Gaussian probability distribution for how often to saccade.
    interval_mu_sig=(1.5, 0.8)
    """
    self.interest_obj = interest_obj
    self.head_ema = EmaPoint(
      kwargs.get("head_seconds", 1.0),
      primary.get_head_target().location
    )
    self.eyes_ema = EmaPoint(
      kwargs.get("eyes_seconds", 0.1),
      primary.get_eyes_target().location
    )
    self.saccade = SaccadePipe(
      **{k:v for k,v in kwargs.items()
      if k in ["radius", "interval_mu_sig"]}
    )

    self.action_topic = rospy.Subscriber('tracking_action', tracking_action, self.action_cb)

  def action_cb(self, action):
      if (action.action  == 'track'):
          self.interest_obj.change_topic(action.target)

