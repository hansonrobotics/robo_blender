#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
from . import primary
from mathutils import Vector
import random
import bpy

class ExpectObject:
  """
  Object requested to track can take a few iterations to appear and later be
  destroyed. This wrapper fallbacks gracefully to another object in those
  cases.
  """
  @property
  def location(self):
    obj = self.get()
    if obj:
      return obj.location
    elif self.fallback_obj:
      return self.fallback_obj.location

  def get(self):
    """ The slash prefix is optional. """
    if self.obj_name in bpy.data.objects:
      return bpy.data.objects[self.obj_name]
    elif '/'+self.obj_name in bpy.data.objects:
      return bpy.data.objects['/'+self.obj_name]

  def __init__(self, obj_name, fallback_obj=None):
    self.obj_name = obj_name.lstrip('/')
    self.fallback_obj = fallback_obj

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

  def __init__(self, radius=0.2, interval_mu_sig=(1.5, 0.8)):
    self.radius = radius
    self.timer = RandomTimer(interval_mu_sig, True)

class GlancePipe:

  @staticmethod
  def weighted_avg(home, dest, weight):
    return dest * weight + home * (1 - weight)

  def is_ready(self):
    return self.target_obj.get() != None

  def eyes(self, primary_loc):
    # Towards the glance target
    return self.target_obj.location

  def head(self, primary_loc):
    # Towards a middle point between the primary and glance targets
    return self.weighted_avg(primary_loc, self.target_obj.location, 0.5)

  def step(self, dt):
    return self.timer.step(dt)

  def __init__(self, target_obj, timer):
    self.target_obj = target_obj
    self.timer = timer

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
    if self.interest_obj == None:
      return

    head_target = primary.get_head_target()
    eyes_target = primary.get_eyes_target()

    # Update EMAs
    if self.glance_pipe and self.glance_pipe.is_ready():
      # Towards the glance-piped target
      eyes_target.location = self.eyes_ema.update(self.glance_pipe.eyes(self.interest_obj.location), dt)
      head_target.location = self.head_ema.update(self.glance_pipe.head(self.interest_obj.location), dt)

      if self.glance_pipe.step(dt):
        self.glance_pipe = None
    else:
      # Towards the primary target
      eyes_target.location = self.eyes_ema.update(self.interest_obj.location, dt)
      head_target.location = self.head_ema.update(self.interest_obj.location, dt)

    # Saccade if eyes caught up with the interest_obj
    if self.distance(
      self.eyes_ema.point, self.interest_obj.location
    ) < self.saccade.radius*0.5:
      eyes_target.location = self.saccade.pipe(eyes_target.location, dt)
    else:
      self.saccade.timer.clear_time()

  def track(self, target_name):
    self.interest_obj = ExpectObject(target_name, primary.get_head_target())

  def glance(self, target_name, mu_sig=(1.25, 0.4)):
    self.glance_pipe = GlancePipe(
      ExpectObject(target_name, primary.get_head_target()),
      RandomTimer(mu_sig)
    )

  def __init__(self, interest_obj=None, **kwargs):
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
    self.glance_pipe = None
