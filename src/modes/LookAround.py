from .controllers import primary
from .controllers import tracking
import inputs, outputs
import rospy
import cmath
from mathutils import Vector
import random
import math

class ImaginaryObject:

  location = Vector((0.0, 0.0, 0.0))

  @staticmethod
  def random_vector(vector_mu, radius_sig):
    offset2D = cmath.rect(
      random.gauss(0, radius_sig),
      random.uniform(0, 2*math.pi)
    )

    #Scaled to have the points less stretched over the horizontal axis
    offset = Vector((offset2D.real, 0.0, 0.5 * offset2D.imag))
    return vector_mu + offset

  def step(self, dt):
    if self.timer.step(dt):
      self.location = self.random_vector(*self.space_mu_sig)

  def __init__(self, space_mu_sig, time_mu_sig):
    """
    Arguments:
    * space_mu_sig: tuple (Vector around which to generate random location, standard deviation of the radius)
    * time_mu_sig: tuple (Average time in which to generate new location, standard deviation of that time)
    """
    self.space_mu_sig = space_mu_sig
    self.timer = tracking.RandomTimer(time_mu_sig, True)

class LookAround:
  """ Implemeent the LookAround mode."""
  def step(self, dt):
    self.interest.step(dt)
    self.tracking_ctrl.step(dt)
    outputs.store.eyes.transmit()
    outputs.store.neck_euler.transmit()

  def __init__(self):
    self.interest = ImaginaryObject(
      ((Vector((0.0, -1.0, 0.45)), 0.4)),
      (3.0, 1.5)
    )
    self.tracking_ctrl = tracking.TrackSaccadeCtrl(self.interest)
