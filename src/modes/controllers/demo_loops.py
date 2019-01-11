#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import random
import math

class SmoothExpressions:
  """
  Cycles through random face expressions varying the intensity in the form of
  a sum of two sines, one of lower and the other of higher frequencies.
  """

  class RandomHalfSine:

    timepos = 0.0
    length = 0.0

    def step(self, dt):
      result = math.sin(self.timepos/self.length*math.pi)
      self.timepos += dt
      return result

    def is_over(self):
      return self.timepos >= self.length

    def start_new(self):
      self.length = random.uniform(*self.length_range)
      self.timepos = 0.0

    def __init__(self, length_range):
      self.length_range = length_range
      self.start_new()

  exprstr = None
  amplitude2 = 0.0

  def step(self, dt):
    intensity = self.sine1.step(dt)*1.3 - self.sine2.step(dt)*self.amplitude2
    self.pub.publish(self.exprstr, intensity)
    
    if self.sine1.is_over():
      self.start_new()
    if self.sine2.is_over():
      self.sine2.start_new()
      self.amplitude2 = random.uniform(0.0, 0.3)

  def start_new(self):
    self.sine1.start_new()
    self.exprstr = random.choice(self.random_em)

  def __init__(self, pub):
    self.pub = pub
    self.random_em = [
      "eureka","innocent","annoyed", "evil","horrified",
      "happy", "sad", "surprised"
    ]
    self.sine1 = self.RandomHalfSine((2.0, 10.0))
    self.sine2 = self.RandomHalfSine((0.5, 2.0))

    self.start_new()
