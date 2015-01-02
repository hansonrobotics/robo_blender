from .controllers import primary
from .controllers import demo_loops
import inputs, outputs
import rospy

class ManualHead:
  """
  Transmits the current neck, face and eye position without actually
  controlling anything.
  """

  def step(self, dt):
    outputs.store.full_head.transmit()
