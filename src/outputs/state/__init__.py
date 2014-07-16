
class StateOutputBase:

  def transmit(self):
    """
    Implement to send out the appropriate part of Blender rig state at current
    moment.
    """
    raise NotImplementedError

  def __init__(self, confentry):
  	""" Confentry is the yaml object associated with this output. """
  	return NotImplemented