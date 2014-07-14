from .. import PendingMsg

class InputBase:
  """ All input classes should implement this base class. """

  def _pend_msg(self, handler):
    """
    Wrap your incoming ROS message handlers with this method for thread
    safety. Handler execution will be postponed until the next Blender
    callback.
    """
    return lambda msg: PendingMsg(msg, handler).pend()

  def __init__(self, confentry):
    """
    On construction input classes are given an entry from inputs.yaml config.
    """
    raise NotImplementedError