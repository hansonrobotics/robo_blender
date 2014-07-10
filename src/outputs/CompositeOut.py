from . import get_instance

class CompositeOut:

  # The inside of this function will only be executed once.
  # self.outputs cannot be built inside __init__, because other outputs may
  # not have been built at that point.
  def transmit(self):
    self.outputs = [
      get_instance(output_name)
      for output_name in self.confentry["outputs"]
    ]

    # Set _transmit() to be executed when transmit() is called from this
    # point on
    self.transmit = self._transmit
    self.transmit()

  def _transmit(self):
    for output in self.outputs:
      output.transmit()

  def __init__(self, confentry):
    self.confentry = confentry