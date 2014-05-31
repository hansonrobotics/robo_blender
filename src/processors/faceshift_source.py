import mathutils
from ros_faceshift.msg import fsMsgTrackingState

class faceshift_source:
    def process(self, msg, config):
        binding = config["source"].split(":")
        #print("MSG %s" % msg.m_headRotation)
        field = getattr(msg, binding[1])

        if binding[1] == "m_coeffs":
            element = field[int(binding[2])]
            #print("%s %s" % (config["source"], element))
            return element

        element = getattr(field, binding[2])
        return element
    def msgclass(self):
        return fsMsgTrackingState

processors["faceshift_source"] = faceshift_source()
