
class roi_from_camera:
    cameraInfo = None
    ciSub = None
    def __init__(self):
        print ("roi_camera INIT")
        self.ciSub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.setCameraInfo)

    def setCameraInfo(self, msg):
        self.cameraInfo = msg
        print("msg")
        self.ciSub.unregister()

    def process(self, msg, config):
        r = Vector([0,0,0])
        if self.cameraInfo.width > 0:
            r.x =  0.5 - (msg.x_offset+(msg.width/2.0))/self.cameraInfo.width
        if self.cameraInfo.height > 0:
            r.z =  0.5-(msg.y_offset+(msg.height/2.0))/self.cameraInfo.height
        return r

    def msgclass(self):
        return RegionOfInterest

processors["roi_from_camera"] = roi_from_camera()
