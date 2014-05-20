import rospy
import bpy
import math

from bpy.app.handlers import persistent
from std_msgs.msg import Float64

xaxis = 0
dynamixel_namespace = rospy.get_namespace()
rospy.init_node('blender_arm', anonymous=True)
dm = rospy.Publisher(dynamixel_namespace + 'tilt_controller/command', Float64)

@persistent
def load_handler(dummy):
    global xaxis
    global dm
    if xaxis != bpy.data.objects['Armature'].pose.bones['bracket3'].x_axis:
        xaxis = bpy.data.objects['Armature'].pose.bones['bracket3'].x_axis
        angle = math.atan2(xaxis[1], xaxis[0])
        print("ANGLE:%s" % angle)
        dm.publish(float(angle))

bpy.app.handlers.scene_update_post.append(load_handler)

print("Started")
