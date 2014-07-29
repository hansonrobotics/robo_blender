import bpy
import rospy
import math
from mathutils import *
import time
from trajectory_msgs.msg import JointTrajectory

__desc__ = """
UI for creating bone arrays out of ROS param server and transmitting their
rotations to ROS.

This should work both as a script and as an addon (if it's installed through
Blender preferences).

TODO:
 - Finish TransmitManager (specifically _build_msg), also test.
 - Add bone array scale/location/rotation locks in edit mode,
   and in pose mode too except the z rotation.
"""

bl_info = {
    "name": "Bone Array Utilities",
    "category": "Hanson Robotics",
}

# Name of the animation channels group.
# Used to distinguish between robot bone arrays from other armatures.
GROUP_NAME = "Array Rots"

def get_by_type(array, val):
  """
  Returns an entry in the given array, which has the 'type' attributes of the
  given value.
  """
  for el in array:
    if getattr(el, "type", NotImplemented) == val:
      return el
  return None

def get_namespaces_containing(name_piece):
  """Scan ROS parameter names and return 'directories' where name_piece is found."""
  return set([
    "/".join(param_path[:param_path.index(name_piece)])
    for param_path in [n.split("/") for n in rospy.get_param_names()]
    if name_piece in param_path
  ])

def robo_props():
  """Get the RoboBlenderProps instance."""
  return bpy.data.scenes[0].robo_blender

def roscore_running():
  """
  Checks whether it can connect to roscore and stores that in the
  RoboBlenderProps instance.
  """
  try:
    rospy.get_param_names()
  except ConnectionRefusedError:
    robo_props().connected = False
    return False
  robo_props().connected = True
  return True

class TransmitManager:
  """ Handles bone array rotation transmissions to ROS """

  objects_on = [] # An array of objects that are on for transmission
  publishers = {} # Maps topic names to publisher instances

  def turn_on(self, obj):
    objects_on.append(obj)

  def turn_off(self, obj):
    objects_of.remove(obj)

    # Remove publisher
    topic = self._get_topic(obj)
    if topic in publishers:
      del publishers[topic]

  @staticmethod
  def _get_topic(obj):
    return obj.p2m_config.namespace + "/cmd_joints"

  @staticmethod
  def _build_msg(obj):
    for bone in obj.pose.bones:
      bone.rotation_euler[2]

  def _transmit(self, obj):
    topic = self._get_topic(obj)

    # Add publisher
    if not topic in publishers:
      publishers[topic] = rospy.Publisher(
        topic, JointTrajectory, queue_size=2
      )

    publishers[topic].pub(self._build_msg(obj))

  @persistent
  def handle_scene_update(self, context):
    # Shut down if 'Run Script' was pressed again.
    if robo_props().script_id != script_id
      bpy.app.handlers.scene_update_post.remove(self.handle_scene_update)
      return

    for ob in objects_on:
      self._transmit(ob)

  def __init__(self):
    for ob in bpy.data.objects:
      if ob.bonearr_transmit:
        self.objects_on.append(ob)

    bpy.app.handlers.scene_update_post.append(self.handle_scene_update)

#--Operators--

class SelectArray(bpy.types.Operator):
  bl_idname = "robo.bonearr_select"
  bl_label = "Select Bones in Pose Mode"
  bl_description = "Enable pose mode, select all the bones in the array and filter everything that's not selected out"

  @classmethod
  def poll(cls, context):
    return context.active_object and context.active_object.type == 'ARMATURE'

  def execute(self, context):
    #Deselect all objects
    if bpy.context.mode != "OBJECT":
      bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")

    #Select all bones in the active armature
    bpy.context.active_object.select = True
    bpy.ops.object.mode_set(mode="POSE")
    bpy.ops.pose.select_all(action="SELECT")

    #Show only selected channels in graph editor
    get_by_type(context.area.spaces, "GRAPH_EDITOR").dopesheet.\
    show_only_selected = True

    #Expand channel group in UI
    bpy.context.active_object.animation_data.action.groups[GROUP_NAME].\
    show_expanded = True
    return {'FINISHED'}

class P2MRefresh(bpy.types.Operator):
  bl_idname = "robo.p2m_refresh"
  bl_label = "Refresh"
  bl_description = "Get available robots from ROS parameter server"

  def execute(self, context):
    configs = robo_props().p2m_configs
    configs.clear()

    if roscore_running():
      for ns in get_namespaces_containing("pau2motors"):
        p2m_config = configs.add()
        p2m_config.name = ns if len(ns) > 0 else "<global namespace>"
        p2m_config.namespace = ns
        p2m_config["params"] = rospy.get_param(ns+"/pau2motors")
      return {'FINISHED'}
    else:
      return {'CANCELLED'}
    

class P2MCreateArray(bpy.types.Operator):
  bl_idname = "robo.p2m_create_bonearr"
  bl_label = "Create Bone Array"
  bl_description = "Create a bone array for the selected robot"
  
  @classmethod
  def poll(self, context):
    return robo_props().connected and len(robo_props().p2m_configs) > 0

  @staticmethod
  def _create_armature(name):
    """ Argument 'name' can be a namespace or string '<global namespace>' """
    if bpy.context.mode != "OBJECT":
      bpy.ops.object.mode_set(mode="OBJECT")
    armature = bpy.data.armatures.new("Bone Array (%s)" % name)
    ob = bpy.data.objects.new("Bone Array (%s)" % name, armature)

    #Rotate 90 degrees
    ob.rotation_euler = Euler((math.pi/2, 0, 0), "XYZ")

    #Create an empty action to add animation channels in
    ob.animation_data_create()
    ob.animation_data.action = bpy.data.actions.new(
      name="Blank %s Action" % name
    )

    #Link new armature to scene
    scene = bpy.context.scene
    scene.objects.link(ob)
    scene.objects.active = ob

  @staticmethod
  def _create_bone(motor_name, motor_entry):
    armature = bpy.context.active_object
    bone = armature.data.edit_bones.new(motor_name)

    #Create a rotation animation channel
    armature.animation_data.action.fcurves.new(
      'pose.bones["%s"].rotation_euler' % motor_name,
      index=2,
      action_group=GROUP_NAME
    )
    return bone

  @staticmethod
  def _locations(num_bones):
    GAP = 0.75
    #Position bones to resemble a square array
    rows = math.floor(math.sqrt(num_bones))
    cols = math.ceil(num_bones/rows)

    for row in range(rows):
      for col in range(cols):
        yield (
          Vector((col*GAP, -row*GAP, 0)), # Head
          Vector((col*GAP, -row*GAP, 1))  # Tail
        )

  def execute(self, context):
    p2m_config = robo_props().p2m_configs[robo_props().p2m_active]
    p2m_motors = p2m_config["params"]["motors"]
    
    # Generates location vectors
    location_gen = self._locations(len(p2m_motors))

    self._create_armature(p2m_config.name)
    bpy.ops.object.mode_set(mode='EDIT')
    for motor_name, motor_entry in p2m_motors.items():
      new_bone = self._create_bone(motor_name, motor_entry)
      new_bone.head, new_bone.tail = next(location_gen)

    #Associate the config with the object for rotation transmission
    context.active_object.p2m_config = p2m_config

    #Shown animation channels in Dope Sheet and Graph Editor
    bpy.ops.robo.bonearr_select()

    return {'FINISHED'}

#--Panels--

class RoboArrayPanel(bpy.types.Panel):
  bl_label = "Robo Bone Array [Hanson Robotics]"
  bl_idname = "OBJECT_PT_robo_bone_array"
  bl_space_type = 'GRAPH_EDITOR'
  bl_region_type = 'UI'

  def draw(self, context):
    layout = self.layout

    ob = context.active_object

    box = layout.row().box()
    box.prop(ob, 'bonearr_transmit', toggle=True)

    row = layout.row()
    row.operator("robo.bonearr_select")
    
  @classmethod
  def poll(cls, context):
    ob = context.active_object
    if not ob:
      return False
    if not ob.animation_data:
      return False
    if not ob.animation_data.action:
      return False
    return GROUP_NAME in ob.animation_data.action.groups

class CreatePanel(bpy.types.Panel):
  bl_label = "Create [Hanson Robotics]"
  bl_idname = "GRAPH_ED_robo_create"
  bl_space_type = 'GRAPH_EDITOR'
  bl_region_type = 'UI'

  def draw(self, context):
    layout = self.layout
    
    if robo_props().connected:
      layout.row().label("Robots in ROS parameter server")
      obj = robo_props()
      layout.row().template_list("UI_UL_list", "custom", obj, "p2m_configs",
        obj, "p2m_active")
    else:
      layout.row().label("Roscore not running")

    row = layout.row()
    row.operator("robo.p2m_refresh")
    row.operator("robo.p2m_create_bonearr")
    
  @classmethod
  def poll(cls, context):
    return True

#--Data types--
        
class RoboBlenderProps(bpy.types.PropertyGroup):

  class P2MConfig(bpy.types.PropertyGroup):
    # Either the same as namespace or string '<global namespace>'
    name = bpy.props.StringProperty()
    # Namespace in which the config was found.
    namespace = bpy.props.StringProperty()

    # Also holds ID property "params", which holds the config entry.
    # Reference it with p2m_config[<propname>] instead of p2m_config.<propname>

  p2m_configs = bpy.props.CollectionProperty(type=P2MConfig)
  p2m_active = bpy.props.IntProperty()
  connected = bpy.props.BoolProperty()

  # Used to execute callbacks on only a single script.
  script_id = bpy.props.IntProperty()
  
#--Module registration--

#Unique to this script run
script_id = int(time.time()*1000)

def register():
  bpy.utils.register_module(__name__)

  #Stores most of the data required to communicate between method runs in this file.
  #Get this object with robo_props()
  bpy.types.Scene.robo_blender = bpy.props.PointerProperty(type=RoboBlenderProps)
  robo_props().script_id = script_id # Save the latest script id on the scene data

  #Specifies whether to transmit rotations of this bone array to ROS.
  bpy.types.Object.bonearr_transmit = bpy.props.BoolProperty(
    name="Transmit Rotations", update=on_bonearr_transmit_update
  )
  #Stores the config from which the array was created.
  bpy.types.Object.p2m_config = bpy.props.PointerProperty(type=P2MConfig)

  global tx_manager
  tx_manager = TransmitManager()

  bpy.ops.robo.p2m_refresh()
    
def unregister():
  del bpy.types.Scene.robo_blender
  bpy.utils.unregister_module(__name__)

if __name__ == "__main__":
  register()

# Would have placed this function inside TransmitManager, but 'self' gets
# overriden in that case.
def on_bonearr_transmit_update(self, context):
  if self.bonearr_transmit:
    tx_manager.turn_on(self)
  else:
    tx_manager.turn_off(self)