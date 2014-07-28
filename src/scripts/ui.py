import bpy
import rospy
import math
from mathutils import *

bl_info = {
    "name": "Bone Array Utilities",
    "category": "Hanson Robotics",
}

# Name of the animation channels group.
# Used to distinguish between robot bone arrays and other armatures.
GROUP_NAME = "Array Rots"

def get_by_type(array, val):
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

#--Operators--

class SelectArray(bpy.types.Operator):
  bl_idname = "robo.bonearr_select"
  bl_label = "Select and Activate"
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
    for ns in get_namespaces_containing("pau2motors"):
      p2m_config = configs.add()
      p2m_config.name = ns
      p2m_config["params"] = rospy.get_param(ns+"/pau2motors")
    return {'FINISHED'}

class P2MCreateArray(bpy.types.Operator):
  bl_idname = "robo.p2m_create_bonearr"
  bl_label = "Create Bone Array"
  bl_description = "Create a bone array for the selected robot"
  
  @classmethod
  def poll(self, context):
    return len(robo_props().p2m_configs) > 0

  @staticmethod
  def _create_armature(namespace):
    if bpy.context.mode != "OBJECT":
      bpy.ops.object.mode_set(mode="OBJECT")
    name = "Bone Array (%s)" % namespace
    armature = bpy.data.armatures.new(name)
    ob = bpy.data.objects.new(name, armature)

    #Rotate 90 degrees
    ob.rotation_euler = Euler((math.pi/2, 0, 0), "XYZ")

    #Create an empty action to add animation channels in
    ob.animation_data_create()
    ob.animation_data.action = bpy.data.actions.new(
      name="Blank %s Action" % namespace
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

    obj = context.object

    row = layout.row()
    row.label(text="Hello world!", icon='WORLD_DATA')

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
    
    layout.row().label("Robots in ROS parameter server")
    
    obj = robo_props()
    layout.row().template_list("UI_UL_list", "custom", obj, "p2m_configs",
      obj, "p2m_active")
    
    row = layout.row()
    row.operator("robo.p2m_refresh")
    row.operator("robo.p2m_create_bonearr")
    
  @classmethod
  def poll(cls, context):
    return True

#--Data types--
        
class RoboBlenderProps(bpy.types.PropertyGroup):

  class P2MConfig(bpy.types.PropertyGroup):
    name = bpy.props.StringProperty() #Namespace in which the config was found.

    # Also holds ID property "params", which holds the config entry.
    # Reference it with p2m_config[<propname>] instead of p2m_config.<propname>

  p2m_configs = bpy.props.CollectionProperty(type=P2MConfig)
  p2m_active = bpy.props.IntProperty()
  
#--Module registration--

def register():
  bpy.utils.register_module(__name__)
  bpy.types.Scene.robo_blender = bpy.props.PointerProperty(type=RoboBlenderProps)
    
def unregister():
  del bpy.types.Scene.robo_blender
  bpy.utils.unregister_module(__name__)

if __name__ == "__main__":
  register()