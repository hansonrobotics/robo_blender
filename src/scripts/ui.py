import bpy
import rospy

bl_info = {
    "name": "Bone Array Utilities",
    "category": "Hanson Robotics",
}

def get_by_type(array, val):
  for el in array:
    if getattr(el, "type", NotImplemented) == val:
      return el
  return None

def get_namespaces_containing(name_piece):
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
    return context.active_object.type == 'ARMATURE'

  def execute(self, context):
    bpy.ops.object.mode_set(mode="POSE")
    bpy.ops.pose.select_all(action="SELECT")
    get_by_type(context.area.spaces, "GRAPH_EDITOR").dopesheet.show_only_selected = True
    return {'FINISHED'}

class P2MRefresh(bpy.types.Operator):
  bl_idname = "robo.p2m_refresh"
  bl_label = "Refresh"
  bl_description = "Get available robots from ROS parameter server"

  def execute(self, context):
    configs = robo_props().p2m_configs
    configs.clear()
    for ns in get_namespaces_containing("pau2motors"):
      configs.add().name = ns
      #configs.add().name = ns
    return {'FINISHED'}

class P2MCreateArray(bpy.types.Operator):
  bl_idname = "robo.p2m_create_bonearr"
  bl_label = "Create Bone Array"
  bl_description = "Create a bone array for the selected robot"
  
  @classmethod
  def poll(self, context):
    return len(robo_props().p2m_configs) > 0

  def execute(self, context):
    robo_props().p2m_configs.clear()
    return {'FINISHED'}

#--Panels--

class RoboArrayPanel(bpy.types.Panel):
  bl_label = "Robo Bone Array [Hanson Robotics]"
  bl_idname = "OBJECT_PT_robo_bone_array"
  bl_space_type = 'GRAPH_EDITOR'
  bl_region_type = 'UI'
  
  group_name = "Array Rots"

  def draw(self, context):
    layout = self.layout

    obj = context.object

    row = layout.row()
    row.label(text="Hello world!", icon='WORLD_DATA')

    row = layout.row()
    row.label(text="Active object is: " + obj.name)
    row = layout.row()
    row.prop(obj, "name")

    row = layout.row()
    row.operator("robo.bonearr.select")
    
  @classmethod
  def poll(cls, context):
    anim_data = context.active_object.animation_data
    if not anim_data:
      return False
    if not anim_data.action:
      return False
    return cls.group_name in anim_data.action.groups

class CreatePanel(bpy.types.Panel):
  bl_label = "Create [Hanson Robotics]"
  bl_idname = "GRAPH_ED_robo_create"
  #bl_space_type = 'GRAPH_EDITOR'
  #bl_region_type = 'UI'
  bl_space_type = "PROPERTIES"
  bl_region_type = "WINDOW"

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
    name = bpy.props.StringProperty()

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