import bpy

def get_by_type(array, val):
  for el in array:
    if getattr(el, "type", NotImplemented) == val:
      return el
  return None

class SelectArray(bpy.types.Operator):
  bl_idname = "robo.select_array"
  bl_label = "Select and Activate"
  bl_description = "Enable pose mode, select all the bones in the array and filter everything that's not selected out."

  @classmethod
  def poll(cls, context):
    return context.active_object.type == 'ARMATURE'

  def execute(self, context):
    bpy.ops.object.mode_set(mode="POSE")
    bpy.ops.pose.select_all(action="SELECT")
    get_by_type(context.area.spaces, "GRAPH_EDITOR").dopesheet.show_only_selected = True
    return {'FINISHED'}

class RoboArrayPanel(bpy.types.Panel):
  bl_label = "Robo Bone Array [Hanson Robotics]"
  bl_idname = "OBJECT_PT_hello"
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
    row.operator("robo.select_array")
    
  @classmethod
  def poll(cls, context):
    anim_data = context.active_object.animation_data
    if not anim_data:
      return False
    if not anim_data.action:
      return False
    return cls.group_name in anim_data.action.groups

def register():
  bpy.utils.register_class(RoboArrayPanel)
  bpy.utils.register_class(SelectArray)

def unregister():
  bpy.utils.unregister_class(RoboArrayPanel)
  bpy.utils.unregister_class(SelectArray)

if __name__ == "__main__":
  register()