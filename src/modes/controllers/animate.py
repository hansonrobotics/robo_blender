import bpy

class Animate:

    def __init__(self, obj):
        #Object for which actions is applied
        self.actionObj = bpy.data.objects[obj]

    # Animation list from blender
    def getAnimationList(self):
        anims = {}
        for s in bpy.data.actions.keys():
            anims[s] = {}
            anims[s]['length'] = bpy.data.actions[s].frame_range[1]-bpy.data.actions[s].frame_range[0]+1
        return anims

    # Sets animation
    def setAnimation(self,act):
        # Check if correct object selected
        if self.actionObj.animation_data is not None:
           self.actionObj.animation_data.action = bpy.data.actions[act]
           bpy.ops.screen.frame_jump()

    def playAnimation(self):
        if not bpy.context.screen.is_animation_playing:
            bpy.ops.screen.animation_play()

    def stopAnimation(self):
        if bpy.context.screen.is_animation_playing:
            bpy.ops.screen.animation_play()

    def resetAnimation(self):
        bpy.ops.screen.frame_jump()
        self.stopAnimation()



