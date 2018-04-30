import bpy
from random import randint

#Generate 50 cubes randomly located
#for i in range(50):
#    bpy.ops.mesh.primitive_cube_add(location=[ randint(-10,10) for axis in 'xyz'])
  
#############################
#Make an almost random movement to a sphere and show his current position and current frame
#############################
positions = (0,3,2),(4,1,6),(3,-5,1),(3,10,1),(1,8,1)
start_pos = (0,0,0)

ob = bpy.data.objects["Sphere"]
s=bpy.context.scene             #Identify the scene
f=s.objects['field']            #Locate the field
d=s.objects['distance']         #Lacate the distance

###########################
#Print out of the current value of the frame & his distance travelled
########################
def update(s):
    i=s.frame_current           #current frame number of field
    f.data.body = 'Frame : ' + str(i)         #change the body of the object by the valu
                                              #converted into a string
                                              
#Updating of the x current position
    x=s.objects['Sphere'].location[0]       #x position
    y=s.objects['Sphere'].location[1]       #y position
    z=s.objects['Sphere'].location[2]       #z position
    d.data.body = 'x : {0:.1f} meters\ny : {1:.1f} meters\nz : {2:.1f} meters'.format(x,y,z)
    
bpy.app.handlers.frame_change_pre.append( update )

##############################
# MOVEMENT OF THE SPHERE
############################

frame_num = 0

for position in positions :
    bpy.context.scene.frame_set(frame_num)
    ob.location = position
    ob.keyframe_insert(data_path="location",index = -1)
    frame_num += 20
    print(ob.location)
