from bge import logic
#import GameLogic
from mathutils import *
from math import *

scene = logic.getCurrentScene()
EmptyLH11 = scene.objects['EmptyLH11']
LH11 = scene.objects['LH11']
EmptyLH12 = scene.objects['EmptyLH12']
LH12 = scene.objects['LH12']

obj3 = scene.objects['Empty']
Car = scene.objects['Car']

#print("EmptyLH11",obj.position)
#print("LH11",obj2.position)

#Variables to calculate the distance between a point and a plane
#pt = obj3.position
pt = Car.position  #We observe the car
plane_co_LH11 = LH11.position    #A point on the plane
plane_no_LH11 = LH11.position - EmptyLH11.position #The direction the plane is facing
plane_co_LH12 = LH12.position
plane_no_LH12 = - LH12.position + EmptyLH12.position

#Distance between a point and a plane
distX = geometry.distance_point_to_plane(pt, plane_co_LH11, plane_no_LH11)
distY = geometry.distance_point_to_plane(pt, plane_co_LH12, plane_no_LH12)

#Time
t=len(logic.data)/3

#update data file
logic.data += [t, distX, distY]

#Previous measurements
prevDistX = logic.data[len(logic.data)-5]
prevDistY = logic.data[len(logic.data)-4]

#Angle of scan
if prevDistX < 0 and distX >= 0 :
    XLH11 = asin(- LH11.localOrientation[0][1]) + pi/2
    #print(XLH11)
if prevDistY < 0 and distY >= 0 :
    YLH12 = acos(LH12.localOrientation[1][0])
    #print(YLH12)

#print(distY)
#print("Old : ", prevDistX, prevDistY)
#print("New : ", distX, distY)
