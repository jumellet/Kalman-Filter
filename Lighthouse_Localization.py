#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

from sympy import *
from sympy import solve_poly_system
from math import pi
from math import *
from sys import stdin
import numpy as np
#from reception import *

#port = serial_init()
#base, axis, centroids = parse_data(port)

#while 1 :
#    print(parse_data(port))


# The few vector math functions that we need
def cross(a, b):
	return [
		a[1]*b[2] - a[2]*b[1],
		a[2]*b[0] - a[0]*b[2],
		a[0]*b[1] - a[1]*b[0]
	]

def vecmul(a, k):
	return [
		a[0]*k,
		a[1]*k,
		a[2]*k
	]
def vecsub(a, b):
	return [
		a[0] - b[0],
		a[1] - b[1],
		a[2] - b[2]
	]

def dot(a, b):
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def len(a):
	return sqrt(dot(a,a))

def unitv(a):
	mag = len(a)
	return [a[0]/mag, a[1]/mag, a[2]/mag]

def ray(h,v):
	#print "a1=", a1*180/pi
	#print "a2=", a2*180/pi

#MAY BE THE PROJECTION FOR THE PLANES HAVE TO BE CHANGED


	# Around Z axis (Horizontal plane)
	planeH = [-sin(h), +cos(h), 0]
	# Around Y axis (Vertical plane)
	planeV = [0, sin(v), cos(v)]	

	# Cross the two planes to get the ray vector
	return unitv(cross(planeH,planeV))

#Transform timing to angle
def tick2angle(a):
	return a * pi / 8333

# Convert mpf array to float array
def mpfToFloat3(tab):
	tab[0] = float(tab[0])
	tab[1] = float(tab[1])
	tab[2] = float(tab[2])
	return tab

def vecToPts(vec):
	vec[0] = -vec[0]
	vec[1] = -vec[1]
	vec[2] = -vec[2]
	return vec

##################################################################
# The default sensor array is 40mm square
# This fits easily on a breadboard.
"""
file = open("data_P2.txt","w")

for i in range(4*200):
	#file.write(str(base) +","+ str(axis) +","+ str(centroids[0]) +","+ str(centroids[1]) +","+ str(centroids[2]) +","+ str(centroids[3]) + "\n")
	file.write(str(parse_data(port)) + "\n")
	               	
file.close()
"""

# Make better measurement of these distances
pos = [
	[+20,-20,0],
	[-20,-20,0],
	[-20,+20,0],
	[+20,+20,0],
]
"""
pos = [
	[-20,+20,0],
	[+20,+20,0],
	[-20,-20,0],
	[+20,-20,0],
]
"""

# Compute the distances between each of the sensors
# WHY THE "N" ???

r01 = N(len(vecsub(pos[0],pos[1])))
r02 = N(len(vecsub(pos[0],pos[2])))
r03 = N(len(vecsub(pos[0],pos[3])))
r12 = N(len(vecsub(pos[1],pos[2])))
r13 = N(len(vecsub(pos[1],pos[3])))
r23 = N(len(vecsub(pos[2],pos[3])))
"""
r01 = len(vecsub(pos[0],pos[1]))
r02 = len(vecsub(pos[0],pos[2]))
r03 = len(vecsub(pos[0],pos[3]))
r12 = len(vecsub(pos[1],pos[2]))
r13 = len(vecsub(pos[1],pos[3]))
r23 = len(vecsub(pos[2],pos[3]))
"""
# Translate them into angles, compute each ray vector for each sensor
# and then compute the angles between them
# TRANSLATE --> SAMPLES = CENTROIDS && ID = BASE
def lighthouse_pos(samples):
	# Order the centoids of diodes to their position
	v0 = ray(samples[0][3] * pi / 8333.3333333, samples[1][3] * pi / 8333.3333333)
	v1 = ray(samples[0][2] * pi / 8333.3333333, samples[1][2] * pi / 8333.3333333)
	v2 = ray(samples[0][0] * pi / 8333.3333333, samples[1][0] * pi / 8333.3333333)
	v3 = ray(samples[0][1] * pi / 8333.3333333, samples[1][1] * pi / 8333.3333333)	
	
	#v0 = ray(samples[0][0] * pi / 8333, samples[1][0] * pi / 8333)
	#v1 = ray(samples[0][1] * pi / 8333, samples[1][1] * pi / 8333)
	#v2 = ray(samples[0][2] * pi / 8333, samples[1][2] * pi / 8333)
	#v3 = ray(samples[0][3] * pi / 8333, samples[1][3] * pi / 8333)	
	v01 = dot(v0,v1)
	v02 = dot(v0,v2)
	v03 = dot(v0,v3)
	v12 = dot(v1,v2)
	v13 = dot(v1,v3)
	v23 = dot(v2,v3)
	#print("v0 = ", v0)
	#print("v1 = ", v1)
	#print("v2 = ", v2)
	#print("v3 = ", v3)
	
	print("v01 = ", acos(v01) * 180 / pi, " deg")
	print("v02 = ", acos(v02) * 180 / pi, " deg")
	print("v03 = ", acos(v03) * 180 / pi, " deg")
	print("v12 = ", acos(v12) * 180 / pi, " deg")
	print("v13 = ", acos(v13) * 180 / pi, " deg")
	print("v23 = ", acos(v23) * 180 / pi, " deg")
	

	k0, k1, k2, k3 = symbols('k0, k1, k2, k3')
	sol = nsolve((
		k0**2 + k1**2 - 2*k0*k1*v01 - r01**2,
		k0**2 + k2**2 - 2*k0*k2*v02 - r02**2,
		k0**2 + k3**2 - 2*k0*k3*v03 - r03**2,
		k2**2 + k1**2 - 2*k2*k1*v12 - r12**2,
		k3**2 + k1**2 - 2*k3*k1*v13 - r13**2,
		k2**2 + k3**2 - 2*k2*k3*v23 - r23**2,
	),
		(k0, k1, k2, k3),
	        (1000,1000,1000,1000),    #Why this vector ???
		verify=False  # ignore tolerance of solution
	)


	print("sol = ",sol)

	# Convet mpf numbers into float
	p0 = mpfToFloat3(vecmul(v0,sol[0]))
	p1 = mpfToFloat3(vecmul(v1,sol[1]))
	p2 = mpfToFloat3(vecmul(v2,sol[2]))
	p3 = mpfToFloat3(vecmul(v3,sol[3]))
	
	
	# Order each vector
	def OrderVec(vec):
		temp = vec[2]
		vec[2] = vec[1]
		vec[1] = vec[0]
		vec[0] = temp
		return vec
	
	p0 = vecToPts(OrderVec(p0))
	p1 = vecToPts(OrderVec(p1))
	p2 = vecToPts(OrderVec(p2))
	p3 = vecToPts(OrderVec(p3))

	print("p0 = ", p0)
	print("p1 = ", p1)
	print("p2 = ", p2)
	print("p3 = ", p3)

	# compute our own estimate of the error
	print("err01 = ", len(vecsub(p0,p1)) - r01, " mm")
	print("err02 = ", len(vecsub(p0,p2)) - r02, " mm")
	print("err03 = ", len(vecsub(p0,p3)) - r03, " mm")
	print("err12 = ", len(vecsub(p1,p2)) - r12, " mm")
	print("err13 = ", len(vecsub(p1,p3)) - r13, " mm")
	print("err23 = ", len(vecsub(p2,p3)) - r23, " mm")
	
	return [p0, p1, p2, p3]
	
	"""
	v0 = ray(tick2angle(samples[0][id*2]), tick2angle(samples[0][id*2+1]))
	v1 = ray(tick2angle(samples[1][id*2]), tick2angle(samples[1][id*2+1]))
	v2 = ray(tick2angle(samples[2][id*2]), tick2angle(samples[2][id*2+1]))
	v3 = ray(tick2angle(samples[3][id*2]), tick2angle(samples[3][id*2+1]))
	"""
	sol = nsolve((
		k0**2 + k1**2 - 2*k0*k1*v01 - r01**2,
		k0**2 + k2**2 - 2*k0*k2*v02 - r02**2,
		k0**2 + k3**2 - 2*k0*k3*v03 - r03**2,
		k2**2 + k1**2 - 2*k2*k1*v12 - r12**2,
		k3**2 + k1**2 - 2*k3*k1*v13 - r13**2,
		k2**2 + k3**2 - 2*k2*k3*v23 - r23**2,
	),
		(k0, k1, k2, k3),
	        (0,0,0,0),    #Why this vector ???
		verify=False  # ignore tolerance of solution
	)

	
	


#
# The four parameter sets as input are the raw tick measurements
# in 48 MHz system clock values.
#

# Accumulate lots of samples for each sensor while they
# are stationary at the origin (0,0,0) and compute
# the average of them so that we have a better measurement.
#
# If we have a good view this should be just a few seconds.
total_count = 0
count = [0,0,0,0]
samples1 = [[0,0,0,0],
            [0,0,0,0]]
samples2 = [[0,0,0,0],
            [0,0,0,0]]


# Throw away any old serial data
#for n in range(0,200):
#	line = stdin.readline()
file = open("data.txt","r")

for line in file :
	#line = stdin.readline()
	#cols = line.split(",", 6)
	#print(cols)
	#i = int(cols[0])
	#if i < 0 or i > 3:
	#	print("parse error")
	#	continue
	centroids = [0,0,0,0]
	
	base, axis, centroids[0], centroids[1], centroids[2], centroids[3] = line.split(",")
	base = int(base[1:])
	axis = int(axis)
	# Conversion of the string reception into floats
	centroids[0] = float(centroids[0][2:])
	centroids[1] = float(centroids[1])
	centroids[2] = float(centroids[2])
	centroids[3] = centroids[3][1:]
	centroids[3] = float(centroids[3][:-4])
	
	#print("base = ", base)
	#print("axis = ", axis)
	#print("centroids = ", centroids)
	total_count += 1

	if (base == 0 and axis == 0) :
		samples1[0][0] += centroids[0]
		samples1[0][1] += centroids[1]
		samples1[0][2] += centroids[2]
		samples1[0][3] += centroids[3]
		count[0] += 1
	if (base == 0 and axis == 1) :
		samples1[1][0] += centroids[0]
		samples1[1][1] += centroids[1]
		samples1[1][2] += centroids[2]
		samples1[1][3] += centroids[3]
		count[1] += 1
	if (base == 1 and axis == 0) :
		samples2[0][0] += centroids[0]
		samples2[0][1] += centroids[1]
		samples2[0][2] += centroids[2]
		samples2[0][3] += centroids[3]
		count[2] += 1
	if (base == 1 and axis == 1) :
		samples2[1][0] += centroids[0]
		samples2[1][1] += centroids[1]
		samples2[1][2] += centroids[2]
		samples2[1][3] += centroids[3]
		count[3] += 1

# Check that we have enough of each
# at least 10% of the samples must be from this one
file.close()

fail = False
print("count = ",count)
print("total_count = ",total_count)

for i in range(4):
	#if count[i] < total_count / 10:
	#	print(str(i) + ": too few samples")
	#	exit(-1)
	samples1[0][i] /= count[i]
	
	samples1[1][i] /= count[i]
	samples2[0][i] /= count[i]
	samples2[1][i] /= count[i]
	print("samples2 0",i," = ",samples2[1][i])

print(samples1)
print(samples2)

# Calculus of the vector pointing LH from Tracker's diodes
pts4_1 = lighthouse_pos(samples1)
pts4_2 = lighthouse_pos(samples2)



print("points 1 : ", pts4_1)
print("points 2 : ", pts4_2)
#print(vecsub(p0_1, p1_1))

#Function that find normal vector from 3 pts
def ptsToNormal(A, B, C):
	#u = vecsub(ABC[1], ABC[0])
	#v = vecsub(ABC[2], ABC[0])
	u = vecsub(A, B)
	v = vecsub(C, B)	
	return cross(u, v)

# This function transform 4 points describing a non perfect plan into an orthonormal
def ptsToOrth(ptsTab4):
	vecNormal0 = ptsToNormal(ptsTab4[0], ptsTab4[1], ptsTab4[2])
	vecNormal1 = ptsToNormal(ptsTab4[1], ptsTab4[2], ptsTab4[3])
	vecNormal2 = ptsToNormal(ptsTab4[2], ptsTab4[3], ptsTab4[0])
	vecNormal3 = ptsToNormal(ptsTab4[3], ptsTab4[0], ptsTab4[1])
	
	vecNormal = [0, 0, 0]
	for i in range(3):
		vecNormal[i] = (vecNormal0[i] + vecNormal1[i] + vecNormal2[i] + vecNormal3[i]) / 4
		
	"""
	print("Normal vector = ",vecNormal0)
	print("Normal vector = ",vecNormal1)
	print("Normal vector = ",vecNormal2)
	print("Normal vector = ",vecNormal3)
	"""
	
	return unitv(vecNormal)
	#return a 3x3 Matrix   

"""
# Function that average the four diodes to make the vector centered
def Average4pts(ptsTab4):
	newPt = [0,0,0]
	for i in range(4) :
		for j in range(3) :
			newPt[j] += ptsTab4[i][j]
	for k in range(3) :
		newPt[k] = newPt[k] / 4
	return newPt

print("Averaged Vec 1 : ",Average4pts(pts4_1))
print("Averaged Vec 2 : ",Average4pts(pts4_2))
"""

#print(ptsToOrth(pts4_1))

# HT bases explained into the LH bases
xyz_1 = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]
xyz_1[2] = ptsToOrth(pts4_1)
xyz_1[0] = unitv(vecsub(pts4_1[1], pts4_1[2]))
xyz_1[1] = cross(xyz_1[2], xyz_1[0])

xyz_2 = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]
xyz_2[2] = ptsToOrth(pts4_2)
xyz_2[0] = unitv(vecsub(pts4_2[1], pts4_2[2]))
xyz_2[1] = cross(xyz_2[2], xyz_2[0])
"""
[[-0.84212949  0.2901653   0.45455696]
 [-0.1154597  -0.92004551  0.37340373]
 [ 0.50342283  0.26994411  0.82078964]]
 """

# Here we search the rotation matrix of the LH 1 & 2
# Base of Tracker
XYZ = np.array(xyz_1)
Id = np.array([[1,0,0],[0,1,0],[0,0,1]])
print(XYZ)
XYZ_ = np.array(xyz_2)
Id = np.array([[1,0,0],[0,1,0],[0,0,1]])
print(XYZ_)

# This for an orthogonal matrix is equivalent to Transpose XYZ
#MatRot = np.linalg.solve(XYZ, Id)