from numpy import *

file = open("pos_test.txt","r")

#I_0 = zeros(3)

for line in file :
    I_0 = line.split(",")
    
    I_0[0] = float(I_0[0][1:])
    #I_0[1] = float(I_0[1])
    I_0[1] = float(I_0[1][:-3])
    
    #print(I_0)
    xh1 = sin(I_0[0])
    yh1 = cos(I_0[0])
    
    xv1 = sin(I_0[1])
    zv1 = cos(I_0[1])
    
    zeroo = I_0[0] - I_0[0]
    
    print(zeroo)
    """
    vecH1_loc = [sin(h1), cos(h1), 0]
    vecV1_loc = [sin(v1), 0, cos(v1)]
    """
    #print(I_0[2])
    #print(I_0[1])
    #print(I_0[2])
    
file.close()
    
"""
file = open("data_P2.txt","w")

for i in range(4*200):
    #file.write(str(base) +","+ str(axis) +","+ str(centroids[0]) +","+ str(centroids[1]) +","+ str(centroids[2]) +","+ str(centroids[3]) + "\n")
    file.write(str(parse_data(port)) + "\n")

file.close()
"""