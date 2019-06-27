from PIL import Image
import numpy as np
import math
def distance(x,y):
    diffx = x[0]-y[0]
    diffy = x[1]-y[1]
    diff = diffx*diffx + diffy*diffy
    return math.sqrt(diff)
def getpoint(resolution,origin,mapresolution,path):
    I = Image.open(path)
    I_array = np.array(I)
    lastPoint = [1,1];
    flag = 0;
    number = 0;
     #resolution = 30
    points = []
    reverseFlag = 0
    for i in range(0,len(I_array)):
        if(math.fabs(lastPoint[0]-i)<resolution/2):
            continue
        avaliablePointCount = 0
        firstIndex = -1
	foundflag = 0
        for j in range(0,len(I_array[i])):
            if(I_array[i][j]>=250):
                avaliablePointCount = avaliablePointCount + 1
                if(firstIndex == -1):
                    firstIndex = j
        if(avaliablePointCount<resolution*2.5):
            continue
	if(firstIndex == -1):
	    firstIndex = 0
	#print(i)
	if(reverseFlag == 0):
	    rang = range(firstIndex,len(I_array[i]))
	else:
	    rang = range(len(I_array[i])-1,firstIndex-1,-1)
        for j in rang:
            #flagBarrier = 0;
            #for u in range(math.floor(i-resolution/8),math.floor(i+resolution/8)):
            #    if(u<0 or u>=len(I_array)):
            #        continue
            #    for v in range(math.floor(j-resolution/8),math.floor(j+resolution/8)):
            #        if(v<0 or v>=len(I_array[u])):
            #            continue
            #        if(I_array[u][v]<250):
            #            flagBarrier = flagBarrier + 1
            if(I_array[i][j]>=250):#and flagBarrier<=resolution/16*0.2
		if(foundflag == 0):
			foundflag = 1
			reverseFlag = not reverseFlag
		print([i,j])
		#print(flag)
                if(flag == 0):
                    lastPoint = [i,j]
                    flag = 1
		    #print(lastPoint)
                    #points.append([i,j])
	 	    #respoint = [origin[0]+j*mapresolution,origin[1]+i*mapresolution]
		    #print(respoint)
		    print([i,j])
                    yield [origin[0] + j * mapresolution, origin[1] + (len(I_array)-i) * mapresolution]
                    number = number + 1
                else:
                    if(distance([i,j],lastPoint)>=resolution):
                        #points.append([i,j])
			#print([origin[0]+j*mapresolution,origin[1]+i*mapresolution])
			print([i,j])
                        yield [origin[0] + j * mapresolution, origin[1] + (len(I_array)-i) * mapresolution]
                        lastPoint = [i,j];
                        number = number + 1
