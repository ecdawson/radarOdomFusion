import csv
import numpy as np
import math



def separate_IDs(data,ID_A,ID_B):
    radarA = []
    radarB = []
    for i in range(len(data[:,1])):
        if data[i,1] == ID_A:
            radarA.append(data[i,:])
        elif data[i,1] == ID_B:
            radarB.append(data[i,:])

    radarA = np.array(radarA)
    radarB = np.array(radarB)

    for i in range(len(radarA[:,6])):
        radarA[i,6] = math.radians(radarA[i,6])

    for i in range(len(radarB[:,6])):
        radarB[i,6] = math.radians(radarB[i,6])
    return radarA,radarB


def calc_target_positions(data,T):
    #caculates x and y target positions using range and angle, then transforms them from sensor frame to vehicle frame
    #appends points to data array
    x_pos = data[:,4]*np.cos(data[:,6])
    y_pos = -data[:,4]*np.sin(data[:,6])

    points = np.column_stack((x_pos,y_pos,np.zeros(len(x_pos))))

    body = []
    for i in range(len(x_pos)):
        body.append(T@points[i,:])
    body = np.array(body)
    rad = np.column_stack((data,body))

    
    return rad
     
def time_structure(data):
    #restructures data so that column 1 is time stamp and column 2 is all corresponding detections and information, making
    #each scan accessible through it's timestamp
    k = 0
    dat = []
    firstTime= data[0,17]
    lastTime = data[len(data[:,1])-1,17]

    cur_time = firstTime
    window = firstTime
    k = 0
    while cur_time < lastTime:
        scan = []
        while window == cur_time:
            scan.append(data[k,:])
            k = k + 1
            window = data[k,17]

        dat.append([data[k-1,17],scan])
        cur_time = data[k,17]

    dat = np.array(dat)
    return(dat)
