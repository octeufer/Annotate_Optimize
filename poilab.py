import sys
import numpy as np
sys.path.append("d:/data/annooptimize")
import triangle
import time

tinternal = list()

def labstart():
    points,tri = triangle.gentri("d:/data/annooptimize/Annodata/200600/poise.shp")
    plabels = triangle.dynamicSize(points)
    conflictg = triangle.conflictgraphdy(points,tri,plabels)
    acg = triangle.accesssubg(conflictg)
    len(acg)
    allsolve = np.zeros((len(points),4,2),np.float64)
    
    points2,tri2 = triangle.gentri("d:/data/annooptimize/Annodata/200600/POIhalf.shp")
    plabels2 = triangle.dynamicSize(points2)
    conflictg2 = triangle.conflictgraphdy(points2,tri2,plabels2)
    acg2 = triangle.accesssubg(conflictg2)
    
    points3,tri3 = triangle.gentri("d:/data/annooptimize/Annodata/200600/POIall.shp")
    plabels3 = triangle.dynamicSize(points3)
    conflictg3 = triangle.conflictgraphdy(points3,tri3,plabels3)
    acg3 = triangle.accesssubg(conflictg3)
    
    time.clock()
    costs,tabucs= triangle.globaltabuiter2dy(acg,points,1,plabels)
    tinternal.append(time.clock())
    costs2,tabucs2= triangle.globaltabuiter2dy(acg2,points2,1,plabels2)
    tinternal.append(time.clock())
    costs3,tabucs3= triangle.globaltabuiter2dy(acg3,points3,1,plabels3)
    tinternal.append(time.clock())
    return tinternal,(costs,tabucs),(costs2,tabucs2),(costs3,tabucs3)
