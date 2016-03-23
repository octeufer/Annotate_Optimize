'''
author : Octeufer
2013/11/30
'''

import sys
import numpy as np
sys.path.append("/Users/octeufer/Work/workspace/annooptimize")

#import AnnoOptmize
import optimizemodel
import triangle
import mcpAlgorithm
'''
#paras = [dbname,poishp,barriershp,annopoishp]
paras = ["tanno1130","supmakclip2","rannoclip2","supmakclip2"]

optimizem = optimizemodel.optimizemodel(paras[0])
'''
'''
optimizem.dataengine.insertapoi(paras[1])
optimizem.dataengine.insertbarriergeo(paras[2])
optimizem.dataengine.insertpbarriergeo(paras[3])
optimizem.domaingenerate(None)
'''
#a = optimizem.solgen()
'''
points,tri = triangle.gentri(triangle.pointfeaturepath)
expoints = triangle.gensubcfg(points)
conflictg,extri,solve,plist = triangle.genSolve(expoints,points)
'''
grapht = np.load("/Users/octeufer/Work/workspace/annooptimize/Annodata/graphdata/conflictgraph.npz.npy")
acgt = np.load("/Users/octeufer/Work/workspace/annooptimize/Annodata/graphdata/accesssubgraph.npz.npy")
points = np.load("/Users/octeufer/Work/workspace/annooptimize/Annodata/graphdata/Points.npz.npy")
allsolve = np.zeros((len(points),4,2),np.float64)
iss,subs = triangle.solvegenerate(acgt,points,allsolve)
triangle.genSolveshp(allsolve,"soll1.shp")
triangle.genPolySolveshp(allsolve,"sollpoly.shp")

