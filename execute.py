'''
author : Octeufer
2013/11/30
'''

import sys
sys.path.append("d:/data/annooptimize")

#import AnnoOptmize
import optimizemodel
import triangle
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
points,tri = triangle.gentri(triangle.pointfeaturepath)
expoints = triangle.gensubcfg(points)
conflictg,extri,solve,plist = triangle.genSolve(expoints,points)