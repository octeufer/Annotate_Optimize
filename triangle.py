'''
Author: octeufer
2014/1/7
'''
 
import numpy as np
import scipy as sp
from scipy.spatial import Delaunay
import math
import shapefile
import random
import mcpAlgorithm
import time
from ctypes import *

shpname = 'd:/data/annooptimize/Annodata/'
pointfeaturepath = 'd:/data/annooptimize/Annodata/supermarket.shp'
roadfeaturepath = 'd:/data/annooptimize/Annodata/200600/roadtet.shp'
sym = 4 #mm
labeldistance = 3 # mm
labelsize = [15,10] #width,height
clen = float(19) / 1000 * 10000# mm sqrt((sym/2 + labeldistance + labelsize[0])*(sym/2 + labeldistance + labelsize[0]) + (sym/2 + labeldistance + labelsize[1])*(sym/2 + labeldistance + labelsize[1]))
dx = 67.5# (sym / 2 + labelsize[0] + labeldistance) / 2 / 1000 * 10000
dy = 22.5
dclen = 142.5 # clen/2
countbbb = 0

def genShp(linelist,edges,name):
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('pnumber','C','40')
    for i,line in enumerate(linelist):
        w.line(parts=[line])
        w.record(str(edges[i][0])+' , '+str(edges[i][1]))
    w.save(shpname + name)

def genSegs(segs,name):
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('segnumber','C','40')
    for i,seg in enumerate(segs):
        w.line(parts=[list(seg)])
        w.record(str(i))
    w.save(shpname + name)

def genPshp(plist,points,name):
    w = shapefile.Writer(shapefile.POINT)
    w.field('player','C','40')
    w.field('pnum','C','40')    
    for i in range(len(plist)):
        for j in range(len(plist[i])):
            w.point(points[plist[i][j]][0],points[plist[i][j]][1])
            w.record(player=str(i),pnum=str(plist[i][j]))
    w.save(shpname + name)

def genEXPshp(psub,expoints,name):
    w = shapefile.Writer(shapefile.POINT)
    w.field('pnum','C','40')
    w.field('porinum','C','40')
    for i,point in enumerate(expoints):
        w.point(point[0],point[1])
        w.record(pnum=str(i % 4),porinum=str(psub[i / 4]))
    w.save(shpname + name)

def genSolveshp(allsolve,name):
    w = shapefile.Writer(shapefile.POINT)
    w.field('pnum','C','40')
    w.field('annonum','C','40')
    for i,solve in enumerate(allsolve):
        for j in range(4):
            if solve[j][0]>0:
                w.point(solve[j][0],solve[j][1])
                w.record(pnum=str(i),annonum=str(j))
    w.save(shpname + name)

def rectp(rpoint,height,width):
    height = float(height) / 1000 * 10000
    width = float(width) / 1000 * 10000
    dx = width/2
    dy = height/2
    minx = rpoint[0]-dx
    miny = rpoint[1]-dy
    maxx = rpoint[0]+dx
    maxy = rpoint[1]+dy
    return [[[minx,maxy],[minx,miny],[maxx,miny],[maxx,maxy],[minx,maxy]]]

def genPolySolveshp(allsolve,name):
    w = shapefile.Writer(shapefile.POLYGON)
    w.field('pnum','C','40')
    w.field('annonum','C','40')
    for i,solve in enumerate(allsolve):
        for j in range(4):
            if solve[j][0]>0:
                w.poly(parts=rectp([solve[j][0],solve[j][1]],4.5,13.5))
                w.record(pnum=str(i),annonum=str(j))
    w.save(shpname + name)

def edgeselection(linelist):
    linec = []
    for i,line in enumerate(linelist):
       if caldis(line) > 1000:
               continue
       linec.append(line)
    return linec

def gentri(pfpath):
    ps=[]
    reader = shapefile.Reader(pfpath)    
    for sr in reader.shapeRecords():
        ps.append(list(sr.shape.__geo_interface__["coordinates"]))
    points = np.array(ps)
    tri = Delaunay(points)
    return points,tri

def gentriedges(tri):
    tris = tri.points[tri.simplices]
    A = tris[:,0,:]
    B = tris[:,1,:]
    C = tris[:,2,:]
    edges = []
    edges.extend(zip(A,B))
    edges.extend(zip(B,C))
    edges.extend(zip(C,A))
    return edges

def find_neighbors(pindex,triang):
    neighborsl = list()
    for simplex in triang.simplices:
            if pindex in simplex:
                    neighborsl.extend([simplex[i] for i in range(len(simplex)) if simplex[i] != pindex])
    return list(set(neighborsl))

def pgraphconstruct(points,tri):
    g = list()
    for i in range(len(points)):
        link = list()
        ne = find_neighbors(i,tri)
        for j in range(len(points)):
                if j in ne:
                        link.append(1)
                else:
                        link.append(0)
        g.append(link)
    return g

def pgraphcons2(points,tri):
    g = np.zeros((len(points),len(points)),np.int32)
    for i in range(len(points)):
        ne = find_neighbors(i,tri)
        #link = np.zeros((len(points),len(points)),np.int32)
        for num in ne:
            g[i,num] = 1
        #g.append(link)
    return g

def conflictgraph(points,tri,disc):
    g = np.zeros((len(points),len(points)),np.int32)
    #subgraph = list()
    for i in range(len(points)):
        ne = find_neighbors(i,tri)
        #link = np.zeros((len(points),len(points)),np.int32)
        for num in ne:
            if caldis(i,num,points) < disc:
                g[i,num] = 1
                #subgraph.extend([i,num])
        #g.append(link)
    return g#,np.array(list(set(np.array(subgraph).tolist())))

def caldis(p1,p2,points):
    x0 = points[p1][0]
    y0 = points[p1][1]
    x1 = points[p2][0]
    y1 = points[p2][1]
    pdis = math.sqrt(math.pow((x0-x1), 2) + math.pow((y0-y1),2))
    return pdis

def drawgraph(pgraph,points):
    lines = list()
    edges = list()
    for i in range(len(points)):
        for j in range(len(points)):
            if pgraph[i,j]==1:
                lines.append([[points[i][0],points[i][1]],[points[j][0],points[j][1]]])
                edges.append([i,j])
    return lines,edges

def drawgraph2(pgraph,points):
    lines = list()
    edges = list()
    for i in range(len(points)):
        for j in range(len(points)):
            if pgraph[i,j]==1:
                lines.append([[points[i].coor[0],points[i].coor[1]],[points[j].coor[0],points[j].coor[1]]])
                edges.append([i,j])
    return lines,edges

def gensubpoint(pi):
    pi1 = [pi[0]+dx,pi[1]+dy]
    pi2 = [pi[0]-dx,pi[1]+dy]
    pi3 = [pi[0]-dx,pi[1]-dy]
    pi4 = [pi[0]+dx,pi[1]-dy]
    return list([pi1,pi2,pi3,pi4])
     

#extend points
def gensubcfg(spoints):
    expandps = []
    for i in range(len(spoints)):
        expandps.extend(gensubpoint(spoints[i]))
    expoints = np.array(expandps)
    return expoints     

def genLayerConstruct(points):
    lenv = [points[:,0].min(),points[:,1].min(),points[:,0].max(),points[:,1].max()]
    midp = [(lenv[0] + lenv[2])/2,(lenv[1] + lenv[3])/2]
    trip = Delaunay(points)
    intmidtri = trip.find_simplex(np.array(midp))
    simplex = trip.simplices[intmidtri]
    plist = list()
    interplist = list()
    aplist = list()
    nblist = simplex.tolist()
    t = len(points)
    while t>0:
        interplist = []
        outplist = []
        #aplist = []
        for p in nblist: 
            interplist.extend([i for i in find_neighbors(p,trip) if i not in interplist and i not in nblist and i not in aplist])
        interplist = list(set(interplist))
        '''
        for p in nblist:
            if interplist.count(p) > 0:
                interplist.remove(p)
        '''
        for p in interplist:
            outplist.extend([i for i in find_neighbors(p,trip) if i not in outplist and i not in interplist and i not in nblist])
        outplist = list(set(outplist))
        '''
        for p in interplist:
            if outplist.count(p) > 0:
                outplist.remove(p)
        '''
        aplist =  interplist + outplist
        plist.append(aplist)
        nblist = outplist
        t=t-len(aplist)
        if (len(plist[len(plist)-1])) > t:
            plist[0] = plist[0] + simplex.tolist()
            lalist = list()
            for i in range(len(plist)):
                lalist.extend(plist[i])
            plist.append([i for i in range(len(points)) if i not in lalist])
            break
    return plist

def genSolve(expoints):
    #solve = np.zeros((len(points),4),np.int32)
    extri = Delaunay(expoints)
    conflictg = conflictgraph(expoints,extri,142.5)
    return conflictg,extri#,solve

def accesssubg(conflictg):
    acesubg = list()
    visited = np.zeros(conflictg.shape[0],np.int32)
    def BFS(conflictg,v):
        order=list()
        queue=list()
        order.append(v)
        queue.append(v)
        visited[v]=1
        while len(queue)>0:
            v=queue.pop(0)
            if (conflictg[v]>0).sum()>0:
                for i in range(conflictg.shape[0]):
                    if conflictg[v][i] > 0 and visited[i]==0:
                        queue.append(i)
                        order.append(i)
                        visited[i]=1
        acesubg.append(order)
    for i in range(conflictg.shape[0]):
        if visited[i]==0:
            BFS(conflictg,i)
    return acesubg

def solvegenerate(accesssubg,points,allsolve):
    isolate = list()
    subsolves = list()
    #solvepoints = np.zeros((points.shape[0],2),np.float64)
    #greedym = mcpAlgorithm.greedymcp()
             
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            bestsubsolve,subexpoints = gensubsolve(subg,points)
            for solve in bestsubsolve:
                ni = subg[solve / 4]
                pi = solve % 4
                #solvepoints[ni] = subexpoints[solve]
                allsolve[ni,pi] = subexpoints[solve]
            subsolves.append(bestsubsolve)
    #isolatesolve = genisolatesol(isolate,points)
    isolatesolve = prefisosol(isolate,points)
    for sol in isolatesolve:
        allsolve[sol[0],sol[1]] = sol[2]
    print "success!"
    return isolate,subsolves

def gensubsolve(subg,points):
    subexpoints = gensubcfg(points[subg])
    subconflictgraph,subextri = genSolve(subexpoints)
    
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if subconflictgraph[i,j]==0 and caldis(i,j,subexpoints)<142.5:
                subconflictgraph[i,j]=1  
    for i in range(subconflictgraph.shape[0]):
        for j in range(i - i % 4,i + 4 - (i % 4)):
            if i==j: continue
            subconflictgraph[i,j] = 1
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
        
    #greedym = mcpAlgorithm.greedymcp()
    #bestsubsolve = greedym.FindMaxClique(subgraphcomplement,200,subgraphcomplement.shape[0])
    tabum = mcpAlgorithm.tabumcp(subgraphcomplement)
    bestsubsolve = tabum.FindMaxClique(subgraphcomplement,50,subgraphcomplement.shape[0])
    return bestsubsolve,subexpoints

def genisolatesol(isolate,points):
    
    sols = list()
    for i in range(len(isolate)):
        exps = gensubpoint(points[isolate[i]])
        pi = random.randint(0,3)
        ps = exps[pi]
        sols.append([isolate[i],pi,ps])
    return sols

def prefisosol(isolate,points):
    sols = list()
    for i in range(len(isolate)):
        exps = gensubpoint(points[isolate[i]])
        pi = 0
        ps = exps[pi]
        sols.append([isolate[i],pi,ps])
    return sols
        
def costac(allsolve):
    cost = 0
    for i,solve in enumerate(allsolve):
        for j in range(4):
            if solve[j][0]>0:
                cost = cost + j * 0.0003
    return cost

def gentabuclass(subg,points):
    subexpoints = gensubcfg(points[subg])
    subconflictgraph,subextri = genSolve(subexpoints)
    
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if subconflictgraph[i,j]==0 and caldis(i,j,subexpoints)<142.5:
                subconflictgraph[i,j]=1  
    for i in range(subconflictgraph.shape[0]):
        for j in range(i - i % 4,i + 4 - (i % 4)):
            if i==j: continue
            subconflictgraph[i,j] = 1
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
        
    #greedym = mcpAlgorithm.greedymcp()
    #bestsubsolve = greedym.FindMaxClique(subgraphcomplement,200,subgraphcomplement.shape[0])
    tabuclass = mcpAlgorithm.tabumcp(subgraphcomplement)
    #bestsubsolve = tabum.FindMaxClique(subgraphcomplement,50,subgraphcomplement.shape[0])
    return (tabuclass,subexpoints,subg,subgraphcomplement)

def globaltabuiter(accesssubg,points,maxiter):
    isolate = list()
    tabucs = list()
    t=0
    costs = dict()
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            tabuclasstuple = gentabuclass(subg,points)
            tabucs.append(tabuclasstuple)
    #isolatesolve = genisolatesol(isolate,points)
    time.clock()
    while t<maxiter:
        t=t+1
        asol = np.zeros((len(points),4,2),np.float64)
        for tc in tabucs:
            bestsubsolve = tc[0].FindMaxClique(10000)
            for solve in bestsubsolve:
                ni = tc[2][solve / 4]
                pi = solve % 4
                #solvepoints[ni] = subexpoints[solve]
                asol[ni,pi] = tc[1][solve]
        isolatesolve = prefisosol(isolate,points)
        for sol in isolatesolve:
            asol[sol[0],sol[1]] = sol[2]
        costs[costac(asol)]=asol
        print "success!time=%s" %time.clock()
    return costs,tabucs

class paras(Structure):
    _fields_ = [("asize",c_int),("adjlist",POINTER(POINTER(c_bool)))]
    
class CliqueData(Structure):
    _fields_ = [("qmax",POINTER(c_int)),("qsize",c_int)]

def mcqdcs(accesssubg,points,allsolve):
    getclique = CDLL("d:/data/annooptimize/test/dytest/Project1.dll")
    isolate = list()
    #subsolves = list()
    
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            bestsubsolve,subexpoints = mcqdsubsol(subg,points,getclique)
            '''
            for i in range(bsubsolsize):
                ni = subg[bestsubsolve[i] / 4]
                pi = bestsubsolve[i] % 4
                #solvepoints[ni] = subexpoints[solve]
                allsolve[ni,pi] = subexpoints[bestsubsolve[i]]
            '''
            for solve in bestsubsolve:
                ni = subg[solve / 4]
                pi = solve % 4
                #solvepoints[ni] = subexpoints[solve]
                allsolve[ni,pi] = subexpoints[solve]
            #subsolves.append(bestsubsolve)
    #isolatesolve = genisolatesol(isolate,points)
    isolatesolve = prefisosol(isolate,points)
    for sol in isolatesolve:
        allsolve[sol[0],sol[1]] = sol[2]
    cost = costac(allsolve)
    print "success!"
    return cost,allsolve

def mcqdsubsol(subg,points,getclique):
    subexpoints = gensubcfg(points[subg])
    subconflictgraph,subextri = genSolve(subexpoints)
    
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if subconflictgraph[i,j]==0 and caldis(i,j,subexpoints)<142.5:
                subconflictgraph[i,j]=1  
    for i in range(subconflictgraph.shape[0]):
        for j in range(i - i % 4,i + 4 - (i % 4)):
            if i==j: continue
            subconflictgraph[i,j] = 1
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
    print subgraphcomplement.shape
    tcount = raw_input("input a time to sleep: ")
    time.sleep(int(tcount))
    x = paras(subgraphcomplement.shape[0],(POINTER(c_bool) * subgraphcomplement.shape[0])())
    for i in range(subgraphcomplement.shape[0]):
        x.adjlist[i] = (c_bool * subgraphcomplement.shape[0])()
    for i in range(x.asize):
        for j in range(x.asize):
            x.adjlist[i][j] = (subgraphcomplement[i][j]==1)
    getclique.getClique.argtypes = [POINTER(POINTER(c_bool)),c_int]
    getclique.getClique.restype = CliqueData
    print "done"
    cliquedata = getclique.getClique(x.adjlist,x.asize)
    bestsubsolve = [cliquedata.qmax[i] for i in range(cliquedata.qsize)]
    #bestsubsolve = tabum.FindMaxClique(subgraphcomplement,50,subgraphcomplement.shape[0])
    return bestsubsolve,subexpoints

'''
line cross consider algorithm
2014/3/9 
'''

def crossornot(line1,line2):
    d1 = crossf(line1[0],line1[1],line1[0],line2[0])
    d2 = crossf(line1[0],line1[1],line1[0],line2[1])
    d3 = crossf(line2[0],line2[1],line2[0],line1[0])
    d4 = crossf(line2[0],line2[1],line2[0],line1[1])
    if d1*d2 < 0 and d3*d4 < 0:
        return True
    else:
        return False

def crossf(p1,p2,q1,q2):
    x1 = p2[0] - p1[0]
    y1 = p2[1] - p1[1]
    x2 = q2[0] - q1[0]
    y2 = q2[1] - q1[1]
    return x1*y2 - x2*y1

def getsegs(roadpath):
    segs=list()
    reader = shapefile.Reader(roadpath)    
    for sr in reader.shapeRecords():
        segs.extend([(sr.shape.__geo_interface__["coordinates"][i],sr.shape.__geo_interface__["coordinates"][i+1]) for i in range(len(sr.shape.__geo_interface__["coordinates"])-1)])
    npsegs = np.array(segs)
    return segs,npsegs

def psnapseg(p,segs):
    nearestseg = tuple()
    nearest = float(1000000000000000)
    for seg in segs:
        xmid = (float(seg[0][0]) + float(seg[1][0])) / 2
        ymid = (float(seg[0][1]) + float(seg[1][1])) / 2
        dis = math.sqrt(math.pow((p[0]-xmid), 2) + math.pow((p[1]-ymid),2))
        #print nearest,dis
        if dis < nearest:
            nearest = dis
            nearestseg = seg
    return nearestseg

'''
road condition 2014/3/9 
OO extend points method 
author by : octeufer leluch
'''
class SubPoint:
    def __init__(self,pnum,subpnum,coor):
        self.point = pnum
        self.subpoint = subpnum
        self.coor = coor

def subextend(spoints,roadfeaturepath):
    expandps = list()
    segs,npsegs = getsegs(roadfeaturepath)
    for i in range(len(spoints)):
        psubs = gensubpoint(spoints[i])
        nearestseg = psnapseg(spoints[i],segs)
        for j in range(len(psubs)):
            if not crossornot((spoints[i],psubs[j]),nearestseg):
                subpoint = SubPoint(i,j,psubs[j])
                expandps.append(subpoint)
    expoints = np.array(expandps)
    return expoints

'''
cell
'''
def subextend2(spoints,roadfeaturepath):
    expandps = list()
    segs,npsegs = getsegs(roadfeaturepath)
    cells = seg2cell(segs,120)
    for i in range(len(spoints)):
        psubs = gensubpoint(spoints[i])
        #nearestseg = psnapseg(spoints[i],segs)
        nearestcell = psnapcell(spoints[i],cells)
        for j in range(len(psubs)):
            if not crossornot((spoints[i],psubs[j]),nearestcell):
                subpoint = SubPoint(i,j,psubs[j])
                expandps.append(subpoint)
    expoints = np.array(expandps)
    return expoints

def caldis2(p1,p2,points):
    x0 = points[p1].coor[0]
    y0 = points[p1].coor[1]
    x1 = points[p2].coor[0]
    y1 = points[p2].coor[1]
    pdis = math.sqrt(math.pow((x0-x1), 2) + math.pow((y0-y1),2))
    return pdis

def gensubsolve2(subg,points):
    #subexpoints = subextend(points[subg],roadfeaturepath)
    subexpoints = subextend2(points[subg],roadfeaturepath)
    #subconflictgraph,subextri = genSolve(subexpoints)
    subconflictgraph = np.zeros((len(subexpoints),len(subexpoints)),np.int32)
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if caldis2(i,j,subexpoints)<142.5 :
                subconflictgraph[i,j]=1
            if subexpoints[i].point == subexpoints[j].point and i!=j:
                subconflictgraph[i,j]=1  
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
        
    #greedym = mcpAlgorithm.greedymcp()
    #bestsubsolve = greedym.FindMaxClique(subgraphcomplement,200,subgraphcomplement.shape[0])
    tabuclass = mcpAlgorithm.tabumcp(subgraphcomplement)
    return (tabuclass,subexpoints,subg,subgraphcomplement,subconflictgraph)

def globaltabuiter2(accesssubg,points,maxiter):
    isolate = list()
    tabucs = list()
    t=0
    costs = dict()
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            tabuclasstuple = gensubsolve2(subg,points)
            tabucs.append(tabuclasstuple)
    #isolatesolve = genisolatesol(isolate,points)
    time.clock()
    while t<maxiter:
        t=t+1
        asol = np.zeros((len(points),4,2),np.float64)
        for tc in tabucs:
            bestsubsolve = tc[0].FindMaxClique(500)
            for solve in bestsubsolve:
                ni = tc[2][tc[1][solve].point]
                pi = tc[1][solve].subpoint
                #solvepoints[ni] = subexpoints[solve]
                asol[ni,pi] = tc[1][solve].coor
        isolatesolve = prefisosol2(isolate,points)
        for sol in isolatesolve:
            asol[sol[0],sol[1]] = sol[2]
        costs[costac(asol)]=asol
        print "success!time=%s" %time.clock()
    return costs,tabucs

def prefisosol2(isolate,points):
    sols = list()
    #exps = subextend(points[isolate],roadfeaturepath)
    exps = subextend2(points[isolate],roadfeaturepath)
    exsols = dict()
    for exp in exps:
        if exp.point in exsols.keys():
            exsols[exp.point].append(exp)
        else:
            exsols[exp.point]=list([exp])
    for v in exsols.values():
        p = v[random.randint(0,len(v)-1)]
        sols.append([isolate[p.point],p.subpoint,p.coor])
    return sols

'''
seg2cell python edition
author octeufer leluch
2014/3/14
'''
class Cell:
    def __init__(self,points,segnum):
        self.points = points
        self.segnum = segnum

def seg2cell(segs,gapdis):
    cells = list()
    for i,seg in enumerate(segs):
        cellpoints = insertPt(seg,gapdis)
        for j in range(len(cellpoints)-1):
            cell = Cell((cellpoints[j],cellpoints[j+1]),i)
            cells.append(cell)
    return cells

def disPtoP(p1,p2):
    x0 = p1[0]
    x1 = p2[0]
    y0 = p1[1]
    y1 = p2[1]
    pdis = math.sqrt(math.pow((x0-x1), 2) + math.pow((y0-y1),2))
    return pdis

def insertPt(seg,gapdis):
    agary = [seg[i+1] for i in range(len(seg)-1)]
    agary.reverse()
    resultary = list([seg[0]])
    #print agary,resultary
    while len(agary)!=0:
        #print agary,resultary
        resultNum = len(resultary)
        pt1 = resultary[resultNum-1]
        agNum = len(agary)
        pt2 = agary[agNum-1]
        '''
        print agary
        print resultary
        print pt1
        print pt2
        print disPtoP(pt1,pt2)
        tcount = raw_input("input a time to sleep: ")
        time.sleep(int(tcount))
        '''
        if disPtoP(pt1,pt2) > gapdis:
            pt3 = (((pt1[0]+pt2[0]) / 2),((pt1[1]+pt2[1]) / 2))
            agary.append(pt3)
        else:
            resultary.append(pt2)
            agary.pop(agNum-1)
    return resultary

def psnapcell(p,cells):
    nearestcell = tuple()
    nearest = float(1000000000000000)
    for cell in cells:
        xmid = (float(cell.points[0][0]) + float(cell.points[1][0])) / 2
        ymid = (float(cell.points[0][1]) + float(cell.points[1][1])) / 2
        dis = math.sqrt(math.pow((p[0]-xmid), 2) + math.pow((p[1]-ymid),2))
        #print nearest,dis
        if dis < nearest:
            nearest = dis
            nearestcell = cell.points
    return nearestcell


'''
road mcqd
author by: octeufer leluch
2014/3/14
'''
def mcqdcs2(accesssubg,points,allsolve):
    getclique = CDLL("d:/data/annooptimize/test/dytest/Project1.dll")
    isolate = list()
    #subsolves = list()
    
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            bestsubsolve,subexpoints = mcqdsubsol2(subg,points,getclique)
            '''
            for i in range(bsubsolsize):
                ni = subg[bestsubsolve[i] / 4]
                pi = bestsubsolve[i] % 4
                #solvepoints[ni] = subexpoints[solve]
                allsolve[ni,pi] = subexpoints[bestsubsolve[i]]
            '''
            for solve in bestsubsolve:
                ni = subg[subexpoints[solve].point]
                pi = subexpoints[solve].subpoint
                #solvepoints[ni] = subexpoints[solve]
                allsolve[ni,pi] = subexpoints[solve].coor
            #subsolves.append(bestsubsolve)
    #isolatesolve = genisolatesol(isolate,points)
    isolatesolve = prefisosol2(isolate,points)
    for sol in isolatesolve:
        allsolve[sol[0],sol[1]] = sol[2]
    cost = costac(allsolve)
    print "success!"
    return cost,allsolve

def mcqdsubsol2(subg,points,getclique):
    subexpoints = subextend2(points[subg],roadfeaturepath)
    #subconflictgraph,subextri = genSolve(subexpoints)
    subconflictgraph = np.zeros((len(subexpoints),len(subexpoints)),np.int32)
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if caldis2(i,j,subexpoints)<142.5 :
                subconflictgraph[i,j]=1
            if subexpoints[i].point == subexpoints[j].point and i!=j:
                subconflictgraph[i,j]=1  
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
    x = paras(subgraphcomplement.shape[0],(POINTER(c_bool) * subgraphcomplement.shape[0])())
    for i in range(subgraphcomplement.shape[0]):
        x.adjlist[i] = (c_bool * subgraphcomplement.shape[0])()
    for i in range(x.asize):
        for j in range(x.asize):
            x.adjlist[i][j] = (subgraphcomplement[i][j]==1)
    getclique.getClique.argtypes = [POINTER(POINTER(c_bool)),c_int]
    getclique.getClique.restype = CliqueData
    cliquedata = getclique.getClique(x.adjlist,x.asize)
    bestsubsolve = [cliquedata.qmax[i] for i in range(cliquedata.qsize)]
    #bestsubsolve = tabum.FindMaxClique(subgraphcomplement,50,subgraphcomplement.shape[0])
    return bestsubsolve,subexpoints


'''
dynamic size
author by: octeufer leluch
2014/3/1
'''
class LabelPara:
    def __init__(self,widn,hein,ll):
        self.width = widn
        self.height = hein
        self.la = ll

def dynamicSize(points):
    plabels = list()
    for i in range(len(points)):
        w = random.randint(3,5)
        h = random.randint(1,2)
        width = w * 40
        height = h * 40
        longla = math.sqrt(width*width + height*height)
        labelpara = LabelPara(width,height,longla)
        plabels.append(labelpara)
    return plabels

def conflictgraphdy(points,tri,plabels):
    g = np.zeros((len(points),len(points)),np.int32)
    #subgraph = list()
    for i in range(len(points)):
        ne = find_neighbors(i,tri)
        #link = np.zeros((len(points),len(points)),np.int32)
        for num in ne:
            if caldis(i,num,points) < (plabels[i].la + plabels[num].la):
                g[i,num] = 1
                #subgraph.extend([i,num])
        #g.append(link)
    return g

def gensubpointdy(pi,i,plabels):
    pi1 = [pi[0]+(float(plabels[i].width) / 2),pi[1]+(float(plabels[i].height) / 2)]
    pi2 = [pi[0]-(float(plabels[i].width) / 2),pi[1]+(float(plabels[i].height) / 2)]
    pi3 = [pi[0]-(float(plabels[i].width) / 2),pi[1]-(float(plabels[i].height) / 2)]
    pi4 = [pi[0]+(float(plabels[i].width) / 2),pi[1]-(float(plabels[i].height) / 2)]
    return list([pi1,pi2,pi3,pi4])

def genPolySolveshpdy(allsolve,plabels,name):
    w = shapefile.Writer(shapefile.POLYGON)
    w.field('pnum','C','40')
    w.field('annonum','C','40')
    for i,solve in enumerate(allsolve):
        for j in range(4):
            if solve[j][0]>0:
                w.poly(parts=rectp([solve[j][0],solve[j][1]],float(plabels[i].height)/10,float(plabels[i].width)/10))
                w.record(pnum=str(i),annonum=str(j))
    w.save(shpname + name)
    
def subextend2dy(spoints,roadfeaturepath,plabels,subg):
    expandps = list()
    segs,npsegs = getsegs(roadfeaturepath)
    cells = seg2cell(segs,120)
    for i in range(len(spoints)):
        psubs = gensubpointdy(spoints[i],subg[i],plabels)
        #nearestseg = psnapseg(spoints[i],segs)
        nearestcell = psnapcell(spoints[i],cells)
        for j in range(len(psubs)):
            if not crossornot((spoints[i],psubs[j]),nearestcell):
                subpoint = SubPoint(i,j,psubs[j])
                expandps.append(subpoint)
    expoints = np.array(expandps)
    return expoints

def gensubsolve2dy(subg,points,plabels):
    #subexpoints = subextend(points[subg],roadfeaturepath)
    subexpoints = subextend2dy(points[subg],roadfeaturepath,plabels,subg)
    #subconflictgraph,subextri = genSolve(subexpoints)
    subconflictgraph = np.zeros((len(subexpoints),len(subexpoints)),np.int32)
    for i in range(subconflictgraph.shape[0]):
        for j in range(subconflictgraph.shape[0]):
            if caldis2(i,j,subexpoints)<(plabels[subg[subexpoints[i].point]].la + plabels[subg[subexpoints[j].point]].la)/2 :
                subconflictgraph[i,j]=1
            if subexpoints[i].point == subexpoints[j].point and i!=j:
                subconflictgraph[i,j]=1  
    subgraphcomplement = np.where(subconflictgraph>0,0,1)
    for i in range(subgraphcomplement.shape[0]):
        subgraphcomplement[i][i] = 0
        
    #greedym = mcpAlgorithm.greedymcp()
    #bestsubsolve = greedym.FindMaxClique(subgraphcomplement,200,subgraphcomplement.shape[0])
    tabuclass = mcpAlgorithm.tabumcp(subgraphcomplement)
    return (tabuclass,subexpoints,subg,subgraphcomplement,subconflictgraph)

def globaltabuiter2dy(accesssubg,points,maxiter,plabels):
    isolate = list()
    tabucs = list()
    t=0
    costs = dict()
    for subg in accesssubg:
        if(len(subg)==1):
            isolate.extend(subg)
        else:
            #print subg
            tabuclasstuple = gensubsolve2dy(subg,points,plabels)
            tabucs.append(tabuclasstuple)
    #isolatesolve = genisolatesol(isolate,points)
    time.clock()
    while t<maxiter:
        t=t+1
        asol = np.zeros((len(points),4,2),np.float64)
        for tc in tabucs:
            bestsubsolve = tc[0].FindMaxClique(500)
            for solve in bestsubsolve:
                ni = tc[2][tc[1][solve].point]
                pi = tc[1][solve].subpoint
                #solvepoints[ni] = subexpoints[solve]
                asol[ni,pi] = tc[1][solve].coor
        isolatesolve = prefisosol2dy(isolate,points,plabels)
        for sol in isolatesolve:
            asol[sol[0],sol[1]] = sol[2]
        costs[costac(asol)]=asol
        print "success!time=%s" %time.clock()
    return costs,tabucs

def prefisosol2dy(isolate,points,plabels):
    sols = list()
    #exps = subextend(points[isolate],roadfeaturepath)
    exps = subextend2dy(points[isolate],roadfeaturepath,plabels,isolate)
    exsols = dict()
    for exp in exps:
        if exp.point in exsols.keys():
            exsols[exp.point].append(exp)
        else:
            exsols[exp.point]=list([exp])
    for v in exsols.values():
        p = v[random.randint(0,len(v)-1)]
        sols.append([isolate[p.point],p.subpoint,p.coor])
    return sols
