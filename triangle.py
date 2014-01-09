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

shpname = 'd:/data/annooptimize/Annodata/'
pointfeaturepath = 'd:/data/annooptimize/Annodata/supermarket.shp'
sym = 4 #mm
labeldistance = 3 # mm
labelsize = [15,10] #width,height
clen = float(25) / 1000 * 10000# mm sqrt((sym/2 + labeldistance + labelsize[0])*(sym/2 + labeldistance + labelsize[0]) + (sym/2 + labeldistance + labelsize[1])*(sym/2 + labeldistance + labelsize[1]))
dx = 100 # (sym / 2 + labelsize[0] + labeldistance) / 2 / 1000 * 10000
dy = 75
dclen = 125 # clen/2


def genShp(linelist,edges,name):
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('pnumber','C','40')
    for i,line in enumerate(linelist):
        w.line(parts=[line])
        w.record(str(edges[i][0])+' , '+str(edges[i][1]))
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

def gensubpoint(pi):
    pi1 = [pi[0]+dx,pi[1]+dy]
    pi2 = [pi[0]-dx,pi[1]+dy]
    pi3 = [pi[0]-dx,pi[1]-dy]
    pi4 = [pi[0]+dx,pi[1]-dy]
    return list([pi1,pi2,pi3,pi4])

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

def genSolve(expoints,points):
    solve = np.zeros((len(points),4),np.int32)
    extri = Delaunay(expoints)
    conflictg = conflictgraph(expoints,extri,250)
    return conflictg,extri,solve

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
    return acesubg,visited

def randomSolve(cfg,sol,ps):
    for i in range(len(ps)):
        123


    