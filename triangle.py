'''
Author: octeufer
2014/1/7
'''
 
import numpy as np
import scipy as sp
from scipy.spatial import Delaunay
import math
import shapefile

shpname = 'd:/data/annooptimize/Annodata/triedges'
pointfeaturepath = 'd:/data/annooptimize/Annodata/supermarket.shp'


def genShp(linelist):
    w = shapefile.Writer(shapefile.POLYLINE)
    w.field('FID','C','40')
    for i,line in enumerate(linelist):
        w.line(parts=[line])
        w.record('FID',str(i))
    w.save(shpname)

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


    