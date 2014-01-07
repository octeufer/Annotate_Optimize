'''
Author: octeufer
2014/1/7
'''
 
import numpy as np
import scipy as sp
import math
import shapefile

shpname = 'd:/data/annooptimize/Annodata/triedges'

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

