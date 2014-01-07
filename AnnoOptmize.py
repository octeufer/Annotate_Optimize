'''
    @author: Octeufer
    2013/11/27
'''

import json
import pymongo
from bson.son import SON
import shapefile

path = "d:/data/annooptimize/Annodata/"


def rectp(rpoint,height,width):
    height = float(height) / 1000 / 1000 * 10000
    width = float(width) / 1000 / 1000 * 10000
    dx = width/2
    dy = height/2
    minx = rpoint[0]-dx
    miny = rpoint[1]-dy
    maxx = rpoint[0]+dx
    maxy = rpoint[1]+dy
    return [[[minx,maxy],[minx,miny],[maxx,miny],[maxx,maxy],[minx,maxy]]]

class dataengine:

    def __init__(self,dbname):
        self.client = pymongo.MongoClient("localhost",27017)
        self.db = self.client[dbname]
        self.db.annopois.ensure_index([("geometry",pymongo.GEOSPHERE)])
        self.db.annopois.ensure_index([("uniqueid",pymongo.ASCENDING)])
        self.db.barriergeo.ensure_index([("geometry",pymongo.GEOSPHERE)])
        #self.db.barriergeo.ensure_index([("objid",pymongo.ASCENDING)])
    
    def annotodatacol(self,annolyrname):
        self.db.annolyrname.ensure_index([("geometry",pymongo.GEOSPHERE)])
        self.db.annolyrname.ensure_index([("linkobjid",pymongo.ASCENDING)])
        
    def insertapoi(self,shpname):
        
        reader = shapefile.Reader(path + shpname + ".shp")
        fields = reader.fields
        for sr in reader.shapeRecords():           
            annopoi = dict(type="Feature",attributes=dict(), \
                           uniqueid=int(float(sr.record[9])),layername=shpname, \
                           geometry=sr.shape.__geo_interface__)
            '''
            annopoi = {"type": "Feature", \
                       "geometry": { \
                                    "type": "Point", \
                                    "coordinates": list(sr.shape.__geo_interface__["coordinates"])}}
            print annopoi
            '''
            annopoi["geometry"]["coordinates"] = list((annopoi["geometry"]["coordinates"][0] / 1000,annopoi["geometry"]["coordinates"][1] / 1000))
            self.db.annopois.insert(annopoi)
    
    def insertbarriergeo(self,barriername):
        reader = shapefile.Reader(path + barriername + ".shp")
        for sr in reader.shapeRecords():
            barriergeo = dict(type="Feature",attributes=dict(), \
                              layername=barriername, \
                              geometry=sr.shape.__geo_interface__)
            barriergeo["geometry"]["coordinates"] = [list(list((t[0]/1000,t[1]/1000)) for t in barriergeo["geometry"]["coordinates"])]
            self.db.barriergeo.insert(barriergeo)
    
    def insertpbarriergeo(self,plyrname):
        for p in self.db.annopois.find({"layername":plyrname}):
            pgeo = p["geometry"]["coordinates"]
            polygeo = dict(type="Polygon",coordinates=rectp(pgeo,3,3))
            newba = dict(type="Feature",attributes=dict(), \
                         layername=plyrname, \
                         geometry=polygeo)
            print newba
            self.db.barriergeo.insert(newba)
    
    def insertlocgeo(self,annopoint):
        pgeo = annopoint.geometry["coordinates"]
        polygeo = dict(type="Polygon",coordinates=rectp(pgeo,3,15))
        locbarrier = barrier(polygeo)
        locbarrier.setlayername("locgeo")
        locbarrier.setattributes(annopoint.attributes)
        self.db.barriergeo.insert(locbarrier)
        
    def insertannobarrier(self,annopoly):
        pgeo = dict(type="Polygon",coordinates=annopoly)
        '''
        locbarrier = barrier(pgeo)
        locbarrier.setlayername("locgeo")
        '''
        locbarrier = dict(type="Feature",attributes=dict(), \
                          layername="locgeo", \
                          geometry = pgeo)
        self.db.barriergeo.insert(locbarrier)
    
    def delannobarrier(self):
        self.db.barriergeo.remove({"layername":"locgeo"})
    
class point:
    
    geometry = dict()
    type = "Feature"
    attributes = dict()
    layername = ""
    uniqueid = 0
    
    def __init__(self,ogeometry):
        geometry = ogeometry
    
    def setattributes(self,oattributes):
        attributes = oattributes
    
    def setgeometry(self,ogeometry):
        geometry = ogeometry
    
    def setlayername(self,olayername):
        layername = olayername
    
    def setuniqueid(self,ouniqueid):
        uniqueid = ouniqueid
        
class barrier:
    
    geometry = dict()
    type = "Feature"
    attributes = dict()
    layername = ""
    
    def __init__(self,ogeometry):
        geometry = ogeometry
    
    def setattributes(self,oattributes):
        attributes = oattributes
    
    def setgeometry(self,ogeometry):
        geometry = ogeometry
    
    def setlayername(self,olayername):
        layername = olayername
    
class annopoint:
    
    geometry = dict()
    type = "Feature"
    attributes = dict()
    layername = ""
    uniqueid = 0
    
    def __init__(self,ogeometry):
        geometry = ogeometry
    
    def setattributes(self,oattributes):
        attributes = oattributes
    
    def setgeometry(self,ogeometry):
        geometry = ogeometry
    
    def setlayername(self,olayername):
        layername = olayername
    
    def setuniqueid(self,ouniqueid):
        uniqueid = ouniqueid
        
        

