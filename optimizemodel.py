'''
    author:Octeufer
    date:2013/11/28
'''

import json
import pymongo
from bson.son import SON
import shapefile
import math
import AnnoOptmize
import random

dis = float(3)
psym = float(3)
annosymwidth = float(15)
pi = math.pi
#pi的位置
scores = [0,2,3,4,4,4,1,2,4]

class optimizemodel:
    
    def __init__(self,dbname):
        self.client = pymongo.MongoClient("localhost",27017)
        self.db = self.client[dbname]
    
    def locgenerate(self,p0,dis,psym,annosymwidth):
        k = dis + psym / 2 + annosymwidth / 2
        x0 = p0[0]
        y0 = p0[1]
        directions = []
        for i in range(8):
            x = x0 + k * math.cos(i * pi / 4)
            y = y0 + k * math.sin(i * pi / 4)
            polyanno = AnnoOptmize.rectp([x,y],3,15)
            directions.append((polyanno,i))
        return directions
    
    def domaingenerate(self,lyrname):
        
        if(lyrname == None):
            for point in self.db.annopois.find():
                direcs = dirloccheck(point)
                self.db.annopois.update({"directions":direcs})
                
        else:
            for point in self.db.annopois.find({"layername":"lyrname"}):
                direcs = dirloccheck(point)
                self.db.annopois.update({"directions":direcs})
    
    def dirloccheck(self,point):
        directions = locgenerate(point["geometry"]["coordinates"])
        for i in range(len(directions)):
            geos = [geo for geo in self.db.barriergeo.find({"geometry":{"$geoWithin":{"$geometry":{  
                                                 "type":"Polygon","coordinates":directions[i][0]}}}})]
            if(len(geos)>0):
                del directions[i]
        return directions
    
    def domains(self):
        annoloc=[]
        for point in self.db.annopois.find():
            l = len(point["directions"])
            nums = [num for num in range(l)]
            if l == 0:
                annoloc.append(int(9))
                continue
            for nu in nums:
                locnum = random.randint(0,len(nums)-1)
                geos = [geo for geo in self.db.barriergeo.find({"geometry":{"$geoWithin":{"$geometry":{  
                                                 "type":"Polygon","coordinates":point["directions"][nums[locnum]][0]}}}})]
                if len(geos) == 0:
                    AnnoOptmize.insertannobarrier(point["directions"][nums[locnum]][0])
                    annoloc.append(point["directions"][nums[locnum]][1])
                    break
                elif len(geos) > 0:
                    del nums[locnum]
            if len(nums)==0:
                annoloc.append(int(9))
        return annoloc
    
    def cost(self,vec):
        cost = 0
        for v in vec:
            cost = cost + scores[v]
        return cost


class annodirection:
    
    def __init__(self,directions):
        if directions == []:
            self.rr = []
            self.ru = []
            self.rb = []
            self.uu = []
            self.bb = []
            self.ll = []
            self.lu = []
            self.lb = []
        else:
            self.rr = directions[0]
            self.ru = directions[1]
            self.rb = directions[2]
            self.uu = directions[3]
            self.bb = directions[4]
            self.ll = directions[5]
            self.lu = directions[6]
            self.lb = directions[7]
    
    def setrr(self,orr):
        self.rr = orr
    def setru(self,oru):
        self.ru = oru
    def setrb(self,orb):
        self.rb = orb
    def setuu(self,ouu):
        self.uu = ouu
    def setbb(self,obb):
        self.bb = obb
    def setll(self,oll):
        self.ll = oll
    def setlu(self,olu):
        self.lu = olu
    def setlb(self,olb):
        self.lb = olb
    
    def getrr(self):
        return self.rr
    def getru(self):
        return self.ru
    def getrb(self):
        return self.rb
    def getuu(self):
        return self.uu
    def getbb(self):
        return self.bb
    def getlu(self):
        return self.lu
    def getll(self):
        return self.ll
    def getlb(self):
        return self.lb