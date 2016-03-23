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
import shapefile

dis = float(3)
psym = float(3)
annosymwidth = float(15)
pi = math.pi
#pi的位置
scores = [0,2,3,4,4,4,1,2,4]

class optimizemodel:
    
    def __init__(self,dbname):
        #self.client = pymongo.MongoClient("localhost",27017)
        #self.db = self.client[dbname]
        self.dataengine = AnnoOptmize.dataengine(dbname)
    
    def locgenerate(self,p0,dis,psym,annosymwidth):
        k = dis + psym / 2 + annosymwidth / 2 
        k = k / 1000 / 1000 * 10000
        x0 = p0[0]
        y0 = p0[1]
        directions = []
        for i in range(8):
            x = x0 + k * math.cos(i * pi / 4)
            #y = y0 + k * math.sin(i * pi / 4)
            if i == 0 or i == 4:
                y = y0
            else:
                y = y0 + (dis + psym) / 1000 / 1000 * 10000
            polyanno = AnnoOptmize.rectp([x,y],3,15)
            directions.append((polyanno,i))
        return directions
    
    def locgen(self,p0,dis,psym,annosymwidth,loc):
        k = dis + psym / 2 + annosymwidth / 2
        k = k / 1000 / 1000 * 10000
        x0 = p0[0]
        y0 = p0[1]
        x = x0 + k * math.cos(loc * pi / 4)
        if loc == 0 or loc == 4:
            y = y0
        else:
            y = y0 + (dis + psym) / 1000 /1000 * 10000
        polyanno = AnnoOptmize.rectp([x,y],3,15)
        return polyanno
    
    def polyannogen(self,p0,dis,psym,annosymwidth,loc):
        k = dis + psym / 2 + annosymwidth / 2
        k = k / 1000 / 1000 * 10000
        x0 = p0[0]
        y0 = p0[1]
        x = x0 + k * math.cos(loc * pi / 4)
        if loc == 0 or loc == 4:
            y = y0
        else:
            y = y0 + (dis + psym) / 1000 /1000 * 10000
        polyanno = AnnoOptmize.rectp([x,y],3,15)
        #print polyanno
        polyanno = [[[part[0] * 1000,part[1] * 1000] for part in polyanno[0]]]
        #print polyanno
        return polyanno
    
    def domaingenerate(self,lyrname):
        
        if(lyrname == None):
            for point in self.dataengine.db.annopois.find():
                direcs = self.dirloccheck(point)
                self.dataengine.db.annopois.update({"uniqueid":point["uniqueid"]},{"$set":{"directions":direcs}})
                
        else:
            for point in self.dataengine.db.annopois.find({"layername":"lyrname"}):
                direcs = self.dirloccheck(point)
                self.dataengine.db.annopois.update({"directions":direcs})
    
    def dirloccheck(self,point):
        directions = self.locgenerate(point["geometry"]["coordinates"],dis,psym,annosymwidth)
        for i in range(len(directions)):
            geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"$geoIntersects":{"$geometry":{  
                                                 "type":"Polygon","coordinates":directions[i][0]}}}})]
            if(len(geos)>0):
                del directions[i]
        return directions
    
    def domains(self):
        annoloc=[]
        for point in self.dataengine.db.annopois.find():
            l = len(point["directions"])
            nums = [num for num in range(l)]
            if l == 0:
                annoloc.append((([[[0,0]]],int(8)),point["directions"]))
                continue
            for nu in nums:
                locnum = random.randint(0,len(nums)-1)
                geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"$geoIntersects":{"$geometry":{  
                                                 "type":"Polygon","coordinates":point["directions"][nums[locnum]][0]}}}})]
                if len(geos) == 0:
                    
                    self.dataengine.insertannobarrier(point["directions"][nums[locnum]][0])
                    annoloc.append((point["directions"][nums[locnum]],point["directions"]))
                    break
                elif len(geos) > 0:
                    del nums[locnum]
            if len(nums)==0:
                annoloc.append((([[[0,0]]],int(8)),point["directions"]))
        return annoloc
    
    def cost(self,vec):
        cost = 0
        if(vec == None):
            return 9999
        for i in range(len(vec)):
            #print vec[i][0][1]
            if len(vec[i][0]) < 2:
                print i,vec[i][0]
            cost = cost + scores[vec[i][0][1]]
        return cost
    
    def costsec(self,dvec):
        cost = 0
        for dk in dvec:
            cost = cost + scores[dvec[dk]]
        return cost
    
    def geneticoptimize(self,popsize=50,step=1,mutprob=0.2,elite=0.2,maxiter=100):
        
        def mutate(vec):
            i = random.randint(0,len(vec)-1)
            l = len(vec[i][1])
            if l == 0:
                return mutate(vec)
            self.dataengine.delannobarrier()
            for ii in range(len(vec)):
                if ii == i:
                    continue
                self.dataengine.insertannobarrier(vec[ii][0][0])
            #ss = [dirv[1][1] for dirv in vec[i][1]]
            #nums = [num for num in range(l)]
            if random.random() < 0.5 and vec[i][1].index(vec[i][0]) > 0:
                s1 = vec[i][1][0:vec[i][1].index(vec[i][0])]
                for locnum in s1[::-1]:
                    geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"geoIntersects":{"$geometry":{  
                                                 "type":"Polygon","coordinates":locnum[0]}}}})]
                    if len(geos) == 0:
                        self.dataengine.insertannobarrier(locnum[0])
                        return vec[0:i] + [(locnum,vec[i][1])] + vec[i+1:]
                        break
                return vec[0:i] + [(([[[0,0]]],int(8)),vec[i][1])] + vec[i+1:]
            elif vec[i][1].index(vec[i][0]) < l-2:
                for locnum in vec[i][1][vec[i][1].index(vec[i][0]):]:
                    geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"geoIntersects":{"$geometry":{  
                                                 "type":"Polygon","coordinates":locnum[0]}}}})]
                    if len(geos) == 0:
                        self.dataengine.insertannobarrier(locnum[0])
                        return vec[0:i] + [(locnum,vec[i][1])] + vec[i+1:]
                        break
                return vec[0:i] + [(([[[0,0]]],int(8)),vec[i][1])] + vec[i+1:]
        
        def crossover(r1,r2):
            self.dataengine.delannobarrier()
            i = random.randint(0,len(vec)-2)
            r = r1[0:i] + r2[i:]
            if self.conflictcheck(r):
                return crossover(r1,r2)
            else:
                return r
        
        pop = []
        for i in range(popsize):
            self.dataengine.delannobarrier()
            vec = self.domains()
            pop.append(vec)
        
        topelite = int(elite * popsize)
        for i in range(maxiter):
            ascores = [(self.cost(v),v) for v in pop]
            ascores.sort()
            ranked = [v for (s,v) in ascores]
            
            pop = ranked[0:topelite]
            
            while len(pop) < popsize:
                if random.random() < mutprob:
                    c = random.randint(0,topelite)
                    pop.append(mutate(ranked[c]))
                else:
                    c1 = random.randint(0,topelite)
                    c2 = random.randint(0,topelite)
                    pop.append(crossover(ranked[c1],ranked[c2]))
            print ascores[0][0]
        return ascores[0][1]
    
    def genannoshp(self,lyrname,vec):
        shpwr = shapefile.Writer(shapefile.POLYGON)
        shpwr.field("uniqueid","D","40")
        points = [annop for annop in self.dataengine.db.annopois.find()]
        for i in range(len(points)):
            if vec[i][0][1] == int(8):
                continue
            shpwr.poly(parts=self.polyannogen(points[i]["geometry"]["coordinates"],dis, psym, annosymwidth, vec[i][0][1]))
            shpwr.record(points[i]["uniqueid"])
        shpwr.save(AnnoOptmize.path + lyrname)
        
    def genshpsec(self,lyrname,dvec):
        shpwr = shapefile.Writer(shapefile.POLYGON)
        shpwr.field("uniqueid","D","40")
        for dk in dvec:
            point = [p for p in self.dataengine.db.annopois.find({"uniqueid":dk})]
            shpwr.poly(parts = self.polyannogen(point[0]["geometry"]["coordinates"], dis, psym, annosymwidth, dvec[dk]))
            shpwr.record(dk)
        shpwr.save(AnnoOptmize.path + lyrname)
        
    def conflictcheck(self,vec):
        
        for v in vec:
             geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"$geoIntersects":{"$geometry":{  
                                                 "type":"Polygon","coordinates":v[0][0]}}}})]
             if len(geos) == 0:
                 self.dataengine.insertannobarrier(v[0][0])
             elif len(geos) > 0:
                 return True
                 break
        return False
    
    def solgen(self):
        points = [point for point in self.dataengine.db.annopois.find()]
        solve = dict()
        ll = 0
        count = 0
        while len(points)>0:
            flag = 0
            ll = ll + count
            for i in range(ll,9):
                if i == 8:
                    for point in points:
                        solve[point["uniqueid"]] = 8
                        points.remove(point)
                        break
                count = 0
                for point in points:
                    poly = self.locgen(point["geometry"]["coordinates"], dis, psym, annosymwidth, i)
                    geos = [geo for geo in self.dataengine.db.barriergeo.find({"geometry":{"$geoIntersects":{"$geometry":{  
                                                     "type":"Polygon","coordinates":poly}}}})]
                    if len(geos) == 0:
                        self.dataengine.insertannobarrier(poly)
                        solve[point["uniqueid"]] = i
                        points.remove(point)
                        flag = 1
                        break
                if flag==1:
                    break
                else:
                    count = count + 1
        return solve
                    

