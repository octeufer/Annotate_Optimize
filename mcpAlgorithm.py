'''
Author:Octeufer
2014/1/10
'''

import random
import hashlib
import numpy as np

class greedymcp:
    
    def __init__(self):
        pass
    
    def FindMaxClique(self,graph,maxTime,targetCliqueSize):
        clique = list()
        time = 0
        timeBestClique = 0
        timeRestart = 0
        nodeToAdd = -1
        nodeToDrop = -1
        
        randomNode = random.randint(0,graph.shape[0]-1)
        print "Adding node %d" %randomNode
        clique.append(randomNode)
        
        bestClique = list()
        bestSize = len(bestClique)
        timeBestClique = time
        
        possibleAdd = self.MakePossibleAdd(graph,clique)
        oneMissing = self.MakeOneMissing(graph,clique)
        
        while time < maxTime and bestSize < targetCliqueSize:
            time = time + 1
            cliqueChanged = False
            if len(possibleAdd) > 0:
                nodeToAdd = self.GetNodeToAdd(graph,possibleAdd)
                print "Adding node %d" %nodeToAdd
                clique.append(nodeToAdd)
                clique.sort()
                cliqueChanged = True
                if len(clique) > bestSize:
                    bestSize = len(clique)
                    bestClique = list()
                    bestClique.extend(clique)
                    timeBestClique = time
            if cliqueChanged == False:
                if len(clique) > 0:
                    nodeToDrop = self.GetNodeToDrop(graph,clique,oneMissing)
                    print "Dropping node %d" %nodeToDrop
                    clique.remove(nodeToDrop)
                    clique.sort()
                    cliqueChanged = True
            
            restart = 2 * bestSize
            if (time - timeBestClique) > restart and (time - timeRestart) > restart:
                print "Restart"
                timeRestart = time
                seedNode = random.randint(0,graph.shape[0]-1)
                clique = list()
            
            possibleAdd = self.MakePossibleAdd(graph,clique)
            oneMissing = self.MakeOneMissing(graph,clique)
        
        return bestClique
    
    def MakePossibleAdd(self,graph,clique):
        def FormsALargerClique(graph,clique,node):
            for i in range(len(clique)):
                if graph[clique[i],node] == 0:
                    return False
            return True
        result = list()
        result = [i for i in range(len(graph)) if FormsALargerClique(graph,clique,i) == True]
        return result
    
    def GetNodeToAdd(self,graph,possibleAdd):
        l = len(possibleAdd)
        if l==1:
            return possibleAdd[0]
        maxDegree = 0
        for i in range(l):
            currNode = possibleAdd[i]
            degreeOfCurrentNode = 0
            for j in range(l):
                otherNode = possibleAdd[j]
                if graph[currNode,otherNode] == 1:
                    degreeOfCurrentNode = degreeOfCurrentNode + 1
            if degreeOfCurrentNode > maxDegree:
                maxDegree = degreeOfCurrentNode
        
        candidates = list()
        for i in range(l):
            currNode = possibleAdd[i]
            degreeOfCurrentNode = 0
            for j in range(l):
                otherNode = possibleAdd[j]
                if graph[currNode,otherNode] == 1:
                    degreeOfCurrentNode = degreeOfCurrentNode + 1
            if degreeOfCurrentNode == maxDegree:
                candidates.append(currNode)
        
        return candidates[random.randint(0,len(candidates)-1)]
    
    def GetNodeToDrop(self,graph,clique,oneMissing):
        lc = len(clique)
        lm = len(oneMissing)
        if lc == 1:
            return clique[0]
        maxCount = 0
        for i in range(lc):
            currCliqueNode = clique[i]
            countNotAdjacent = 0
            for j in range(lm):
                currOneMissingNode = oneMissing[j]
                if graph[currCliqueNode,currOneMissingNode] == 0:
                    countNotAdjacent = countNotAdjacent + 1
            if countNotAdjacent > maxCount:
                maxCount = countNotAdjacent
        
        candidates = list()
        for i in range(lc):
            currCliqueNode = clique[i]
            countNotAdjacent = 0
            for j in range(lm):
                currOneMissingNode = oneMissing[j]
                if graph[currCliqueNode,currOneMissingNode] == 0:
                    countNotAdjacent = countNotAdjacent + 1
            if countNotAdjacent == maxCount:
                candidates.append(currCliqueNode)
        
        return candidates[random.randint(0,len(candidates)-1)]
    
    def MakeOneMissing(self,graph,clique):
        count = 0
        result = list()
        for i in range(graph.shape[0]):
            if (graph[i]>0).sum() < len(clique):continue
            if i in clique:continue
            for j in range(len(clique)):
                if graph[i,clique[j]] == 1:
                    count = count + 1
            if count == len(clique) - 1:
                result.append(i)
        return result
    

class tabumcp:
    
    def __init__(self,graph):
        self.graph = graph
        self.clique = list()
        self.time = 0
        self.timeBestClique = 0
        self.timeRestart = 0
        
        self.prohibitPeriod = 1
        self.timeProhibitChanged = 0
                   
        self.lastMoved = np.zeros((graph.shape[0]),np.int32)
        self.history = {}
    
    def FindMaxClique(self,maxTime):
        nodeToAdd = -1
        nodeToDrop = -1
        iter = 0
        seedNode = -1
        temp = [self.lastMoved[i] for i in range(len(self.lastMoved)) if self.lastMoved[i] == 0]
        if len(temp)>0:
            seedNode = temp[random.randint(0,len(temp)-1)]
        else:
            seedNode = random.randint(0,self.graph.shape[0]-1)

        #randomNode = random.randint(0,graph.shape[0]-1)
        #print "Adding node %d" %randomNode
        self.clique = list()
        self.clique.append(seedNode)
        
        bestClique = list()
        bestSize = len(bestClique)
        self.timeBestClique = self.time
        
        possibleAdd = self.MakePossibleAdd(self.graph,self.clique)
        oneMissing = self.MakeOneMissing(self.graph,self.clique)
        
        while iter < maxTime and bestSize < self.graph.shape[0]:
            iter = iter + 1
            self.time = self.time + 1
            cliqueChanged = False
            if len(possibleAdd) > 0:
                #nodeToAdd = self.GetNodeToAdd(graph,possibleAdd)
                allowedAdd = self.SelectAllowedNodes(possibleAdd,self.time,self.prohibitPeriod,self.lastMoved)
                if len(allowedAdd)>0:
                    nodeToAdd = self.GetNodeToAdd(self.graph,allowedAdd,possibleAdd)
                    #print "Adding node %d" %nodeToAdd
                    self.clique.append(nodeToAdd)
                    self.lastMoved[nodeToAdd] = self.time
                    self.clique.sort()
                    cliqueChanged = True
                    if len(self.clique) > bestSize:
                        bestSize = len(self.clique)
                        bestClique = list()
                        bestClique.extend(self.clique)
                        self.timeBestClique = self.time
            if cliqueChanged == False:
                if len(self.clique) > 0:
                    #nodeToDrop = self.GetNodeToDrop(graph,clique,oneMissing)
                    allowedInClique = self.SelectAllowedNodes(self.clique,self.time,self.prohibitPeriod,self.lastMoved)
                    if len(allowedInClique)>0:
                        nodeToDrop = self.GetNodeToDrop(self.graph,allowedInClique,oneMissing)
                        #print "Dropping node %d" %nodeToDrop
                        self.clique.remove(nodeToDrop)
                        self.lastMoved[nodeToDrop] = self.time
                        self.clique.sort()
                        cliqueChanged = True
            
            if cliqueChanged == False:
                if len(self.clique) > 0:
                    nodeToDrop = self.clique[random.randint(0,len(self.clique)-1)]
                    self.clique.remove(nodeToDrop)
                    self.lastMoved[nodeToDrop] = self.time
                    self.clique.sort()
                    cliqueChanged = True                
            
            restart = 2 * bestSize
            if (self.time - self.timeBestClique) > restart and (self.time - self.timeRestart) > restart:
                #print "Restarting with prohibit period %d" %self.prohibitPeriod
                self.timeRestart = self.time
                self.prohibitPeriod = 1
                self.timeProhibitChanged = self.time
                
                self.history = {}
                
                self.seedNode = -1
                temp = [self.lastMoved[i] for i in range(len(self.lastMoved)) if self.lastMoved[i] == 0]
                if len(temp)>0:
                    seedNode = temp[random.randint(0,len(temp)-1)]
                else:
                    seedNode = random.randint(0,self.graph.shape[0]-1)
                #seedNode = random.randint(0,graph.shape[0]-1)
                self.clique = list()
                self.clique.append(seedNode)
            
            possibleAdd = self.MakePossibleAdd(self.graph,self.clique)
            oneMissing = self.MakeOneMissing(self.graph,self.clique)
            
            self.prohibitPeriod,self.timeProhibitChanged = self.UpdateProhibitPeriod(self.graph,self.clique,bestSize,self.history,self.time,self.prohibitPeriod,self.timeProhibitChanged)
        
        return bestClique
    
    def MakePossibleAdd(self,graph,clique):
        def FormsALargerClique(graph,clique,node):
            for i in range(len(clique)):
                if graph[clique[i],node] == 0:
                    return False
            return True
        result = list()
        result = [i for i in range(len(graph)) if FormsALargerClique(graph,clique,i) == True]
        return result
     
    def GetNodeToAdd(self,graph,allowedAdd,possibleAdd):
        l = len(allowedAdd)
        lp = len(possibleAdd)
        if l==1:
            return allowedAdd[0]
        maxDegree = 0
        for i in range(l):
            currNode = allowedAdd[i]
            degreeOfCurrentNode = 0
            for j in range(lp):
                otherNode = possibleAdd[j]
                if graph[currNode,otherNode] == 1:
                    degreeOfCurrentNode = degreeOfCurrentNode + 1
            if degreeOfCurrentNode > maxDegree:
                maxDegree = degreeOfCurrentNode
        
        candidates = list()
        for i in range(l):
            currNode = allowedAdd[i]
            degreeOfCurrentNode = 0
            for j in range(lp):
                otherNode = possibleAdd[j]
                if graph[currNode,otherNode] == 1:
                    degreeOfCurrentNode = degreeOfCurrentNode + 1
            if degreeOfCurrentNode == maxDegree:
                candidates.append(currNode)
        
        return candidates[random.randint(0,len(candidates)-1)]

    def GetNodeToDrop(self,graph,allowedInClique,oneMissing):
        lc = len(allowedInClique)
        lm = len(oneMissing)
        if lc == 1:
            return allowedInClique[0]
        maxCount = 0
        for i in range(lc):
            currCliqueNode = allowedInClique[i]
            countNotAdjacent = 0
            for j in range(lm):
                currOneMissingNode = oneMissing[j]
                if graph[currCliqueNode,currOneMissingNode] == 0:
                    countNotAdjacent = countNotAdjacent + 1
            if countNotAdjacent > maxCount:
                maxCount = countNotAdjacent
        
        candidates = list()
        for i in range(lc):
            currCliqueNode = allowedInClique[i]
            countNotAdjacent = 0
            for j in range(lm):
                currOneMissingNode = oneMissing[j]
                if graph[currCliqueNode,currOneMissingNode] == 0:
                    countNotAdjacent = countNotAdjacent + 1
            if countNotAdjacent == maxCount:
                candidates.append(currCliqueNode)
        
        return candidates[random.randint(0,len(candidates)-1)]

    def MakeOneMissing(self,graph,clique):
        count = 0
        result = list()
        for i in range(graph.shape[0]):
            if (graph[i]>0).sum() < len(clique):continue
            if i in clique:continue
            for j in range(len(clique)):
                if graph[i,clique[j]] == 1:
                    count = count + 1
            if count == len(clique) - 1:
                result.append(i)
        return result
    
    def SelectAllowedNodes(self,listOfNodes,time,prohibitPeriod,lastMoved):
        result = list()
        if len(listOfNodes)==0:
            return result
        for i in range(len(listOfNodes)):
            currNode = listOfNodes[i]
            if time > lastMoved[currNode] + prohibitPeriod:
                result.append(currNode)
        return result
    
    def UpdateProhibitPeriod(self,graph,clique,bestSize,history,time,prohibitPeriod,timeProhibitChanged):
        result = prohibitPeriod
        
        cliqueInfo = CliqueInfo(clique,time)
        if history.has_key(cliqueInfo.GetHashCode()):
            ci = history[cliqueInfo.GetHashCode()]
            intervalSinceLastVisit = time - ci.lastSeen
            ci.lastSeen = time
            if intervalSinceLastVisit < 2 * (graph.shape[0] - 1):
                timeProhibitChanged = time
                if prohibitPeriod + 1 < 2 * bestSize:
                    return prohibitPeriod + 1,timeProhibitChanged
                else:
                    return 2 * bestSize,timeProhibitChanged
        else:
            history[cliqueInfo.GetHashCode()] = cliqueInfo
        
        if time - timeProhibitChanged > 10 * bestSize:
            timeProhibitChanged = time
            if prohibitPeriod - 1 > 1:
                return prohibitPeriod - 1,timeProhibitChanged
            else:
                return 1,timeProhibitChanged
        else:
            return result,timeProhibitChanged

class CliqueInfo:
    
    def __init__(self,clique,lastSeen):
        self.clique = list()
        self.clique.extend(clique)
        self.lastSeen = lastSeen
    
    def GetHashCode(self):
        strc = ""
        for i in range(len(self.clique)):
            if i==len(self.clique) - 1:
                strc = strc + str(self.clique[i])
            else:
                strc = strc + str(self.clique[i]) + " "
        return hashlib.sha1(strc).hexdigest()
        