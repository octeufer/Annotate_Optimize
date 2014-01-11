'''
Author:Octeufer
2014/1/10
'''

import random

class greedymcp:
    
    def __init__(self):
        pass
    
    def FindMaxClique(self,graph,maxTime,targetCliqueSize):
        clique = list()
        time = 0
        timeBeatClique = 0
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
    
     