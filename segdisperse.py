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
