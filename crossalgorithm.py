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