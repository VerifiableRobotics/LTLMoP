import sys, re, math
sys.path.append('../..')
import lib.regions as regions
import xml.dom.minidom
import wx

f_in = sys.argv[1]
fn_out = re.sub(r"\.xml$", ".regions", f_in)

dom = xml.dom.minidom.parse(f_in)

def getText(nodelist):
    rc = []
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc.append(node.data)
    return ''.join(rc)

def getPoints(nl):
    pts = []
    for n in nl:
        if n.nodeType == n.TEXT_NODE:
            continue
        pts.append(map(float, (getText(n.getElementsByTagName("X")[0].childNodes), getText(n.getElementsByTagName("Y")[0].childNodes))))
    return pts

rfi = regions.RegionFileInterface()

SCALE = 100
OFFSET = 300
WALL_THICKNESS = 10

bl = getPoints(dom.getElementsByTagName("bottomLeft")[0].childNodes)[0]
tr = getPoints(dom.getElementsByTagName("topRight")[0].childNodes)[0]
border = regions.Region(type=regions.reg_RECT, position=wx.Point(SCALE*bl[0]+OFFSET, -SCALE*tr[1]+OFFSET), size=wx.Size(SCALE*(tr[0]-bl[0]), SCALE*(tr[1]-bl[1])), name='boundary')
rfi.regions.append(border)

wall_num = 1
for x in dom.getElementsByTagName("ArrayOfVector2"):
    #print getPoints(x.childNodes)
    pts = getPoints(x.childNodes)

    for i, pt in enumerate(pts):
        pts[i] = wx.Point(SCALE*pt[0]+OFFSET, -SCALE*pt[1]+OFFSET)

    ang = math.atan2(pts[1].y-pts[0].y, pts[1].x-pts[0].x) + math.pi/2
    pv = wx.Point((WALL_THICKNESS/2)*math.cos(ang), (WALL_THICKNESS/2)*math.sin(ang))
    
    #pts = [pts[0]+pv, pts[1]+pv, pts[1]-pv, pts[0]-pv]
    
    reg = regions.Region(type=regions.reg_POLY, points=pts, name="wall"+str(wall_num))
    reg.recalcBoundingBox()
    rfi.regions.append(reg)
    wall_num = wall_num + 1

rfi.thumb = "None"
rfi.recalcAdjacency()
rfi.writeFile(fn_out)
