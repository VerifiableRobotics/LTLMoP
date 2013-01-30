import sys, re, os
import Polygon, Polygon.IO
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
import lib.regions as regions


fn_in = sys.argv[1]
fn_out = re.sub(r"\.regions$", ".svg", fn_in)
rfi = regions.RegionFileInterface()
rfi.readFile(fn_in)

polyList = []

for region in rfi.regions:
    points = [(pt.x,pt.y) for pt in region.getPoints()]
    poly = Polygon.Polygon(points)
    polyList.append(poly)

Polygon.IO.writeSVG(fn_out, polyList)
