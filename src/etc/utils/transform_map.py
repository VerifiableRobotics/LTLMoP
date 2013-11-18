#!/usr/bin/env python

""" Rescales and translates maps to make them fit reasonably on the screen.
    (This is mainly a stop-gap convenience tool for importing "New Region Editor"
    maps into "Old Region Editor.")
    Outputs resulting "calibration matrix" that LTLMoP should use for pose transformation,
    assuming input coordinates are in the "lab" coordinate system.
"""

import os
import sys
from numpy import *

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p, "src", "lib"))
import regions

# Check that we have been passed an input file
if len(sys.argv) != 2:
    print "Usage: {} [input_region_file]".format(sys.argv[0])
    sys.exit(-1)
    
# Load in map file
print "Loading '{}'...".format(sys.argv[1])
mapfile = regions.RegionFileInterface()
mapfile.readFile(sys.argv[1])

# Make all regions relative to (0,0), so they effectively store absolute coordinates
for region in mapfile.regions:
    region.pointArray = map(lambda pt: region.position+pt, region.pointArray)
    region.position = regions.Point(0,0)

# Calculate transformation matrix
# Flip over y-axis, scale up to make width be 1000, and topleft be (100,100)
leftMargin, topMargin, rightExtent, downExtent = mapfile.getBoundingBox()
scale_factor = 1000.0/rightExtent
offset = regions.Point(100,100) - regions.Point(leftMargin, -(topMargin+downExtent))*scale_factor
T = matrix([[scale_factor, 0, offset.x],
            [0, -scale_factor, offset.y],
            [0, 0, 1]])

# Apply transformation
# From lib/project.py
coordmap_lab2map = lambda pt: regions.Point(*((T * mat([pt.x, pt.y, 1]).T).T.tolist()[0][0:2]))

for region in mapfile.regions:
    region.pointArray = map(coordmap_lab2map, region.pointArray)
    region.recalcBoundingBox()

# Save file
output_filename = os.path.splitext(sys.argv[1])[0] + "_adjusted.regions"
mapfile.writeFile(output_filename)

print "Wrote to '{}'.".format(output_filename)

print "The calculated calibration matrix is: "
print repr(T)
print "You can just squirrel this into the appropriate .config file"
