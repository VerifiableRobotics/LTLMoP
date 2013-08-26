#!/usr/bin/env python

import os
import sys
import json

# find LTLMoP repo
ltlmop_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.append(os.path.join(ltlmop_root, "lib"))

# load in the LTLMoP library
import regions

# load in topo file
print "Loading '{}'...".format(sys.argv[1])
with open(sys.argv[1]) as f:
    data = f.read()

data = json.loads(data)

# create LTLMoP regions
rfi = regions.RegionFileInterface()

for rn in data["region_names"]:
    newRegion = regions.Region(name=rn)
    rfi.regions.append(newRegion)

# force topology (we avoid LTLMoP's automatic coincident edge detection)
rfi.transitions = [[[] for j in range(len(rfi.regions))] for i in range(len(rfi.regions))]
for path in data["adjacencies"]:
    r1 = rfi.indexOfRegionWithName(path[0])
    r2 = rfi.indexOfRegionWithName(path[1])
    fake_face = [frozenset((regions.Point(1,2), regions.Point(3,4)))]
    rfi.transitions[r1][r2] = fake_face
    rfi.transitions[r2][r1] = fake_face

# save file
output_filename = os.path.splitext(sys.argv[1])[0] + ".converted.regions"
rfi.writeFile(output_filename)

print "Wrote to '{}'.".format(output_filename)
print "WARNING: Be sure to manually edit the .spec file you will use this map with and set 'decompose: False'!"
