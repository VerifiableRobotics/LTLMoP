# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
loadGravelGaits, 1
loadInclineGaits, 1
sitDown, 1
wave, 0
reactToSpiderGotOut, 1
calmDown, 1
relief, 1
threatenTheNao, 1
threatenTheSpider, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Full simulation

Customs: # List of custom propositions
reachedNao

RegionFile: # Relative path of region description file
SpiderAndNao.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
headTap, 1
theEnd, 0
boxOverYou, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p13, p14
r5 = p2
r6 = p15, p16
r1 = p17, p18
r2 = p5
r3 = p4
naoArea = p7
mesh = p8
others = 
incline = p9

Spec: # Specification in structured English
# For this demo the Spider will walk across the field and go scare the nao. The spider will start under a box
# and the demo will beguin once the spider is unboxed. The box will be thrown back on the pider once it
# starts to walk away from the nao and just before it goes up the incline
#																		Robert V.

# INITIAL CONDITIONS AND SETUP
robot starts in r1 with false
reachedNao is set on threatenTheNao and reset on false

# GAITS WILL BE LOADED BASED ON MAP LOCATION
do loadGravelGaits if and only if you are in mesh
do loadInclineGaits if and only if you are in incline

# ACTIONS
do sitDown if and only if you are sensing boxOverYou
do reactToSpiderGotOut if and only if end of boxOverYou and you are not activating reachedNao
do calmDown if and only if you are sensing headTap
do threatenTheNao if and only if you are in naoArea
do threatenTheSpider if and only if you are in naoArea
if start of boxOverYou and you are activating reachedNao then do relief

# GO TOWARDS THE NAO UNTILL YOU GET TO HIM AND ONCE YOU DO START TO HEAD BACK
if you are not sensing boxOverYou or headTap and you are not activating reachedNao then visit naoArea
if you are not sensing boxOverYou or headTap and you are activating reachedNao then visit r1

if you were sensing boxOverYou or you are sensing boxOverYou then stay there
if you were sensing headTap or you are sensing headTap then stay there

