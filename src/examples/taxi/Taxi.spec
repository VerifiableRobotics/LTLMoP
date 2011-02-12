# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.01638155825,-7.93811136134,-0.0228593749925,5.93851573486

InitialRegion: # Initial region number
0

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
Taxi_stage.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
PickUp,0
DropOff,0

Customs: # List of custom propositions
Passenger
Stop

RegionFile: # Relative path of region description file
Taxi.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
RedLight,1
GreenLight,0
Person,1
Jaywalker,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

R01=p10
R03=p8
R02=p9
R05=p6
R04=p7
R07=p4
R06=p5
I04=p14
others=p18,p19,p20,p21,p22,p23,p24,p25
I02=p16
I03=p15
I01=p17
I06=p12
Lot=p11
I05=p13

Spec: # Specification in simple English
# Initialization
Env starts with false
Robot starts with false
If you were in R01 or R02 or R03 or R04 or R05 or R06 or R07 or Lot then do not RedLight
If you were in Lot or I01 or I02 or I03 or I04 or I05 or I06 then do not Person

# Stop at red lights
#Do Stop if and only if you are sensing RedLight and you are in (I01 or I02 or I03 or I04 or I05 or I06) or you are sensing Jaywalker

# Stop when necessary
If you activated Stop then stay there

# Drive around looking for passengers
If you are not activating Passenger and you are not activating Stop then visit R01
If you are not activating Passenger and you are not activating Stop then visit R02
If you are not activating Passenger and you are not activating Stop then visit R03
If you are not activating Passenger and you are not activating Stop then visit R04
If you are not activating Passenger and you are not activating Stop then visit R05
If you are not activating Passenger and you are not activating Stop then visit R06
If you are not activating Passenger and you are not activating Stop then visit R07

#Always do not Passenger

# Pick up and drop off passengers
If you are sensing Person and you did not activate Passenger then do Passenger
If you activated Passenger and you are in Lot then do not Passenger
If you activated Passenger and you are not in Lot then do Passenger
If you did not activate Passenger and you are not sensing Person then do not Passenger

# Drive passenger to destination
If you are activating Passenger and you are not activating Stop then visit Lot


