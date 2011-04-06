# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0146012262333,-8.03342310198,-0.0223243244596,6.14221627567

InitialRegion: # Initial region number
5

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
PlayerStage

RobotFile: # Relative path of robot description file
sheepdog_stage.robot


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0102655727979,-3.03586515592,-0.0108150581347,3.08960925333

InitialRegion: # Initial region number
2

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
cornell_asl.lab

Name: # Name of the experiment
ASL

RobotFile: # Relative path of robot description file
sheepdog_real.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
start_chase,1
set_free,1
bark,1

Customs: # List of custom propositions
herding

RegionFile: # Relative path of region description file
demoday.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
sheep,1
wolf,1

currentExperimentName:
ASL


======== SPECIFICATION ========

RegionMapping:

hill2=p5
hill1=p6
corral=p11
woods=p2
between$woods$and$valley$=p28,p29,p30
fence1=p10
others=p13,p14,p15,p16,p17,p18,p19,p20,p21,p22,p23,p24,p25,p26,p27
pond=p4
valley=p3
fence3=p8
fence2=p9
hayfield=p7

Spec: # Specification in simple English
### Initial conditions ###
Env starts with false
Robot starts in valley
Robot starts with false

### Environment assumptions ###
If you were in corral then do not sheep
If you were in corral then do not wolf

### Herding behavior ###
herding is set on start_chase and reset on set_free
Do start_chase if and only if you are sensing sheep and you are not activating herding
Do set_free if and only if you are activating herding and you were in corral

# Don't move while starting chase or setting free (need to wind up)
If you are activating start_chase or you activated start_chase then stay there
If you are activating set_free or you activated set_free then stay there

### Wolf-deterring behavior ###
Do bark if and only if you are sensing wolf
If you are activating bark or you activated bark then stay there

### Patrol goals ###
If you are not activating herding and you are not activating bark then visit pond
If you are not activating herding and you are not activating bark then visit hayfield
If you are not activating herding and you are not activating bark then visit woods
If you are not activating herding and you are not activating bark then visit valley

If you are activating herding and you are not activating bark then visit corral
If you did not activate herding then always not corral

### Safety constraints ###
Always not (fence1 or fence2 or fence3 or hill1 or hill2)


# and you are not in between woods and valley


