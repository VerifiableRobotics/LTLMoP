# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
count, 1
whistle, 1
hide, 1
say_foundyou, 1
say_imfound, 1
say_hider, 1
say_seeker, 1

CurrentConfigName:
Basic simulation

Customs: # List of custom propositions
seeker
playing

RegionFile: # Relative path of region description file
hideandseek.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
see_player, 1
hear_whistle, 1
hear_counting, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Classroom1 = p12
Classroom2 = p11
Office = p7
Closet = p10
Gym = p8
others = p13, p14, p15, p16, p17, p18, p19, p20, p21, p22, p23, p24, p25, p26
Parking = p6
between$Tree$and$Wall$ = p2

Spec: # Specification in structured English
### Overview ###
# Start in the parking lot.  If you are a seeker, stay there and count until you hear a ready whistle.
# Then search all the hiding spots until you find someone.  Once you've found someone, you are now a hider.
# If you are a hider, go back to the parking lot and wait for counting to start.
# Once that happens, go hide somewhere and whistle.

### Initialization ###
# Allow for the possibility of starting out with other players in view
Environment starts with not hear_whistle and not hear_counting
Robot starts in Parking
# Robot can start as either seeker or hider
Robot starts with not count and not whistle and not hide and not playing

### Game start and end conditions ###
playing is set on ((seeker and Parking and hear_whistle) or (not seeker and Parking and hear_counting)) and reset on (see_player)
If you are not activating playing then go to Parking and stay there

### Hider/seeker alternation ###
seeker is toggled on end of playing

### Seeking Behavior ###
Do count if and only if you are activating seeker and you are not activating playing and you are in Parking

group SeekingSpots is Classroom2, Office, Gym, Closet, Classroom1, between Tree and Wall
If you are activating seeker and playing then visit all SeekingSpots

### Hiding Behavior ###

group HidingSpots is between Tree and Wall, Closet, Office
If you are not activating seeker and you are activating playing then go to any HidingSpots
Do hide if and only if you are not activating seeker and you are activating playing and you are in ((between Tree and Wall) or Closet or Office)

If you were activating hide then stay there
Do whistle if and only if start of hide

# Make reactions to see_player immediate, not waiting for transition to next region
#If end of playing then stay there

### Talking ###
Do say_imfound if and only if you were not activating seeker and end of playing
Do say_foundyou if and only if you were activating seeker and end of playing
Do say_hider if and only if you are not activating seeker and you are not activating playing and you are in Parking
Do say_seeker if and only if you are activating seeker and you are not activating playing and you are in Parking

# don't talk while walking
If you were activating say_imfound or say_foundyou or say_hider or say_seeker then stay there

