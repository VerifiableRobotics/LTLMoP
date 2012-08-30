/*----------------------------------------------------------------------------------------------------
This is the code that should be uploaded onto the Spider's Arduino Mega board. Please refer to the
comments and the Spider Robot page on the wiki for information with regards to this robot and its 
current functionalities and limitations.

Robert Villalba
----------------------------------------------------------------------------------------------------*/

/* The common gaits for performing most movements available to this robot. If you have created
 a gait in the format specified in the Init handler, use the printGait() function to copy 
 and paste them here*/
int standUp[][19] = {
  {
    1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 2000  }
  ,
  {
    1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 500  }
  ,
  {
    1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 500  }
};

int sitDown[][19] = {
  {
    1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 1500, 1450, 1550, 500  }
  ,
  {
    1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 1500, 1100, 1230, 2000  }
  ,
  {
    1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 1500, 700, 1230, 2000  }
};

int walk1[][19] = {
  {
    -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, 200  }
  ,
  {
    -1, 1450, 1550, 1850, -1, -1, -1, 1450, 1550, -1, -1, 1350, 1500, 1430, 1490, -1, -1, 1750, 200  }
  ,
  {
    -1, 1540, 1350, -1, 1490, 1480, -1, 1540, 1750, -1, 1540, -1, 1850, 1490, 1480, -1, 1540, -1, 200  }
  ,
  {
    -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, 200  }
  ,
  {
    -1, -1, 1750, 1500, 1430, 1490, -1, -1, 1350, -1, 1450, 1550, 1150, -1, -1, -1, 1450, 1550, 200  }
  ,
  {
    -1, 1540, -1, 1150, 1490, 1480, -1, 1540, -1, -1, 1540, 1750, -1, 1490, 1480, -1, 1540, 1350, 200  }
};

int walk1Prepare[][19] = {
  {
    -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, 500  }
  ,
  {
    -1, -1, -1, 1500, 700, 1230, -1, -1, -1, -1, -1, -1, 1500, 700, 1230, -1, -1, -1, 300  }
  ,
  {
    -1, -1, -1, -1, 1430, 1490, -1, -1, -1, -1, -1, -1, -1, 1430, 1490, -1, -1, -1, 200  }
  ,
  {
    1780, 700, 1230, -1, -1, -1, -1, -1, -1, 1780, 700, 1230, -1, -1, -1, -1, -1, -1, 300  }
  ,
  {
    -1, 1450, 1550, -1, -1, -1, -1, -1, -1, -1, 1450, 1550, -1, -1, -1, -1, -1, -1, 200  }
  ,
  {
    -1, -1, -1, -1, -1, -1, 1220, 700, 1230, -1, -1, -1, -1, -1, -1, 1220, 700, 1230, 300  }
  ,
  {
    -1, -1, -1, -1, -1, -1, -1, 1450, 1550, -1, -1, -1, -1, -1, -1, -1, 1450, 1550, 200  }
};

int walk2[][19] = {
  {
    -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, 200  }
  ,
  {
    -1, -1, 1750, -1, 1450, 1550, -1, -1, 1450, -1, 1500, 1650, -1, -1, 1450, -1, 1450, 1550, 200  }
  ,
  {
    -1, 1540, -1, -1, 1540, 1350, -1, 1500, -1, -1, 1550, 1850, -1, 1500, -1, -1, 1540, 1350, 200  }
  ,
  {
    -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, -1, -1, -1, -1, 1200, -1, 200  }
  ,
  {
    -1, 1450, 1550, -1, -1, 1750, -1, 1500, 1650, -1, -1, 1450, -1, 1500, 1650, -1, -1, 1750, 200  }
  ,
  {
    -1, 1540, 1350, -1, 1540, -1, -1, 1550, 1850, -1, 1500, -1, -1, 1550, 1850, -1, 1540, -1, 200  }
};

int walk2Prepare[][19] = {
  {
    -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, -1, 1450, 1550, 500  }
  ,
  {
    1500, 700, 1230, -1, -1, -1, -1, -1, -1, 1500, 700, 1230, -1, -1, -1, -1, -1, -1, 300  }
  ,
  {
    -1, 1450, 1550, -1, -1, -1, -1, -1, -1, -1, 1450, 1550, -1, -1, -1, -1, -1, -1, 200  }
  ,
  {
    -1, -1, -1, -1, -1, -1, 940, 700, 1230, -1, -1, -1, -1, -1, -1, 940, 700, 1230, 300  }
  ,
  {
    -1, -1, -1, -1, -1, -1, -1, 1450, 1550, -1, -1, -1, -1, -1, -1, -1, 1450, 1550, 200  }
  ,
  {
    -1, -1, -1, 2060, 700, 1230, -1, -1, -1, -1, -1, -1, 2060, 700, 1230, -1, -1, -1, 300  }
  ,
  {
    -1, -1, -1, -1, 1450, 1550, -1, -1, -1, -1, -1, -1, -1, 1450, 1550, -1, -1, -1, 200  }
};
//----------------------------------------------------------------------------------

// Settings
#define BAUD_XBEE 9600
#define BAUD_SSC32 9600
#define TIMEOUT 2000	 //Time in mS before we give up on waiting for data
#define LIGHT_SENSOR 15  //The pin for the light sensor

// Commands
#define STOP_MOVING 0xA0
#define CONTINUE_MOVING 0xA1
#define	TURN_OFF_SERVOS 0xA2
#define CHANGE_GAIT 0xA3
#define PERFORM_GAIT 0xA4
#define STORE_GAIT 0xA5
#define SEND_DIRECTLY 0xA6
#define STOP_SENDING_DIRECTLY 0xA7
#define GET_BRIGHTNESS 0xA8

// Gaits
#define STAND_UP 0
#define SIT_DOWN 1
#define WALK1 2
#define WALK1_PREPARE 3
#define WALK2 4
#define WALK2_PREPARE 5

byte currentGait;	//the index of the gait to be performed
byte currentRot;        //the rotation of the current gait must be between 0 and 5
byte currentMove;       //the current move to be performed on the currentGait
bool moving = false;	//decides if robot is moving or not
unsigned long time;
unsigned long timeSinceMove;  //timer for inbetween moves


//The mapping of each servo. The index is the int with which we refer to the 
//servo and the value at each index is the pin on the motor controller 
byte allServoMapping[18] = {
  12, 13, 14,   //leg 0
  4, 5, 6,     	//leg 1
  0, 1, 2,      //leg 2
  16, 17, 18,   //leg 3
  20, 21, 22,   //leg 4
  24, 25, 26};  //leg 5

int allServoOffset[18] = {
  0, 30, -10,      //leg 0
  0, -50, -150,    //leg 1
  0, 0, 0,         //leg 2
  0, 0, -60,       //leg 3
  0, 10, -60,      //leg 4
  20, -40, 10};    //leg 5

//----------------------------------------------------------------------------------

void setup() {
  pinMode(52,OUTPUT);                      // 5v to the light sensor
  digitalWrite(52, HIGH);
  Serial.begin(9600);                      // Serial to PC
  Serial1.begin(BAUD_XBEE);	           // XBee
  Serial2.begin(BAUD_SSC32);	           // SSC-32
  Serial1.println("Starting Spider");	
  Serial.println("Starting Spider");	
  turnOffServos();
  delay(500);
}


/* Run a gait or recieve a command. If a command is recieved it will reply with 
 an echo once the byte is read. */
void loop() {  
  Serial.println("main loop");

  byte gaitIndex;
  byte rotation;

  if (Serial1.available() > 0) {
    byte inByte = Serial1.read();
    Serial1.write(inByte);  //Echo back
    Serial.println("Got something!");

    switch (inByte) {
    case STOP_MOVING:				
      moving = false;
      break;
    case CONTINUE_MOVING:				
      moving = true;
      break;
    case CHANGE_GAIT:				
      time = millis();
      //wait for gait index and rotation to get there or timeout
      while (Serial1.available() < 2) {
        if (millis() - time > TIMEOUT) return;
      }	
      currentGait = Serial1.read();	//must be value not character
      currentRot = Serial1.read();      //must be between 0 and 5
      break;
    case PERFORM_GAIT:

      time = millis();
      //wait for gait index and rotation to get there or timeout
      while (Serial1.available() < 2) {
        if (millis() - time > TIMEOUT) 
          return;
      }	
      gaitIndex = Serial1.read();	//must be value not character
      rotation = Serial1.read();      //must be between 0 and 5	
      performStoredGait(gaitIndex, rotation, false);
      break;
    case STORE_GAIT:
      time = millis();
      //wait for gait index to arrive
      while (Serial1.available() < 1) {
        if (millis() - time > TIMEOUT) {
          Serial.println("Did not recieve gait Index");
          delay(1000);
          return;
        }
      }	
      gaitIndex = Serial1.read();	//must be value not character
      storeGait(gaitIndex);
      break;
    case SEND_DIRECTLY:				
      sendDirectly();
      break;		
    case TURN_OFF_SERVOS:
      turnOffServos();
      break;
    case GET_BRIGHTNESS:
      Serial1.write(readSensor(LIGHT_SENSOR) >> 2);  //divide by 2 (range 0-255)
      break;
    }
  }
  else if (moving) {  //continue with the walking gait
    performStoredGait(currentGait, currentRot, true);
  }
}

/* Send data directly to the board untill terminate sequence is recieved */
void sendDirectly() {
  Serial.println("sendDirectly");

  byte byteIn;	
  time = millis();
  while (millis() - time < TIMEOUT) {	//will exit upon recieving 0xA6 from PC or timeout
    if (Serial1.available() > 0) {	//recieved data from PC
      byteIn = Serial1.read();
      time = millis();
      if (byteIn == STOP_SENDING_DIRECTLY) {  //terminate
        Serial1.write(byteIn);                //echo back end command
        return;
      }
      Serial2.write(byteIn);
    }
    if (Serial2.available() > 0)	//recieved data from SSC32 board
      Serial1.write(Serial2.read());
  }
  // if the code gets here then it must have timed out
  Serial1.write(0xFF);
  Serial.println("Timed out!");
  delay(1000);
}

/* Does nescesary work to call performGait
 @param thisGaitIndex: The index of the gait to be performed
 @param rotation: The amount of rotation to give this gait
 @param partial: True for perform only the move currentMove, False for full move
 */
void performStoredGait(byte thisGaitIndex, byte rotation, boolean partial) {
  Serial.println("perform stored gait");

  switch(thisGaitIndex) {
  case STAND_UP:
    performGait(standUp, rotation, 3, partial);
    break;
  case SIT_DOWN:
    performGait(sitDown, rotation, 3, partial);
    break;
  case WALK1:
    performGait(walk1, rotation, 6, partial);
    break;
  case WALK1_PREPARE:
    performGait(walk1Prepare, rotation, 7, partial);
    break;
  case WALK2:
    performGait(walk2, rotation, 6, partial);
    break;
  case WALK2_PREPARE:
    performGait(walk2Prepare, rotation, 7, partial);
    break;
  }
}

/* Performs a gait that is rotated by the desired amount 
 @param thisGait: The gait to be performed
 @param rotation: The amount of rotation to give this gait
 @param moves: The number of moves that this gait has
 @param partial: True for perform only the move currentMove, False for full move
 */
void performGait(int thisGait[][19], byte rotations, byte moves, boolean partial) {
  Serial.println("perform gait");
  
  if (partial) {  //assure that sufficient time has passed by for a partial gait
    time = millis() - timeSinceMove;
    int timeLoc = currentMove - 1;
    if (timeLoc == -1)
      timeLoc = moves;
    if (time < thisGait[timeLoc][18])
      return;
  }

  for (byte mov = 0; mov < moves; mov++) { //do each move
  
    if (partial) { //special case to perform only one move per iteration 
      if (currentMove >= moves)
        currentMove = 0;
      mov = currentMove;
    }

    for (int i = 0; i < 18; i++) {  //do each servo position
      if (thisGait[mov][i] == -1) 
        continue;

      int servo = i + 3 * rotations;  //decide which servo 
      while (servo > 17)
        servo -= 18;
      int pos = thisGait[mov][i] + allServoOffset[servo];
      Serial2.print('#');
      Serial2.print(allServoMapping[servo]);
      Serial2.print('P');
      Serial2.print(pos);
    }

    //time
    Serial2.print('T');
    Serial2.print(thisGait[mov][18]);
    Serial2.print('\r');
    
    if (partial) { //we have ended our single itteration 
      timeSinceMove = millis();
      currentMove++;
      return;
    }
    delay(thisGait[mov][18]);
  }
  //If a full gait was performed we can reset currentMove
  currentMove = 0;
}


/* Stored new values for one of the gaits that are stored on the arduino. Should be used
 for temporary gaits for the tough terrains.
 @param thisGaitIndex: The index of the gait to be modified
 */
void storeGait(byte gaitIndex) {
  Serial.println("store gait");

  switch(gaitIndex) {
  case STAND_UP:
    storeOneGait(standUp, 3);
    break;
  case SIT_DOWN:
    storeOneGait(sitDown, 3);
    break;
  case WALK1:
    storeOneGait(walk1, 6);
    break;
  case WALK1_PREPARE:
    storeOneGait(walk1Prepare, 7);
    break;
  case WALK2:
    storeOneGait(walk2, 6);
    break;
  case WALK2_PREPARE:
    storeOneGait(walk2Prepare, 7);
    break;
  }
}

/*
 Will store the new values recieved from the PC into one of the pre-existing gaits.
 Each of the integers will be passed as two bytes. The checksum will be calculated
 by doing a Longitudinal Redundancy Check. Go to: 
 <http://en.wikipedia.org/wiki/Longitudinal_redundancy_check>
 for a reference. */
void storeOneGait(int thisGait[][19], byte numMoves) {
  Serial.println("storing gait");

  delay(20);
  byte checksum = 0;
  byte byteIn;
  int tempInt;

  for (byte move = 0; move < numMoves; move++) {  //each move
    for (byte servoI = 0; servoI < 19; servoI++) {
      time = millis();
      while (Serial1.available() < 2) {  //wait for two bytes of int
        if (millis() - time > TIMEOUT) {
          Serial1.write(0xFF);  //return 0xFF for fail
          return;
        }
      }	
      byteIn = Serial1.read();
      checksum += byteIn;
      tempInt = byteIn << 8;
      byteIn = Serial1.read();
      checksum += byteIn;
      tempInt += byteIn;

      thisGait[move][servoI] = tempInt;
    }
  }

  checksum = (checksum ^ 0xFF) + 1;
  Serial1.write(checksum);
}	

/* Send all servos a 0 command thus cutting power to them */
void turnOffServos() {
  for (int i = 0; i < 18; i++) { //do each servo position
    Serial2.print('#');
    Serial2.print(allServoMapping[i], DEC);
    Serial2.print('P');
    Serial2.print(0, DEC);
    Serial2.print('\r');
  }
}

/* Reads the desired sensor 16 times and returns an approximation of the average */
int readSensor(byte sensor) {
  int sum = 0;
  for (byte i = 0; i < 16; i++)
    sum += analogRead(sensor);
    
  return sum >> 4;  //divide by 16 
}

/* For debugging */
void print2DArray(int thisArray[][19], int rows) {
  Serial.println("------------------------------------");
  for (int r = 0; r < rows; r++) {
    for (int i = 0; i < 19; i++) {
      Serial.print(thisArray[r][i]);
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println("------------------------------------");
}


