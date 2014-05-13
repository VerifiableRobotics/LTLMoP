#include <SoftwareSerial.h>

#define DEBUG false
#define LEFT false
#define RIGHT true

#define Fib 8.2
#define Tib 12.8
#define TT 163.84
#define FF 67.24
#define LEFT 0x01
#define RIGHT 0x02
#define SPEED 100

//neutral position
unsigned int neutral[] = 
  {1500, 1500, 1350, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1270, 1500, -1, 1500, 1500, 1650, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1300, 1700, 1500, 1500};

//low position
int low[] = 
    {1500, 2200, 2000, -1, 1500, 2200, 2000, -1, 1500, 2200, 2000, -1, 1500, 1270, 1500, -1, 1500, 800, 1000, -1, 1500, 800, 1000, -1, 1500, 800, 1000, -1, 1500, 1500, 1500, 1500};
    
//high position
int high[] = 
    {1500, 800, 1000, -1, 1500, 800, 1000, -1, 1500, 800, 1000, -1, 1500, 1270, 1500, -1, 1500, 2200, 2000, -1, 1500, 2200, 2000, -1, 1500, 2200, 2000, -1, 1500, 1500, 1500, 1500};

//nodUp position
unsigned int nodUp[] = 
  {1500, 1500, 1350, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1270, 1200, -1, 1500, 1500, 1650, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, 1500};

//nodDown position
unsigned int nodDown[] = 
  {1500, 1500, 1350, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1270, 1800, -1, 1500, 1500, 1650, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, 1500};

//shake left position
unsigned int shakeL[] = 
  {1500, 1500, 1350, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1200, 1270, 1500, -1, 1500, 1500, 1650, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, 1500};

//shake right position
unsigned int shakeR[] = 
  {1500, 1500, 1350, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1800, 1270, 1500, -1, 1500, 1500, 1650, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, -1, 1500, 1500, 1500, 1500};

  //angle positions (radians)
float rightStart[9] = {
 0, -0.227081937, PI/2 -1.687802641, 
0.15, -0.08, PI/2 -1.52161556, 
 0.7, -0.1,   PI/2 -1.732514015};
 
float rightEnd[9] = {
 -0.5, -0.227081937, PI/2 -1.687802641, //0.203412118
 -0.303844002, -0.08, PI/2 -1.52161556, 
  0.3, -0.1, PI/2 -1.732514015};

float rightStartR[9] = {
 -0.5, -0.227081937, PI/2 -1.687802641, 
-0.303844002, -0.08, PI/2 -1.52161556, 
 0.3, -0.1,   PI/2 -1.732514015};
 
float rightEndR[9] = {
 0, -0.227081937, PI/2 -1.687802641, //0.203412118
 0.15, -0.08, PI/2 -1.52161556, 
  0.7, -0.1, PI/2 -1.732514015};
  

float leftStart[9];
float leftEnd[9];

SoftwareSerial Servos(10, 11); // RX, TX

boolean servoCom(int servo, int val, boolean side, boolean serial = false);

 const float leftTable[9] = {
    24, 25, 26,
    20, 21, 22,
    16, 17, 18 };
  const float rightTable[9] = {
    8, 9, 10,
    4, 5, 6,
    0, 1, 2  };

 const int rightNeutral_val[9] = {
    1500, 1500, 1500, 
    1500, 1500, 1500,
    1500, 1500, 1500 };
    
  const int leftNeutral_val[9] = {
    1500, 1500, 1500, 
    1500, 1500, 1500, 
    1500, 1500, 1500 };
 
 
//set up force sensors
// These constants won't change.  They're used to give names to the pins used:
const int analogInPin0 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin1 = A1;  // Analog input pin that the potentiometer is attached to

int sensorValue0 = 0;        // value read from the pot
int sensorValue1 = 0;
uint8_t outputValue0 = 0;        // value output to the PWM (analog out)
uint8_t outputValue1 = 0;

uint8_t byteArray[4];  //byte array for force sensors to send to ltlmop
int length = 4;    //length of byteArray

  int leftPincer = 29;				//servo number for left pincer
  int rightPincer = 28;				//servo number for right pincer
  int rightPincerStart = 1300;		//starting servo value for right pincer
  int leftPincerStart = 1700;		//starting servo value for left pincer
  int increment = 400;				//the increment value that the pincer will move
  int pincerStatus = 0;				//flag to check if pincer opened or closed
  int object = 0;					//flag to check if object is detected


void setup() {
  Serial.begin(9600);
  Servos.begin(9600);   
  for (int i = 0; i <= 31; i++){
    moveServos(i, neutral[i],0.5);
  }
  byteArray[0] = 0x22;    //set bit 0 (will not change); used for verifying package retrieval
  byteArray[1] = 0x23;    //set bit 1  (will not change); used for verifying package retrieval
}


void loop() {
  //if (Serial.available() > 0){
    // flush the serial rx buffer
   while (Serial.available())
      Serial.read();
   
   while(!Serial.available());
    int inByte = Serial.read();
    
  switch (inByte) {
      case 'a':  //forwards
       for (int i = 0; i < 9; i++) {
          leftStart[i] = -rightStart[i];
          leftEnd[i] = -rightEnd[i];
        }
        forward();
        Serial.write('q');
        Serial.flush();
        break;
      case 'b':  //neutral
        for (int i = 0; i <= 31; i++){
          moveServos(i, neutral[i],0.5);
        }
        Serial.write('q');
        Serial.flush();
        break;
      case 'c':  //clockwise
        right();
        Serial.write('q');
        Serial.flush();

        break;
      case 'd':  //counterclockwise
       left();
        Serial.write('q');
        Serial.flush();
        break;
      case 'e':  //stand up
        for (int i = 0; i <= 31; i++){
          moveServos(i, high[i],0.5);
        }
        Serial.write('q');
        Serial.flush();
        break;
      case 'f':  //sit down
        for (int i = 0; i <= 31; i++){
          moveServos(i, low[i],0.5);
        }
        Serial.write('q');
        Serial.flush();
        break;
        
      case 'g':  //close pincers 
        neutral[leftPincer] = neutral[leftPincer] - increment;
        neutral[rightPincer] = neutral[rightPincer] + increment;
            
        moveServos(leftPincer, neutral[leftPincer],1);
        moveServos(rightPincer, neutral[rightPincer],1);
        Serial.write('q');
        Serial.flush();
        break;
        
       case 'h':  //open pincers
         neutral[leftPincer] = leftPincerStart;
         neutral[rightPincer] = rightPincerStart;
         moveServos(leftPincer, neutral[leftPincer],2);
         moveServos(rightPincer, neutral[rightPincer],2);
         Serial.write('q');
         Serial.flush();
         break; 
        
      case 'i':  //nod
        for (int i = 0; i <= 31; i++){
          moveServos(i, nodDown[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, nodUp[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, nodDown[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, nodUp[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, neutral[i],0.5);
        }
        Serial.write('q');
        Serial.flush();
        break;
        
        case 'j':  //shake
        for (int i = 0; i <= 31; i++){
          moveServos(i, shakeL[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, shakeR[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, shakeL[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, shakeR[i],0.5);
        }
        delay(0.5);
        for (int i = 0; i <= 31; i++){
          moveServos(i, neutral[i],0.5);
        }
        Serial.write('q');
        Serial.flush();
        break;
        
      case 'k':  //check left force sensor
          // read the analog in value:
          sensorValue0 = analogRead(analogInPin0);            
          // map it to the range from 0 to 100 to indicate precentage:
          outputValue0 = (uint8_t)map(sensorValue0, 0, 1023, 0, 100); 
         
         byteArray[2] = 'l'; 				//send char 'l' to indicate left sensor reading
         byteArray[3] = outputValue0; 		//send reading of force sensor
          
          //Serial.print(outputValue0);    //use this to debug, prints value but sends ascii so do not want to send this value
          Serial.write(byteArray,length);  //use this when sending to python script
          Serial.flush();
          break;
          
       case 'l':  //check right force sensor
          // read the analog in value:
          sensorValue1 = analogRead(analogInPin1);            
          // map it to the range from 0 to 100
          outputValue1 = (uint8_t)map(sensorValue1, 1, 1023, 0, 100); 
          
          byteArray[2] = 'r';			//send char 'r' to indicate left sensor reading
          byteArray[3] = outputValue1; 	//send reading of force sensor
          
          //Serial.print(outputValue1);
          Serial.write(byteArray,length);
          Serial.flush();
          break;
    } 
}


void forward() {

  //POD 1 go to beginning
  servoSerial(0, rightStart[0], 80, true);
  servoSerial(1, rightEnd[1] + PI/6, 80, true);
  servoSerial(2, rightEnd[2], 80, true);
  
  servoSerial(6, rightStart[6], 100, true);
  servoSerial(7, rightEnd[7] + PI/6, 80, true);
  servoSerial(8, rightEnd[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftEnd[4] - PI/6, 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //POD 1
  servoSerial(0, rightStart[0], 80, true);
  servoSerial(1, rightStart[1], 80, true);
  servoSerial(2, rightStart[2], 80, true);
  
  servoSerial(6, rightStart[6], 80, true);
  servoSerial(7, rightStart[7], 80, true);
  servoSerial(8, rightStart[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftStart[4], 80, false);
  servoSerial(5, leftStart[5], 80, false);
  
  
  //POD 1 moving
  servoSerial(0, rightEnd[0], 80, true);
  servoSerial(1, rightEnd[1], 80, true);
  servoSerial(2, rightEnd[2], 80, true);
  
  servoSerial(6, rightEnd[6], 80, true);
  servoSerial(7, rightEnd[7], 80, true);
  servoSerial(8, rightEnd[8], 80, true);
  
  servoSerial(3, leftEnd[3], 80, false);
  servoSerial(4, leftEnd[4], 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //delay(100);
  //POD 2 to beginning
  servoSerial(3, rightStart[3], 80, true);
  servoSerial(4, rightEnd[4] + PI/6, 80, true);
  servoSerial(5, rightEnd[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftEnd[1] - PI/6, 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftEnd[7] - PI/6, 80, false);
  servoSerial(8, leftEnd[8], 80, false);  
  
  
  //POD 2
  servoSerial(3, rightStart[3], 80, true);
  servoSerial(4, rightStart[4], 80, true);
  servoSerial(5, rightStart[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftStart[1], 80, false);
  servoSerial(2, leftStart[2], 80, false);
  
  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftStart[7], 80, false);
  servoSerial(8, leftStart[8], 80, false);
  
  //POD 2 moving
  servoSerial(3, rightEnd[3], 80, true);
  servoSerial(4, rightEnd[4], 80, true);
  servoSerial(5, rightEnd[5], 80, true);
  
  servoSerial(0, leftEnd[0], 80, false);
  servoSerial(1, leftEnd[1], 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftEnd[6], 80, false);
  servoSerial(7, leftEnd[7], 80, false);
  servoSerial(8, leftEnd[8], 80, false);

}

void right() {

  //POD 1 go to beginning
  servoSerial(0, rightStartR[0], 80, true);
  servoSerial(1, rightEndR[1] + PI/6, 80, true);
  servoSerial(2, rightEndR[2], 80, true);
  
  servoSerial(6, rightStartR[6], 100, true);
  servoSerial(7, rightEndR[7] + PI/6, 80, true);
  servoSerial(8, rightEndR[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftEnd[4] - PI/6, 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //POD 1
  servoSerial(0, rightStartR[0], 80, true);
  servoSerial(1, rightStartR[1], 80, true);
  servoSerial(2, rightStartR[2], 80, true);
  
  servoSerial(6, rightStartR[6], 80, true);
  servoSerial(7, rightStartR[7], 80, true);
  servoSerial(8, rightStartR[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftStart[4], 80, false);
  servoSerial(5, leftStart[5], 80, false);
  
  
  //POD 1 moving
  servoSerial(0, rightEndR[0], 80, true);
  servoSerial(1, rightEndR[1], 80, true);
  servoSerial(2, rightEndR[2], 80, true);
  
  servoSerial(6, rightEndR[6], 80, true);
  servoSerial(7, rightEndR[7], 80, true);
  servoSerial(8, rightEndR[8], 80, true);
  
  servoSerial(3, leftEnd[3], 80, false);
  servoSerial(4, leftEnd[4], 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //delay(100);
  //POD 2 to beginning
  servoSerial(3, rightStartR[3], 80, true);
  servoSerial(4, rightEndR[4] + PI/6, 80, true);
  servoSerial(5, rightEndR[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftEnd[1] - PI/6, 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftEnd[7] - PI/6, 80, false);
  servoSerial(8, leftEnd[8], 80, false);  
  
  
  //POD 2
  servoSerial(3, rightStartR[3], 80, true);
  servoSerial(4, rightStartR[4], 80, true);
  servoSerial(5, rightStartR[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftStart[1], 80, false);
  servoSerial(2, leftStart[2], 80, false);
  
  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftStart[7], 80, false);
  servoSerial(8, leftStart[8], 80, false);
  
  //POD 2 moving
  servoSerial(3, rightEndR[3], 80, true);
  servoSerial(4, rightEndR[4], 80, true);
  servoSerial(5, rightEndR[5], 80, true);
  
  servoSerial(0, leftEnd[0], 80, false);
  servoSerial(1, leftEnd[1], 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftEnd[6], 80, false);
  servoSerial(7, leftEnd[7], 80, false);
  servoSerial(8, leftEnd[8], 80, false);

}

void left() {
         for (int i = 0; i < 9; i++) {
          leftStart[i] = -rightStartR[i];
          leftEnd[i] = -rightEndR[i];
        }
  
  //POD 1 go to beginning
  servoSerial(0, rightStart[0], 80, true);
  servoSerial(1, rightEnd[1] + PI/6, 80, true);
  servoSerial(2, rightEnd[2], 80, true);
  
  servoSerial(6, rightStart[6], 100, true);
  servoSerial(7, rightEnd[7] + PI/6, 80, true);
  servoSerial(8, rightEnd[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftEnd[4] - PI/6, 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //POD 1
  servoSerial(0, rightStart[0], 80, true);
  servoSerial(1, rightStart[1], 80, true);
  servoSerial(2, rightStart[2], 80, true);
  
  servoSerial(6, rightStart[6], 80, true);
  servoSerial(7, rightStart[7], 80, true);
  servoSerial(8, rightStart[8], 80, true);
  
  servoSerial(3, leftStart[3], 80, false);
  servoSerial(4, leftStart[4], 80, false);
  servoSerial(5, leftStart[5], 80, false);
  
  
  //POD 1 moving
  servoSerial(0, rightEnd[0], 80, true);
  servoSerial(1, rightEnd[1], 80, true);
  servoSerial(2, rightEnd[2], 80, true);
  
  servoSerial(6, rightEnd[6], 80, true);
  servoSerial(7, rightEnd[7], 80, true);
  servoSerial(8, rightEnd[8], 80, true);
  
  servoSerial(3, leftEnd[3], 80, false);
  servoSerial(4, leftEnd[4], 80, false);
  servoSerial(5, leftEnd[5], 80, false);
  
  
  //delay(100);
  //POD 2 to beginning
  servoSerial(3, rightStart[3], 80, true);
  servoSerial(4, rightEnd[4] + PI/6, 80, true);
  servoSerial(5, rightEnd[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftEnd[1] - PI/6, 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftEnd[7] - PI/6, 80, false);
  servoSerial(8, leftEnd[8], 80, false);  
  
  
  //POD 2
  servoSerial(3, rightStart[3], 80, true);
  servoSerial(4, rightStart[4], 80, true);
  servoSerial(5, rightStart[5], 80, true);
  
  servoSerial(0, leftStart[0], 80, false);
  servoSerial(1, leftStart[1], 80, false);
  servoSerial(2, leftStart[2], 80, false);
  
  servoSerial(6, leftStart[6], 80, false);
  servoSerial(7, leftStart[7], 80, false);
  servoSerial(8, leftStart[8], 80, false);
  
  //POD 2 moving
  servoSerial(3, rightEnd[3], 80, true);
  servoSerial(4, rightEnd[4], 80, true);
  servoSerial(5, rightEnd[5], 80, true);
  
  servoSerial(0, leftEnd[0], 80, false);
  servoSerial(1, leftEnd[1], 80, false);
  servoSerial(2, leftEnd[2], 80, false);

  servoSerial(6, leftEnd[6], 80, false);
  servoSerial(7, leftEnd[7], 80, false);
  servoSerial(8, leftEnd[8], 80, false);

}


void servoSerial(int ser, float pos, int time, boolean right) {
  pos = pos * 180/PI;
  pos = 11.111*pos; //+1500.1;
  if (right) {
    pos = pos+rightNeutral_val[ser];
    ser = rightTable[ser];
  }
  else {
    pos = pos+leftNeutral_val[ser];
    ser = leftTable[ser];
  }
  Servos.print('#');
  Servos.print(ser);
  Servos.print(" P");
  Servos.print((int)pos);
  Servos.print(" T");
  Servos.println(time);
}

void moveServos(int servo, int pos, int time){

  Servos.print("#");
  Servos.print(servo);
  Servos.print(" P");
  Servos.print(pos);
  Servos.print(" T");
  Servos.println(time);
  
}
