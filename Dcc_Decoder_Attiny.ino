#include "Configs.h"
#include <EEPROM.h>

/*Macros */
#define HIGH_VAL(x) ((unsigned)(x) >> 8)
#define LOW_VAL(x)  ((x) & 0xFF)

unsigned long pulseWidth;
unsigned char dccState;
unsigned char preambleCount;
boolean debug;

//DCC Bit States
#define PREAMBLE_STATE    0
#define WAIT_INIT_BIT	  1  // Init packet bit
#define FIRST_BYTE_STATE  2
#define WAIT_START1_BIT   3
#define SECOND_BYTE_STATE 4
#define WAIT_START2_BIT   5
#define THIRD_BYTE_STATE  6
#define WAIT_START3_BIT   7
#define FORTH_BYTE_STATE  8
#define WAIT_START4_BIT   9
#define FIFTH_BYTE_STATE  10
#define WAIT_START5_BIT   11

#define SPEED_DIR_CMD 0b01000000  // 01XXXXXX   (010=Reverese   011=Forward)
#define SPEED_DIR_CMD2  0b01100000  // 01XXXXXX   (010=Reverese   011=Forward)

unsigned char receivedBit;
unsigned char bitsReceived;
unsigned char buffer[5];
unsigned char direction;
unsigned char address;
char bytesReceived;

typedef struct{
  char CV1;     // Address
  char CV2;   // VStart
  char CV3;   // Acceleration Rate
  char CV4;   // Deceleration Rate
  char CV5;   // VHigh
  char CV6;   // VMid (Apply only on 128 Speed Step)
  char CV7;   // Manufacturer Version
  char CV8;   // Manufactured ID
  char CV11;  // Packet Time out Value to stop when not receiving a DCC packet address to the decoder
  char CV19;  // Consist Address
  //TCV29 CV29; // Configuration Variable
  char Speed;
  //TLocoFlags Flags;
  //TFuncGroup  Func1;
  //TFuncGroup  Func2;
}  TLocoData;
  
//Simulation variables
int counter=0;
//Test Data-1 111111111111 0 00000011 0 01100100 0 01000011 1
//Test Data-2 111111111111 0 00000011 0 01000100 0 01000011 1 
int bits[80]={60,60,60,60,60,60,60,60,60,60,60,60,100,
        100,100,100,100,100,100,60,60,100,
        100,60,60,100,100,60,100,100,100,
        100,60,100,100,100,100,60,60,60,
        
        60,60,60,60,60,60,60,60,60,60,60,60,100,
        100,100,100,100,100,100,60,60,100,
        100,60,100,100,100,60,100,100,100,
        100,60,100,100,100,100,60,60,60};
int cvValues[] = {3}

void loadDefaultCVs(){
  TLocoData.CV1 = cvValues[0];
}

void setup()
{ 
  pinMode(DCCInputPin, INPUT);
  pinMode(MOTOR_FORWARD_PIN, OUTPUT);  
  pinMode(MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(FRONT_LED_PIN, OUTPUT);
  pinMode(REAR_LED_PIN, OUTPUT);
  
  dccState = 0;
  preambleCount = 0;
  bitsReceived = 0;
  address = 3;
  
  if (EEPORM.read(0))
  {
	  
  }
  
  
}

void checkLights(unsigned char direction){
  if(direction==1){
    digitalWrite(FRONT_LED_PIN, HIGH);
    digitalWrite(REAR_LED_PIN, LOW);
  }else if(direction==0){
    digitalWrite(FRONT_LED_PIN, LOW);
    digitalWrite(REAR_LED_PIN, HIGH);
  }
}

void setSpeed(unsigned char speed, unsigned char direction){
  int PWM;
  int temp;
  
  //28 Speed PWM = (LocoData.Speed*(LocoData.CV5-LocoData.CV2)/28)  + LocoData.CV2;  
  unsigned char calculatedSpeed = 0;
  unsigned char minimumPwm = 0;
  unsigned char transformedSpeed = 0;
  unsigned char speedRange = 255 - minimumPwm;
  
  if (speed==0)
  {
    analogWrite(MOTOR_FORWARD_PIN,0);
    analogWrite(MOTOR_REVERSE_PIN,0);   
  }else{  
	calculatedSpeed = (speed & B00001111) << 1;
    
    if(bitRead(speed,4)) {
		calculatedSpeed++;  // Sets the LSb of the Speed
	}
	
    if (calculatedSpeed > 3)
    {
		temp = 151 * speedRange;
		PWM = (calculatedSpeed-4) * HIGH_VAL(temp);
      	PWM = (PWM >> 4);
		transformedSpeed = (LOW_VAL(PWM) + minimumPwm);
	}
	
    //transformedSpeed = ((speed - 1)/(float)28)*speedRange + minimumPwm;    
    if (direction==1)
    {
      analogWrite(MOTOR_FORWARD_PIN,transformedSpeed);
      analogWrite(MOTOR_REVERSE_PIN,0);
    } 
    else if(direction==0)
    {
      analogWrite(MOTOR_FORWARD_PIN,0);
      analogWrite(MOTOR_REVERSE_PIN,transformedSpeed);
    }
  }
}

void analyzeDCC(){
   
  char TempCommand; // To help decoding DCC commands
  char checkSum;
  char i;
  
  checkSum=0;
  for(i=0; i<bytesReceived-1;i++){
    checkSum ^= buffer[i];
  } 
  
  if (preambleCount <= 20){
	  if ((buffer[0] == address) && (checkSum == buffer[i])){
    
	  TempCommand = buffer[1] & B11100000;
       
		switch (TempCommand){
		case SPEED_DIR_CMD: case SPEED_DIR_CMD2:
		  unsigned char speedCommand = buffer[1] & B00111111;
		  direction = (buffer[1] & B00100000) >> 5;
     
		  //Check the lights
		  checkLights(direction);	
		  setSpeed(speedCommand, direction);
		  break;
		}
    
		/*case CV_ACCESS_CMD:{
		  setCVValues(buffer[2], buffer[3]);
		}*/
	  }
  }
  
  
}

void setCVValues(char cvAddr, char CV){
  switch (cvAddr)
  {
    case 0: address = CV; break;  
  }
}

void writeToEEPROM(char dataAdd, char data){
  //EEPROM.update(dataAdd, data); 
}

void loop()
{
  			
  //Simulation needed 
  pulseWidth = pulseIn(DCCInputPin,HIGH); //bits[counter]; //
  
  if(pulseWidth > DCC_ONE_LOW && pulseWidth < DCC_ONE_HIGH){
    receivedBit = 1;
  }else{
    receivedBit = 0;
  }
  
  switch(dccState){
    case PREAMBLE_STATE:
      if(receivedBit==1){
        preambleCount++;
        
        if(preambleCount==10){
          dccState++;
        }
      }
    break;  
    
    case WAIT_INIT_BIT:     
      if (receivedBit==0)
      {
        dccState++;
        bytesReceived = 0;
        buffer[0]=0;
        buffer[1]=0;
        buffer[2]=0;
        buffer[3]=0;
        buffer[4]=0;            
      }
    break;
    
    case FIRST_BYTE_STATE:
      buffer[0] |=  receivedBit;
      bitsReceived++;
      if(bitsReceived<8){
        buffer[0] = (buffer[0] << 1);               
      }else{
        dccState++;
        bitsReceived=0;
        bytesReceived++;
      }
    break;  
    
    case WAIT_START1_BIT:
      if(receivedBit==0){
        dccState++;
      }else{
        dccState=0;
        preambleCount=0;
      }
    
    break;
    
    case SECOND_BYTE_STATE:
    buffer[1] |=  receivedBit;
    bitsReceived++;    
    if(bitsReceived<8){
      buffer[1] = (buffer[1] << 1);
    }else{
      dccState++;
      bitsReceived=0;
      bytesReceived++;
    }
    
    break;
    
    case WAIT_START2_BIT:
      if (receivedBit==0){
        dccState++;
      }else{
        dccState=0;
        preambleCount=0;
      }
    break;
    
    case THIRD_BYTE_STATE:
      buffer[2] |=  receivedBit;
      bitsReceived++;
      if(bitsReceived<8){
        buffer[2] = (buffer[2] << 1);
      }else{
        dccState++;
        bitsReceived=0;
        bytesReceived++;
      }
    break;
    
    case WAIT_START3_BIT : 
      if (receivedBit == 0) {
        dccState++;
        } else {
        analyzeDCC();
        dccState = 0; // An error on the protocol occurred so restarts the packet
        preambleCount=0;
      }
    break;
        
    case FORTH_BYTE_STATE : 
      buffer[3] |= receivedBit;
      bitsReceived++;
      if (bitsReceived < 8) {
        buffer[3] = buffer[3] << 1;
        } else {
        dccState++; // Checks for the 0 bit between bytes
        bitsReceived =0;
        bytesReceived++;
      }
    break;
        
    case WAIT_START4_BIT : 
      if (receivedBit == 0) {
        dccState++;
        } else {
        analyzeDCC();
        dccState = 0; // An error on the protocol occurred so restarts the packet
        preambleCount=0;
      }
    break;    
    
    case FIFTH_BYTE_STATE : 
      buffer[4] |=receivedBit;
      bitsReceived++;
      if (bitsReceived < 8) {
        buffer[4] = buffer[4] << 1;
        } else {
        dccState++; // Checks for the 0 bit between bytes
        bitsReceived =0;
        bytesReceived++;
      }
      break;
    
    case WAIT_START5_BIT : 
      if (receivedBit == 1) { // End Packet Bit
        analyzeDCC();
        dccState = 0;
        } else {
        dccState =0; // Resets the Packet State because It didn't received and 1 end bit
        preambleCount=0;
      }
    break;
  
    default : dccState = 0;
	}
  /*counter++;
  
  if(counter==79){
    counter=0;
  }*/
    
}
