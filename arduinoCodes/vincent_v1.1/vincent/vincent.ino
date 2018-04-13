#include <PID_v1.h>

#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0, 
  FORWARD = 1, 
  BACKWARD = 2, 
  LEFT = 3, 
  RIGHT = 4, 
} TDirection; 

volatile TDirection dir = STOP; 
/*
 * Vincent's configuration constants
 */

// Number of ticks per revolution from the wheel encoder.
//Values below needed for dist and angle functions
// NOTE: Left and Right are slightly different 
#define COUNTS_PER_RREV      195
#define COUNTS_PER_LREV      195
#define WHEEL_CIRC          20.8
#define VINCENT_LENGTH      16
#define VINCENT_BREADTH     6
float vincentDiagonal = 0.0; 
float vincentCirc = 0.0; 

// Motor control pins
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

/*
 *    Vincent's State Variables
 */

// Store the ticks from Vincent's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftBackwardTicks; 
volatile unsigned long rightBackwardTicks; 

//store the ticks while turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftBackwardTicksTurns;
volatile unsigned long rightBackwardTicksTurns; 
  
// Store the revolutions on Vincent's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long backwardDist;
volatile unsigned long newDist; 
volatile unsigned long deltaDist;  
volatile unsigned long deltaTicks; 
volatile unsigned long targetTicks;

// PID Stuff
double kp=10, ki=0, kd=0; 
volatile double inputLeft, inputRight, outputLeft, outputRight, setPoint=0;
PID myLeftPID(&inputLeft, &outputLeft, &setPoint, kp, ki, kd, DIRECT); 
PID myRightPID(&inputRight, &outputRight, &setPoint, kp, ki, kd, DIRECT); 

/*
 * 
 * Vincent Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  TPacket statusPacket; 
  statusPacket.packetType = PACKET_TYPE_RESPONSE; 
  statusPacket.command = RESP_STATUS; 
  statusPacket.params[0] = leftForwardTicks; 
  statusPacket.params[1] = rightForwardTicks; 
  statusPacket.params[2] = leftBackwardTicks; 
  statusPacket.params[3] = rightBackwardTicks; 
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftBackwardTicksTurns;
  statusPacket.params[7] = rightBackwardTicksTurns;
  statusPacket.params[8]  = forwardDist; //forwardDist;
  statusPacket.params[9] = newDist; //backwardDist;   
  sendResponse(&statusPacket); 
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;   
}

// Functions to be called by INT0 and INT1 ISRs.
//moved Serial moves out of the code to avoid hiccups
//ISR for wheels 
ISR (INT0_vect)
{
  if (dir == FORWARD) 
  {
    leftForwardTicks++;
    forwardDist = (unsigned long) (((float) leftForwardTicks / COUNTS_PER_LREV) * WHEEL_CIRC); 
  }
  else if (dir == BACKWARD)  
  {
    leftBackwardTicks++;
    backwardDist = (unsigned long) ((float) leftBackwardTicks / COUNTS_PER_LREV * WHEEL_CIRC);
  }
  else if (dir == LEFT) leftBackwardTicksTurns++;
  else if (dir == RIGHT) leftForwardTicksTurns++; 
 
}

ISR (INT1_vect)
{
  if (dir == FORWARD) rightForwardTicks++;
  else if (dir == BACKWARD) rightBackwardTicks++;
  else if (dir == LEFT) rightForwardTicksTurns++;
  else if (dir == RIGHT) rightBackwardTicksTurns++; 
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  EIMSK = 0b11; 
  EICRA = 0b1010; //Open COllector Circuit
                  //faster read with falling edge 
}


/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Vincent's motor drivers.
 * 
 */

// Set up Vincent's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Vincent's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
    
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Vincent will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD; 
  int val = pwmVal(speed);


  if (dist == 0) deltaDist = 9999999; 
  else deltaDist = dist; 

  newDist = forwardDist + deltaDist; 

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val-outputRight);
  analogWrite(RF, val-outputLeft);
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// Reverse Vincent "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Vincent will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD; 
  int val = pwmVal(speed);

  if (dist == 0) deltaDist = 9999999; 
  else deltaDist = dist; 

  newDist = backwardDist + deltaDist; 

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang/360.0)*(vincentCirc/WHEEL_CIRC)*COUNTS_PER_LREV/2); //last /2 by me
  //unsigned long ticks = (unsigned long) ((ang*PI/180.0)*3.85)/WHEEL_CIRC*COUNTS_PER_LREV; 
  return ticks; 
}

// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT; 
  int val = pwmVal(speed);

  if (ang == 0) deltaTicks = 9999999; 
  else deltaTicks = computeDeltaTicks(ang); 

  targetTicks = leftBackwardTicksTurns + deltaTicks; 
  
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT; 
  int val = pwmVal(speed);

  if (ang == 0) deltaTicks = 9999999; 
  else deltaTicks = computeDeltaTicks(ang); 

  targetTicks = rightBackwardTicksTurns + deltaTicks; 

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Vincent. To replace with bare-metal code later.
void stop()
{
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  outputLeft = 0; 
  outputRight = 0;
}
/*
 * Vincent's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  leftBackwardTicks=0;
  rightForwardTicks=0;
  rightBackwardTicks=0;
  leftForwardTicksTurns=0;
  leftBackwardTicksTurns=0; 
  rightForwardTicksTurns=0; 
  rightBackwardTicksTurns=0;  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  backwardDist=0;
}

// Clears one particular counter
//NOTE: Have to work on this. Currently unfinished. 
void clearOneCounter(int which)
{
  clearCounters(); 
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  if (command->command == COMMAND_FORWARD)
  {
    sendOK();
    forward((float) command->params[0], (float) command->params[1]);
  }
  else if (command->command == COMMAND_REVERSE)
  {
     sendOK();
     reverse((float) command->params[0], (float) command->params[1]);
  }
  else if (command->command == COMMAND_TURN_RIGHT)
  {
        sendOK();
        right((float) command->params[0], (float) command->params[1]);    
  }
  else if (command->command == COMMAND_TURN_LEFT)
  {
        sendOK();
        left((float) command->params[0], (float) command->params[1]);    
  }
  else if (command->command == COMMAND_STOP)
  {
        sendOK();
        dir = STOP;
        stop();     
  }
  else if (command->command == COMMAND_GET_STATS)
  {
        sendOK();
        sendStatus();    
  }
  else if (command->command == COMMAND_CLEAR_STATS)
  {
        sendOK();
        clearOneCounter(command->params[0]);     
  }
  else 
  {
    sendBadCommand();    
  }
  
  /*switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
     case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
     case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
     case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
     case COMMAND_STOP:
        sendOK();
        stop(); 
      break;
     case COMMAND_GET_STATS:
        sendOK();
        sendStatus(); 
      break;
     case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]); 
      break;
            
    default:
      sendBadCommand();
  }*/
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  myLeftPID.SetMode(AUTOMATIC); 
  myRightPID.SetMode(AUTOMATIC);
  //have to increase sample time? (currently at 200ms)


  vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH * VINCENT_BREADTH));
  vincentCirc = PI * vincentDiagonal;  
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
  
  //The code below ensures precise distance movement and turning. 
  //NOTE: Can we put the counter cleanup in stop()?
  //NOTE: Debug statements inside!
  char message[100]; 
  if (deltaDist > 0)
  {
    //See if PID is changing 

    //sendStatus();  
    //if (outputLeft > 0 || outputRight > 0) sendStatus(); 
      if (dir == FORWARD)
      {
          inputLeft = leftForwardTicks-rightForwardTicks; 
          inputRight = 0 - leftForwardTicks + rightForwardTicks; 
          myLeftPID.Compute();
          myRightPID.Compute();
          if (forwardDist < newDist) forward((int)(newDist-forwardDist), 100);  //NO CONTROL OVER THE SPEED
          else if (forwardDist >= newDist)
          {
            deltaDist = 0; 
            newDist = 0; 
            stop(); 
          }
      }
      else 
      {
          if (dir == BACKWARD)
          {
              if (backwardDist >= newDist)
              {
                deltaDist = 0; 
                newDist = 0; 
                stop(); 
              }
              else if (dir == STOP)
              {
                deltaDist = 0; 
                newDist = 0; 
                stop(); 
              }
          }
      
      }
 
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT && leftBackwardTicksTurns >= targetTicks)
    {         
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
    }
    else if (dir == RIGHT && rightBackwardTicksTurns >= targetTicks)
    {
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
    }
    else if (dir == STOP)
    {
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
    }  
  }
}
