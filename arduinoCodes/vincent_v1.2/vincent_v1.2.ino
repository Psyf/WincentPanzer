#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

void forward(float, float); 
void reverse(float, float); 
void right(float, float); 
void left(float, float);  
void stop(); 
void mark();

//VERTEX FOR STACK CLASS
struct vertex {
              char command;
              float distance,sp;
              vertex *next;
            };

class stack 
{
  private:
            vertex *head;
            int len;
  public:
          stack(): head(NULL), len(0) {}

          void insert(char com, float d, float s) {

            /*
            switch (com) {
              case 'f': case 'F':
                        com = 'b';
                        break;
              case 'b': case 'B':
                        com = 'f';
                        break;
              case 'r': case 'R': 
                        com = 'l';
                        break;
              case 'l': case 'L': 
                        com = 'r';
                        break;
              case 's': 
                        com = 's'; 
                        break; 
              case 'm': 
                        com = 'm'; 
                        break; 
            }
            */
            vertex *vtx = new vertex();
            vtx->command = com;
            vtx->distance = d;
            vtx->sp = s;
            vtx->next = head;
            head = vtx;
            len++; 
          }

          void pop() {
            vertex *vtx = head;
            head = head->next;
            switch (vtx->command)
            {
              case 'f':
                        forward(vtx->distance, vtx->sp);
                        break;
              case 'b': 
                        reverse(vtx->distance, vtx->sp);
                        break;
              case 'r': 
                        right(vtx->distance, vtx->sp);
                        break;
              case 'l': 
                        left(vtx->distance, vtx->sp);
                        break;
              case 'm': case 'M': 
                        stop(); 
                        mark();
                        break;
              case 's': case 'S': 
                        stop();   
                        delay(1000); 
                        break; 
            }
            len--; 
            delete vtx; 
          }
        int getLen()
        {
          return len; 
        }
         
}; 

//Obvious - Prof Ng, 2017
stack commandStack; 

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

int backTrack = 0; 

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
  statusPacket.params[8]  = forwardDist;
  statusPacket.params[9] = commandStack.getLen(); //backwardDist    
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
//NOTE: Helper functions leftISR and rightSIR removed
//moved Serial moves out of the code to avoid hiccups
//ISR for wheels 
ISR (INT0_vect)
{
  if (dir == FORWARD) 
  {
    leftForwardTicks++;
    //forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_LREV * WHEEL_CIRC); EXPERIMENT1
  }
  else if (dir == BACKWARD)  
  {
    leftBackwardTicks++;
    //backwardDist = (unsigned long) ((float) leftBackwardTicks / COUNTS_PER_LREV * WHEEL_CIRC); EXPERIMENT1
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

//INCOMPLETE: TO DO 
void mark()
{
  //LED TURN ON
  delay(2000); 
  //TURN LED OFF  
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
  
  analogWrite(LF, val);
  analogWrite(RF, val);
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

//  if (dist == 0) deltaDist = 9999999; 
  //else deltaDist = dist; 

  if (dist > 0)
  {
    deltaDist = dist; 
  }
  else deltaDist = 999; 
    
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
  delay(1000); 
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
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
        commandStack.insert('b', command->params[0], command->params[1]); 
      break;
     case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
        commandStack.insert('f', command->params[0], command->params[1]);
      break;
     case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
        commandStack.insert('l', command->params[0], command->params[1]);
      break;
     case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
        commandStack.insert('r', command->params[0], command->params[1]);
      break;
     case COMMAND_STOP:
        sendOK();
        stop(); 
        commandStack.insert('s', 0, 0); 
      break;
     case COMMAND_GET_STATS:
        sendOK();
        sendStatus(); 
      break;
     case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]); 
      break;
     case COMMAND_MARK: 
        mark(); //MARK DEBUG: TO REMOVE  
        commandStack.insert('m', 0, 0);
      break; 
     case COMMAND_END: 
        backTrack = 1; 
      break; 
    default:
      sendBadCommand();
  }
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

int inCommand = 0; 

void loop() {

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
  {
    backTrack = 0; 
    handlePacket(&recvPacket);
  }
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
  {  inCommand = 1; 
      if (dir == FORWARD)
      {
          forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_LREV * WHEEL_CIRC); 
          if (forwardDist >= newDist)
          {
            stop();  
            deltaDist = 0; 
            newDist = 0; 
            inCommand = 0;  
          }
      }
      else if (dir == BACKWARD)
      {
              backwardDist = (unsigned long) ((float) leftBackwardTicks / COUNTS_PER_LREV * WHEEL_CIRC);
              if (backwardDist >= newDist)
              {
                deltaDist = 0; 
                newDist = 0; 
                stop();
                inCommand = 0;  
              }
              else if (dir == STOP)
              {
                deltaDist = 0; 
                newDist = 0; 
                stop(); 
                inCommand = 0; 
              }
     }
     else if (dir == LEFT && leftBackwardTicksTurns >= targetTicks)
     {         
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
      inCommand = 0;  
     }
    else if (dir == RIGHT && rightBackwardTicksTurns >= targetTicks)
    {
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
      inCommand = 0; 
    }
    else if (dir == STOP)
    {
      targetTicks = 0; 
      deltaTicks = 0; 
      stop(); 
      inCommand = 0; 
    }  
  }
  
  if (backTrack && !inCommand)
  { 
    if (commandStack.getLen() > 0)
      commandStack.pop(); 
    else backTrack = 0; 
  }
}
