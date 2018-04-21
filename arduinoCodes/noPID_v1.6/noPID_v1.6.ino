#include <PID_v1.h>
#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

#define MAXIRF 300
#define LEFTIR A0
#define BACKIR A1
#define RIGHTIR A2

#define L_RADIUS 10 
#define R_RADIUS 9.7
//Function prototypes
void forward(float, float); 
void reverse(float, float); 
void right(float, float); 
void left(float, float); 
void stop(); 
void mark();

//vertext for stack class
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
                    break; 
            }
            len--; 
            delete vtx; 
        }

        void undo() {
            if (head != NULL) {
                vertex *vtx = head;
                head = head->next;
                delete vtx;
                len--; 
            }
        }

        int getLen()
        {
            return len; 
        }

        char getCommand() {
            if (head != NULL) {
                return head->command;            
            }
            return 'e';
        }

        int getAngle() {
          if (head != NULL) {
            return (int)head->distance;
          }
          return -1;
        }

}; 

stack commandStack; //history
volatile int inCommand = 0; //are we executing a command rn? 
int backTrack = 0;  //are we backtracking?  
volatile char nextCommand; volatile int nextAngle; //the next command in our stack
volatile char isGap;
volatile int gSpeed; //global speed, for motor correction
volatile int gAngle; //global angle, for turn brake
volatile int gDist;

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
    statusPacket.params[9] = backwardDist;     
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

void enableLEDs()
{
    DDRB |= 0b00110000; 
    PORTB &= 0b11001111; 
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
        forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_LREV * WHEEL_CIRC); 
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

    if(speed > 100)  //capped at 95 for preventing overflow
        speed = 100;

    return (int) ((speed / 100.0) * 255.0);
}

void mark()
{
    PORTB |= 0b00110000;  //LED TURN ON
    delay(2000); 
    PORTB &= 0b11001111; //TURN LED OFF  
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Vincent will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
    dir = FORWARD; 
    gSpeed = (speed < 95) ? pwmVal(speed) : pwmVal(95);
    gDist = (int) dist;
    if (gDist > 0) deltaDist = dist; 
    else { deltaDist = 0; newDist = forwardDist; dir == STOP; return; }

    newDist = forwardDist + deltaDist; 

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.

    analogWrite(LF, gSpeed);
    analogWrite(RF, gSpeed);
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
    gSpeed = (speed < 95) ? pwmVal(speed) : pwmVal(95); 
    gDist = (int) dist;
    if (gDist > 0)
    {
        deltaDist = dist; 
    }
    else { deltaDist = 0; newDist = forwardDist; dir == STOP; return; } //NO INFINITE DISTANCE NEEDED 

    newDist = backwardDist + deltaDist; 

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    analogWrite(LR, gSpeed);
    analogWrite(RR, gSpeed);
    analogWrite(LF, 0);
    analogWrite(RF, 0);
}

unsigned long computeDeltaTicks(float ang)
{   
    unsigned long ticks;
    float rads = ang * (PI/180.0); 
    if (dir == LEFT) 
    {
        ticks = (unsigned long) ( (L_RADIUS*rads/WHEEL_CIRC)*COUNTS_PER_LREV);
    }
    else if (dir == RIGHT)
    {
        ticks = (unsigned long) ( (R_RADIUS*rads/WHEEL_CIRC)*COUNTS_PER_LREV);
    }
    return ticks; 
}

// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed)
{
    gAngle = (int) ang;
    dir = LEFT; 
    int val = pwmVal(speed); 

    if (ang > 0) deltaTicks = computeDeltaTicks(ang); 
    else { deltaTicks = 0; dir = STOP; targetTicks = leftBackwardTicksTurns; return;}

    targetTicks = leftBackwardTicksTurns + deltaTicks; 

    analogWrite(LR, val);
    //analogWrite(RF, val);
    analogWrite(LF, 0);
    //analogWrite(RR, 0);
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn right indefinitely.
void right(float ang, float speed)
{ 
    gAngle = (int) ang;
    dir = RIGHT; 
    int val = pwmVal(speed); 

    if (ang > 0) deltaTicks = computeDeltaTicks(ang); 
    else { deltaTicks = 0; dir = STOP; targetTicks = leftBackwardTicksTurns; return;}

    targetTicks = rightBackwardTicksTurns + deltaTicks; 

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    //analogWrite(RR, val);
    analogWrite(RR, val);
    analogWrite(RF, 0);
}

// Stop Vincent. To replace with bare-metal code later.
void stop()
{
    deltaDist = 0;
    newDist = 0;
    targetTicks = 0; 
    deltaTicks = 0; 
    if (dir == FORWARD)
    {
        analogWrite(LF, 0); 
        analogWrite(RF, 0);
        analogWrite(LR, 225);
        analogWrite(RR, 225); 
        delay(37.9*sqrt(gDist)); 
    }
    else if (dir == BACKWARD)
    {
        analogWrite(LR, 0);
        analogWrite(RR, 0);
        analogWrite(LF, 225); 
        analogWrite(RF, 225);
        delay(31.6*sqrt(gDist));    
    }
    else if (dir == RIGHT)
    {
        analogWrite(RR,0);
        analogWrite(LF,0);
        //analogWrite(RF,255);
        analogWrite(LR,255);
        delay(5);
    }
    else if (dir == LEFT)
    {
        analogWrite(LR,0);
        analogWrite(RF,0);
        analogWrite(LF,255);
        //analogWrite(RR,255);
        delay(10);
    }
    analogWrite(LF,0);
    analogWrite(RF,0);
    analogWrite(LR, 0);
    analogWrite(RR, 0); 
    delay(500);
    dir = STOP;  
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
    //COMMENT FOR DEBUG (getting g)
    clearCounters(); 
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
            stop();  
            //commandStack.insert('s', 0, 0);
            sendOK(); 
            break;
        case COMMAND_GET_STATS:
            sendStatus();
            sendOK();
            break;
        case COMMAND_CLEAR_STATS:
            clearOneCounter(command->params[0]);
            sendOK();
            break;
        case COMMAND_MARK: 
            mark();  
            commandStack.insert('m', 0, 0);
            sendOK(); 
            break; 
        case COMMAND_END: 
            backTrack = 1;
            sendOK(); 
            break; 
        case COMMAND_POP:  
            commandStack.undo(); 
            sendOK();
            char message[100]; 
            sprintf(message, "stackSize = %d\n", commandStack.getLen()); 
            sendMessage(message); 
            break; 
        case COMMAND_INSERT:  
            if (command->params[0] == 'f') commandStack.insert('b', command->params[1], 95); 
            else if (command->params[0] == 'r') commandStack.insert('l', command->params[1], 95); 
            else if (command->params[0] == 'l') commandStack.insert('r', command->params[1], 95); 
            else if (command->params[0] == 'm') commandStack.insert('m', 0, 0); 
            else if (command->params[0] == 'b') commandStack.insert('f', command->params[1], 95);
            sendOK();
            sprintf(message, "stackSize = %d\n", commandStack.getLen()); 
            sendMessage(message);
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
    enableLEDs(); 
    initializeState();
    pinMode(LEFTIR,INPUT);
    pinMode(BACKIR,INPUT);
    pinMode(RIGHTIR,INPUT);
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

void correctMotors() {
    if (dir == FORWARD)
    {
        if (leftForwardTicks - rightForwardTicks > 2)
        {    
            analogWrite(LF, gSpeed*1.05);
            analogWrite(RF,gSpeed*0.95);
        }
        else if (rightForwardTicks-leftForwardTicks > 2)
        {
            analogWrite(LF, gSpeed*0.95);
            analogWrite(RF, gSpeed*1.05);
        }
    }
    else if (dir == BACKWARD)
    {
        if (leftBackwardTicks - rightBackwardTicks > 2)
        {
            analogWrite(LR, gSpeed*1.05);
            analogWrite(RR, gSpeed*0.95);
        }

        else if (rightBackwardTicks - leftBackwardTicks > 2)
        {
            analogWrite(LR, gSpeed*0.95);
            analogWrite(RR, gSpeed*1.05);
        }
    }
    /*else if (dir == LEFT) //harcoded to 70 speed
    {
        if (leftBackwardTicksTurns - rightForwardTicksTurns > 2)
        {
            analogWrite(RF,204);
            analogWrite(LR,185);
        } 
        else if (leftBackwardTicksTurns - rightForwardTicksTurns < -2)
        {
            analogWrite(LR,204); 
            analogWrite(RF,179);
        }
        else {
            analogWrite(LR,185);
            analogWrite(RF,179);
        }
    }
    else if (dir == RIGHT)
    {
        if (leftForwardTicksTurns - rightBackwardTicksTurns > 2)
        {
            analogWrite(RR,204);
            analogWrite(LF,179);
        } 
        else if (leftForwardTicksTurns - rightBackwardTicksTurns < -2)
        {
            analogWrite(LF,204);
            analogWrite(RR,179);
        }
        else {
            analogWrite(LF,179);
            analogWrite(RR,179);
        }
    }*/
}

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
    {
        if(result == PACKET_BAD)
        {
            sendBadPacket();
        }
        else
        {
            if(result == PACKET_CHECKSUM_BAD)
            {
                sendBadChecksum();
            } 
        }
    }

    //The code below ensures precise distance movement and turning. 
    //NOTE: We put counterCleanup in stop(). Reprucussions? 
    if (deltaDist > 0)
    {  
        inCommand = 1; 
        if (dir == FORWARD)
        {
            if (forwardDist >= newDist)
            {
                stop(); 
                inCommand = 0;  
            }
            else 
            {
                correctMotors();
            }
        }
        else if (dir == BACKWARD)
        {
            if (backwardDist >= newDist)
            {
                stop();
                inCommand = 0;  
            }
            else
            {
                correctMotors();
            }
        }
        else if (dir == STOP)
        {
            stop(); 
            inCommand = 0; 
        }
    }

    if (deltaTicks > 0)
    {
        inCommand = 1; 
        if (dir == LEFT && leftBackwardTicksTurns >= targetTicks)
        {         
            stop(); 
            inCommand = 0;  
        }
        else if (dir == LEFT)
        {
            correctMotors();
        }
        else if (dir == RIGHT && rightBackwardTicksTurns >= targetTicks)
        { 
            stop(); 
            inCommand = 0; 
        }
        else if (dir == RIGHT)
        {
            correctMotors();
        }
        else if (dir == STOP)
        { 
            stop(); 
            inCommand = 0; 
        }  
    }

    if (backTrack && !inCommand)
    {   
        isGap = (char) 0;
        if (commandStack.getLen() > 0)
        {
            clearCounters(); 
            commandStack.pop();
            nextCommand = commandStack.getCommand();
            nextAngle = commandStack.getAngle();
        }
        else backTrack = 0;
    }
    else if (backTrack && dir == BACKWARD)
    { 
      if (analogRead(BACKIR) > 500)
      {
        backwardDist = newDist;
      }
      /*else if (backwardDist > 0.6*newDist && nextCommand == 'l' && nextAngle == 90 && analogRead(LEFTIR) > 120)
      {
        newDist = backwardDist + 10;
      }
      else if (backwardDist > 0.6*newDist && nextCommand == 'r' && nextAngle == 90 && analogRead(RIGHTIR) > 120)
      {
        newDist = backwardDist + 10;
      }
    }
    
    else if (backTrack && dir == FORWARD) 
    {
      if (nextCommand == 'l' && nextAngle == 90 && analogRead(LEFTIR) > 120)
      {
        newDist = forwardDist + 10;
      }
      else if (nextCommand == 'r' && nextAngle == 90 && analogRead(RIGHTIR) > 120)
      {
        newDist = forwardDist + 10;
      }*/
    }
    
    else if (backTrack && isGap == (char)0) {
      if (nextCommand == 'l' && analogRead(LEFTIR)<100) 
            if (dir == FORWARD && newDist-forwardDist < 0.2*gDist ) {
                newDist = forwardDist + 25;
                isGap = (char)1;
            }
            else if (dir == BACKWARD && newDist-backwardDist < 0.2*gDist) {
                newDist = backwardDist + 15;
                isGap = (char)1;
            }
    }
    else if (nextCommand == 'r' && analogRead(RIGHTIR)<100) {
        if (dir == FORWARD && newDist-forwardDist < 0.2*gDist ) {
            newDist = forwardDist + 25;
            isGap = (char)1;
        }
        else if (dir == BACKWARD && newDist-backwardDist < 0.2*gDist) {
            newDist = backwardDist + 15;
            isGap = (char)1;
        }
    }
}
