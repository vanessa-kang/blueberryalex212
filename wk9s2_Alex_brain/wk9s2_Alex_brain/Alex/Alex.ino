#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include <stdarg.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  REVERSE = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      195

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21

// Turning radius of Alex, for turns
#define RADIUS              5.5

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

// Constants for PID
#define IDEAL_SPEED         10  //Roughly 195 counts (1 rev) per 2 seconds
#define BASE_POWER          125 //Starting power level
#define PID_INTERVAL        100 //Period between each PID value update
#define TURNING_DIV         1.75  //Divide by this constant for turns to prevent turning too fast

//P,I and D coefficients
#define kp                  4
#define ki                  1
#define kd                  3

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long lfT;
volatile unsigned long rfT;

//Variables for PID
int pidpwr = 150, pidpwrR = 150;
unsigned long startFlag = 0, startFlagR = 0;
float startPID = 0, totalTime = 0, startPIDR = 0, totalTimeR = 0;
int startlfT = 0, totallfT = 0, intervalError = 0, lastError = 0, deltaError = 0, totalError = 0;
int startrfT = 0, totalrfT = 0, intervalErrorR = 0, lastErrorR = 0, deltaErrorR = 0, totalErrorR = 0;
int clearFlag = 0, clearFlagR = 0;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = lfT;
  statusPacket.params[1] = rfT;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendFinish() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_FINISH;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  cli();
  DDRD &= 0b11110011; //set Arduino pins 2 and 3 as input
  PORTD |= 0b00001100; //drive HIGH to enable pull-up resistors
  sei();
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  lfT++;
}

void rightISR() {
  rfT++;
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  cli();
  EICRA |= 0b00001010; //falling edge for INT1 and INT0
  EIMSK |= 0b00000011; //activate INT1 and INT0
  sei();
}

// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

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

  int count = 0;

  while (Serial.available())
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
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward()
{
  leftpid();
  rightpid();
  analogWrite(LF, pidpwr);
  analogWrite(RF, pidpwrR);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse()
{
  leftpid();
  rightpid();
  analogWrite(LR, pidpwr);
  analogWrite(RR, pidpwrR);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left()
{
  leftpid();
  rightpid();
  int pidpwrTurn = pidpwr / TURNING_DIV;
  int pidpwrTurnR = pidpwrR / TURNING_DIV;
  analogWrite(LR, pidpwrTurn);
  analogWrite(RF, pidpwrTurnR);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right()
{  
  leftpid();
  rightpid();
  int pidpwrTurn = pidpwr / TURNING_DIV;
  int pidpwrTurnR = pidpwrR / TURNING_DIV;
  analogWrite(LF, pidpwrTurn);
  analogWrite(RR, pidpwrTurnR);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

void leftpid()
{
  if(clearFlag == 1) {
    totalError = 0;
    lastError = 0;
    clearFlag = 0;
  }
  if(startFlag == 0) {
    startPID = millis();
    startlfT = lfT;
    startFlag = 1;
  }
  totalTime = millis() - startPID;

  if(totalTime > PID_INTERVAL) {
    startFlag = 0;
    totallfT = (lfT - startlfT);
    
    /* Positive error = too slow; negative error = too fast */
    intervalError = IDEAL_SPEED - totallfT;
    totalError += intervalError;
    deltaError = intervalError - lastError;
    
    pidpwr = BASE_POWER + kp*intervalError + ki*totalError + kd*deltaError;
    if(pidpwr > 255) pidpwr = 255;
    else if (pidpwr <0) pidpwr = 0;

    lastError = intervalError;
  }
  
}

void rightpid()
{
  if(clearFlagR == 1) {
    totalErrorR = 0;
    lastErrorR = 0;
    clearFlagR = 0;
  }
  if(startFlagR == 0) {
    startPIDR = millis();
    startrfT = rfT;
    startFlagR = 1;
  }
  totalTimeR = millis() - startPIDR;

  if(totalTimeR > PID_INTERVAL) {
    startFlagR = 0;
    totalrfT = (rfT - startrfT);
    
    /* Positive error = too slow; negative error = too fast */
    intervalErrorR = IDEAL_SPEED - totalrfT;
    totalErrorR += intervalErrorR;
    deltaErrorR = intervalErrorR - lastErrorR;
    
    pidpwrR = BASE_POWER + kp*intervalErrorR + ki*totalErrorR + kd*deltaErrorR;
    if(pidpwrR > 255) pidpwrR = 255;
    else if (pidpwrR <0) pidpwrR = 0;
    
    lastErrorR = intervalErrorR;

//    Serial.print("Interval error: ");
//    Serial.print(intervalErrorR);
//    Serial.print(" // totalError: ");
//    Serial.print(totalErrorR);
//    Serial.print(" // pidpwrR: ");
//    Serial.println(pidpwrR);
  }
  
}
/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  lfT = 0;
  rfT = 0;
}

// Clears one particular counter
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
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      dir = FORWARD;
      break;
    case COMMAND_REVERSE:
      sendOK();
      dir = REVERSE;
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      dir = LEFT;
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      dir = RIGHT;
      break;

    case COMMAND_STOP:
      sendOK();
      dir = STOP;
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

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
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      clearFlag = 1;
      clearFlagR = 1;
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

//  dir = FORWARD;
//  Serial.println(lfT);
//  Serial.println(rfT);
//  delay(500);

      
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }

  if(dir == FORWARD) {
    forward();
  }
  else if(dir == REVERSE) {
    reverse();
  }
  else if(dir == LEFT) {
    left();
  }
  else if(dir == RIGHT) {
    right();
  }
  else stop();


}
