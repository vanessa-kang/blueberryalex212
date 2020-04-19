#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include <stdarg.h>
#include <avr/sleep.h>

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
#define IDEAL_SPEED         15  //Roughly 15 counts per 100ms = 150 counts per sec = 3/4 revs per second
#define IDEAL_SPEED_TURN    6   //Ideal speed for turning.
#define BASE_POWER          50  //Starting power level
#define BASE_POWER_TURN     125  //Starting base power for turns
#define PID_INTERVAL        100 //Period between each PID value update

//P,I and D coefficients
#define KP                  10
#define KI                  7
#define KD                  10

//P,I and D coefficients for turning
#define KPT                 15
#define KIT                 2
#define KDT                 4

/*
      Alex's State Variables
*/
//power reduction and sleep mode control
#define PRR_TWI_MASK            0b10000000 
#define PRR_SPI_MASK            0b00000100 
#define ADCSRA_ADC_MASK         0b10000000 
#define PRR_ADC_MASK            0b00000001 
#define PRR_TIMER2_MASK         0b01000000 
#define PRR_TIMER0_MASK         0b00100000 
#define PRR_TIMER1_MASK         0b00001000 
#define SMCR_SLEEP_ENABLE_MASK  0b00000001 
#define SMCR_IDLE_MODE_MASK     0b11110001


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
void WDT_off(void) { 

  //Global interrupt should be turned OFF here if not already done so
  cli();

  //Clear WDRF in MCUSR 
  MCUSR &= ~(1<<WDRF); 
 
  //Write logical one to WDCE and WDE
  //Keep old prescaler setting to prevent unintentional time-out
  WDTCSR |= (1<<WDCE) | (1<<WDE); 
 
  //Turn off WDT 
  WDTCSR = 0x00; 
   
  /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require turning off global interrupt */
  sei();
}

//call this function within void setup() later
void setupPowerSaving() {
  // Turn off the Watchdog Timer   
  WDT_off();
 
  // Modify PRR to shut down TWI
  PRR |= PRR_TWI_MASK;
 
  // Modify PRR to shut down SPI
  PRR |= PRR_SPI_MASK;
 
  // Modify ADCSRA to disable ADC,
  ADCSRA &= 0b01111111;

  // then modify PRR to shut down ADC
  PRR |= PRR_ADC_MASK;

  // Set the SMCR to choose the IDLE sleep mode   
  // Do not set the Sleep Enable (SE) bit yet 
  SMCR &= SMCR_IDLE_MODE_MASK;
}

//call this in void loop() when appropriate
//e.g. when there is no USART data AND motors are stopped
void putArduinoToIdle() {   

  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= PRR_TIMER0_MASK;
  PRR |= PRR_TIMER1_MASK;
  PRR |= PRR_TIMER2_MASK;

  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;

  // The following function puts ATmega328P’s MCU into sleep;   
  // it wakes up from sleep when USART serial data arrives 
  sleep_cpu();     

  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= 0b11111110;
  
  // Modify PRR to power up TIMER 0, 1, and 2 
  PRR &= 0b10010111;
}



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
  DDRD |= 0b01100000; //set pin 5 (PD5) and pin 6 (PD6) as output
  DDRB |= 0b00001100; //set pin 10 (PB2) and pin 11 (PB3) as output
  
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is blank.
void startMotors()
{
  //implement baremetal PWM here?
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward()
{
  leftpid(IDEAL_SPEED, BASE_POWER, KP, KI, KD);
  rightpid(IDEAL_SPEED, BASE_POWER, KP, KI, KD);
  analogWrite(LF, pidpwr);
  analogWrite(RF, pidpwrR);
//  PORTD &= 0b10111111; //replaces analogWrite(LR, 0);
//  PORTB &= 0b11111011; //replaces analogWrite(RR, 0);
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
  leftpid(IDEAL_SPEED, BASE_POWER, KP, KI, KD);
  rightpid(IDEAL_SPEED, BASE_POWER, KP, KI, KD);
  analogWrite(LR, pidpwr);
  analogWrite(RR, pidpwrR);
//  PORTD &= 0b11011111; //replaces analogWrite(LF, 0);
//  PORTB &= 0b11110111; //replaces analogWrite(RF, 0);
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
  //leftpid(IDEAL_SPEED_TURN, BASE_POWER_TURN, KPT, KIT, KDT);
  pidpwr = (pidpwrR > 240) ? 160 : 0;
  rightpid(IDEAL_SPEED_TURN, BASE_POWER_TURN, KPT, KIT, KDT);
  analogWrite(LR, pidpwr);
  analogWrite(RF, pidpwrR);
//  PORTD &= 0b11011111; //replaces analogWrite(LF, 0);
//  PORTB &= 0b11111011; //replaces analogWrite(RR, 0);
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
  leftpid(IDEAL_SPEED_TURN, BASE_POWER_TURN, KPT, KIT, KDT);
  pidpwrR = (pidpwr > 240) ? 160 : 0;
  //rightpid(IDEAL_SPEED_TURN, BASE_POWER_TURN, KPT, KIT, KDT);
  analogWrite(LF, pidpwr);
  analogWrite(RR, pidpwrR);
//  PORTD &= 0b10111111; //replaces analogWrite(LR, 0);
//  PORTB &= 0b11110111; //replaces analogWrite(RF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  //this baremetal is not working idk why
//  PORTD &= 0b10011111; //replaces analogWrite(LR, 0) and analogWrite(LF, 0)
//  PORTB &= 0b11110011; //replaces analogWrite(RF, 0) and analogWrite(RR, 0)
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  analogWrite(LF, 0);
  analogWrite(LR, 0);
}

void leftpid(int idealSpeed, int basePower, int kp, int ki, int kd)
{
  if(clearFlag == 1) {
    pidpwr = 150;
    totalError = 0;
    lastError = 0;
    clearFlag = 0;
    startFlag = 1;
  }
  if(startFlag == 1) {
    startPID = millis();
    startlfT = lfT;
    startFlag = 0;
  }
  totalTime = millis() - startPID;

  if(totalTime > PID_INTERVAL) {
    startFlag = 1;
    totallfT = (lfT - startlfT);
    
    /* Positive error = too slow; negative error = too fast */
    intervalError = idealSpeed - totallfT;
    totalError += intervalError;
    deltaError = intervalError - lastError;
    
    pidpwr = basePower + kp*intervalError + ki*totalError + kd*deltaError;
    if(pidpwr > 255) pidpwr = 255;
    else if (pidpwr <0) pidpwr = 0;
    lastError = intervalError;
  }
  
}

void rightpid(int idealSpeed, int basePower, int kp, int ki, int kd)
{
  if(clearFlagR == 1) {
    pidpwrR = 150;
    totalErrorR = 0;
    lastErrorR = 0;
    clearFlagR = 0;
    startFlagR = 1;
  }
  if(startFlagR == 1) {
    startPIDR = millis();
    startrfT = rfT;
    startFlagR = 0;
  }
  totalTimeR = millis() - startPIDR;

  if(totalTimeR > PID_INTERVAL) {
    startFlagR = 1;
    totalrfT = (rfT - startrfT);
    
    /* Positive error = too slow; negative error = too fast */
    intervalErrorR = idealSpeed- totalrfT;
    totalErrorR += intervalErrorR;
    deltaErrorR = intervalErrorR - lastErrorR;
    
    pidpwrR = basePower + kp*intervalErrorR + ki*totalErrorR + kd*deltaErrorR;
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
  clearFlag = 1;
  clearFlagR = 1;
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
  setupPowerSaving();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
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

//  dir = FORWARD;
//  Serial.println(lfT);
//  Serial.println(rfT);
//  delay(500);

  //dir = STOP;
      
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
  else{
    stop();
    putArduinoToIdle();
  }

//  Serial.print(pidclearpwr);
//  Serial.print(" // ");
//  Serial.println(pidpwrR);


}
