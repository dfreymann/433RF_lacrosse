// 
// 433_RF_lacrosse_WIFI_Metro_fin
// dmf 11.15.15+
//
// ----------------------------------------------------------------
// 
// 9.17.19 Begin migration to C++ Platformio IDE and deal with all the problems... 
// 3.27.20 Issues: 
//          x argument of type "volatile byte *" is incompatible with parameter of type "byte *"
//          x argument of type "volatile byte (*)[9]" is incompatible with parameter of type "void *"
// 4.14.20 Compiles even with the above, ISR is responding. 
// 4.14.20 May not need the volatile for FinishedPacket. Tested this, getting the same result. 
// 4.14.20 This is testing with my new 433Mhz board based on Max1473. Tested with a Chineseum 
//         433mhz board - same result. 
// 4.14.20 Get the same behavior with compilation for the Metro using Arduino (rather
//         than Platformio), and with compilation to an Arduino Nano. 
// 4.14.20 OK, things are working fine... 
//
//
// =============================== Pin configuration ===================================
// 
// 433 Mhz Receiver Data Pin - DIGITAL PIN 8 
// 433 Mhz Receiver Power    - 5V
// 433 Mhz Status LED        - DIGITAL PIN 6    [5.18.16 was D2, previously]
// 433 Mhz Status LED Power  - 3V3
// These are pretty much hardwired in the code (see notes below)
//    4.15.20 - Note: the 433 Mhz status pin is invoked in the ISR, so it flickers 
//    too fast to be seen well. Don't count on it.
// 
// I2C SDA Pin - ANALOG PIN 4 (ARDUINO standard)
// I2C SCl Pin - ANALOG PIN 5 (ARDUINO standard)
// Interrupt Pin - DIGITAL PIN 7 [5.18.16 was D5, previously]
//                 (wired to N-Mosfet -> control Photon Interrupt)
//
// =============================== End Pin configuration ===============================
//
// Receive 433Mhz Temperature and Humidity from LaCrosse TX4U sender
//
//         To do: clean up debug statements.
//
// 6.4.16   a) made FinishedPacket volatile (volatile byte FinishedPacket [NUMBER_OF_PACKETS][PACKET_SIZE];)
//          b) note possible unhandled error if PacketDone, somehow, becomes > NUMBER_OF_PACKETS;
//
// 5.18.16 Changed LED indicator and I2C interrupt pins to D6 and D7, respectively. 
//         (Simplifies connector configuration.) All good.
//
// 5.15.16 During startup, the Interrupt pin is pulled low before it's pulled 
//         high, which triggers an I2C request from the Photon. I don't see 
//         how to get around this; however, can catch in code because the 
//         'dataStatus' flag is 0. So this can be handled on the Photon side. 
//
// 5.14.16 Code works and communicates to Photon as expected. However, during 
//         startup there's an unexplained empty transfer that occurs - apparently
//         the interrupt pin is pulled low without being explicitly called - so 
//         the photon requests data before any is available (so '0's are transferred). 
//         Following that, however, if there's a signal, it seems to be transferring 
//         subsequent temperature and humidity measurements correctly.
//         Note that I dropped the swap register stuff (which was causing problems 
//         due to the issue noted above), and things appear to work fine.
//
// 5.13.16 This version - free at last from P.O.S. CC3000 shield. CC3000 workaround
//         code from previous version is to be deleted, and most commments
//         relating to CC3000 issues will also be deleted.
//         Instead, configure to use I2C, serving as an I2C slave to a Photon
//         master. Deliver values when available and on request. That code is
//         straightforward. And works.
//
// **** NOTE *** updating the Arduino Boards Library (5.12.16) **breaks**
// compilation and upload to the ExtraCore board!! ****
//
// dmf 5.12.16 setup an ExtraCore board to be an I2C slave to the Photon:
//
// Follow the discussion at http://dsscircuits.com/articles/86-articles/78-arduino-i2c-slave-guide
// for the structure of this. conceptualize a 14 byte register:
// 0x00         Status Register (1 byte)
// 0x01 - 0x04  Temperature Register (Float - 4 bytes)
// 0x05 - 0x08  Humidity Register (Float - 4 bytes)
// 0x09 - 0x0B  Dewpoint Register (Float - 4 bytes)
// 0x0C         Identification Register (1 byte)
//
// 1.1.16 -> 5.12.16 CC3000 connectivity/hangup horror. Even with external reset board, continued
//                   getting connection losses and data gaps. Not worth it. Toss the CC3000 shield.
//
// 1.1.16 Begin removing superfluous DEBUG and testing routines.
//
// 12.4.15: Note that millis and delay use TIMER0. The ISR (input from the 433 
//   receiver) is triggered from TIMER1. So, as far as I can tell, no conflict; 
//   but in a more complicated scenario there could be problems galore. As of 
//   now, this code seems to be working 'perfectly'. 
//
// 12.3.15: Added elapsedMillis Library to allow control of how often 
//   data is uploaded to the Ubidots website (otherwise, pick up data 
//   as sent every minute from LaCrosse transmitter) [5.18.16 - not needed anymore
//   after conversion to I2C slave, but I'm leaving this in for now]
//
// 11.27.15 Delete PING 'tests'. Too buggy to work with. CC3000 is a failed chip -
//   (see: https://github.com/cmagagna/CC3000Patch#this-project-is-abandoned)
// 
// 11.22.15 LED configured to trigger on/off during ISR - LED flicker reflects ISR activity. 
//
// 11.21.15+
// The LaCrosse transmitter sends packets in groups of 2x2 (2 temperature, 
//   then 2 humidity). Processing and transmitting takes a lot of time, so better
//   to get both temperature and humidity packets before beginnning processing and
//   Ubidots upload steps. Configure the interrupt routine to manage collecting
//   successive packets (configurable from one to four, but testing suggests can
//   only capture two, a temperature packet and then a humidity packet).
// (Note that original plan to run a SHARP MEMORY DISPLAY off this board runs into 
// issues with program and memory space, so going with Plan B')
//
// Done:
// Manage two functional blocks: capture 433 receiver with ISR -> process & upload data. Therefore, 
//   after packet capture setup to save the ISR register states, restore default states, carry out 
//   the data processing & WIFI upload, and then restore registers for ISR, and repeat the loop.
//
//
// ================================ 433 Mhz Explication ================================
//
// The definitions for MIN_ONE, MAX_ONE, MIN_ZERO, MAX_ZERO, MIN_WAIT, MAX_WAIT
//   may be critical and look to be faulty in the distributed code.
//   With my sensor, I measure 0.554ms, 1.37ms and 0.938ms, respectively, for
//   the ONE, ZERO, and WAIT values, and have adjusted the timing ranges in the
//   #define statements below.
//
// See comments about CS11, CS10 in TCCR1B for timing code.
//
// Arduino Uno clock speed it 16 Mhz; 1Mhz -> .001ms period.
//
// dmf 11.1.15 Configuration of this code -
//
// A.  433 Mhz Data pin connected to Digital Pin 8 (PB0). (This pin will signal an
// interrupt routine ISR( TIMER1_CAPT_vect ) [see below])
//
// Port (PB0) Explication:
//   Pin 8 is pin 0 of PORT B. PORT B comprises digital pins 8 to 13.
//   (PORT C comprises analog pins A0 to A5, and PORT D comprises digital pins 0 to 7.)
//   Each PORT is controlled by three registers: DDR (Data Direction Register - read/
//   write), PORT (Data Register, read/write), and PIN (Input Pins Registor - read only)
//   Each bit of these registers correspons to a single pin.
//   So for DDRB - PIN0 to PIN6 correspond to digital Pin 8 to 13
//   For DDRC - PIN0 to PIN5 correspond to analog Pin 0 to 5
//   For DDRD - PIN0 to PIN7 correspond to digital Pin 0 to 7
//
// Set the INPUT/OUTPUT states of the pins using DDR -
//      e.g. in the code: DDRB = 0x2F; // B00101111 sets pins PB0-PB3 as OUTPUT,
//      pin PB4 as INPUT, pin PB5 as OUTPUT, and pins PB6 & PB7 as INPUT
// Note: Perhaps better usage is DDRB = DDRB | 00000001; to set pin PB0 (i.e. digital
//   pin 8) as OUTPUT without changing the state of the other pins. Or, simplified
//   as DDRB |= 00000001;
//
// Set the STATE of the outputs using PORT -
//      e.g. PORTD = B10101000; // sets digital pins 7,5,3 HIGH
// See below for examples of usage to set the LED (with additional code explication).
//
// B.  LED control pin connected to Digital Pin 2.
//   This pin has a pullup to the GPIO voltage. The LED is wired VCC->R->LED->Pin2,
//   so it is OFF when Pin 2 is HIGH, and ON when PIN 2 is LOW. Note that if power
//   to the LED is supplied externally (e.g. 5V supply to the 433 Mhz receiver), and
//   if the METRO is configured for 3V3 GPIO, there will be a voltage difference
//   (5V - 3V3) even when PIN 2 is HIGH, and the LED will glow weakly.
//
// 10.25.15 Begin migration to Metro + CC3000 sheild configuration.
//
// 9.1.15 Got this to work after changing the Timer1 prescaler from 64 to 256! Why? My 
//   guess for now is that since the ICR1 was rolling over every 65536, or ~256 ms,
//   and since the data pulse (44 bits) was taking 72-102ms, there was a
//   significant probability of data occurring over the rollover boundary. But it could
//   be some other issue I don't yet appreciate.
//
// Copyright 2016 Douglas Freymann
// info@jaldilabs.org
// License: GPLv3
//
//Based on original code written by Kelsey Jordahl: 
/* lacrosse.pde
 * Version: 1.1
 * Author: Kelsey Jordahl
 * Copyright: Kelsey Jordahl 2010
   (portions copyright Marc Alexander, Jonathan Oxer 2009;
    Interactive Matter 2009 [licensed under GPL with permission])
 * License: GPLv3
 * Time-stamp: <Mon Jun 27 12:02:27 EDT 2011> 

Receive La Crosse TX4 weather sensor data with Arduino. 

Assumes the 433 MHz data pin is connected to Digital Pin 8 (PB0).

Based on idea, and some code, from Practical Arduino
 http://www.practicalarduino.com/projects/weather-station-receiver
 http://github.com/practicalarduino/WeatherStationReceiver

Also useful was the detailed data protocol description at
 http://www.f6fbb.org/domo/sensors/tx3_th.php

433.92 MHz RF receiver:
 http://www.sparkfun.com/commerce/product_info.php?products_id=8950

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.  A copy of the GPL
    version 3 license can be found in the file COPYING or at
    <http://www.gnu.org/licenses/>.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

// ================================ End 433 Mhz Explication =============================

// Comment out for a normal build
// Uncomment for a debug build
#define DEBUG

// ========= includes ============
#include <Arduino.h>

#include <math.h>
#include <elapsedMillis.h>
#include <Wire.h>

// ========= end includes ========

// ========= correction defines ========

#define SENSORCORRECTIONF 0.0        // No correction. Problem: nonlinear, depending 
                                     // on inside/outside temp and depending on
                                     // sensor placement next to building. A -5degF
                                     // 'correction' was measured 1.1.16 (31.5 reading,
                                     // 26.4 actual measured at surface), but no longer
                                     // used.

// ========= end correction defines ====

// ========= i2c defines ========

#define SLAVE_ADDRESS  0x29
#define REG_MAP_SIZE 14
#define INTERRUPT_PIN 7   // PIN D7 [*** Was pin 5 (D5) ***]

// ========= end i2c defines ====

// ========= 433 Mhz defines =============

#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))
// TCCR1B: The Timer/Counter Control register is used to set the timer mode,
// prescaler and other options.
// ICES1: Input Capture Edge Select;
// When cleared, the contents of TCNT1 are transferred to ICR (Input Capture
// Register) on the falling edge of the ICP pin.
// If set, the contents of TCNT1 are transferred on the rising edge of the ICP pin.
//
// dmf 11.1.15 Usage (_BV) explication:
// _BV stands for Bit Value, where you pass it a bit (here ICES1) and it gives
// you a byte value with that bit set. Here, ICES1 is Bit 6 of the TCCR1B register.
// The code ((TCCR1B & _BV(ICES1)) != 0) is testing whether the ICES1 bit 6 of the
// TCCR1B registor is set (!=0), or not (==0).
// The code (TCCR1B |= _BV(ICES1) is using the OR operator to set the ICES1 bit 6
// of the TCCR1B register to 1 (and for the expression &= ~_BV(ICES1), to 0).

// Configuration with LED controlled from PORTD Pin 7
// #define RED_TESTLED_OFF()           ((PORTD &= ~(1<<PORTD7)))
// #define RED_TESTLED_ON()            ((PORTD |=  (1<<PORTD7)))
// Configuration with LED controlled from PORTD Pin 2
// 11.3.15 changed from configuration above (see notes below)
// 5.18.16 changed from D2 to D6 (also change DDRD statement - see later)
// #define RED_TESTLED_ON()           ((PORTD &= ~(1<<PORTD2)))
// #define RED_TESTLED_OFF()          ((PORTD |=  (1<<PORTD2)))
#define RED_TESTLED_ON()           ((PORTD &= ~(1<<PORTD6)))
#define RED_TESTLED_OFF()          ((PORTD |=  (1<<PORTD6)))
//
// dmf 11.1.15 Usage explication:
// The expression 1<<PORTD2 is a bit shift operation that sets the bit value
// at PORTD2 (i.e. PD2, digital pin 2) to '1'. The expression ((PORTD &= ~(1<<PORTD2)))
// sets a value of '0' at bit position PORTD2, and sets the corresponding bit
// of PORTD to 0. Similarly, the expression ((PORTD |= (1<<PORTD2))) sets a value
// of '1' at bit position PORTD2, and sets the corresponding bit of PORTD to 1.
// PORTD controls the output state of PORTD, so this corresponds to setting the
// output pin to LOW or HIGH, respectively.
//
// The orginal code had GREEN_TESTLED_OFF() and ON() configured opposite (e.g.
// GREEN_TESTLED_ON() was defined as ((PORTD &= ~(1<<PORTD6))) ) so that it was
// sinking current to switch the LED on. I only have the RED LED setup now,
// configured as the GREEN LED was (see above), but did not change this part of
// the code. [NOTE: Do I have LED wired backwards?]
// 
// 4.15.20 Note: the 433 Mhz status pin is invoked in the ISR, so it flickers too 
// fast to be seen well. Dont count on it.

/* serial port communication (via USB) */
#define BAUD_RATE 9600
//#define BAUD_RATE 115200

// dmf 8.30.15 the packet is 11 nibbles (44 bits); first byte is constant, value 0x0A,
// and used to identify a valid packet-in-progress, hence PACKET_START. data and check 
// are in the remaining 9 nibbles, hence PACKET_SIZE
#define PACKET_SIZE 9           /* number of nibbles in packet (after inital byte) */
#define PACKET_START 0x0A       /* byte to match for start of packet */
// 0x0A is 0000 1010, which is the bit set that occurs at the beginning of a packet. 

// dmf 11.21.15 add NUMBER_OF_PACKETS (1-4)
#define NUMBER_OF_PACKETS 2       // capture more than one packet before processing and uploading  
                                  // note that it looks like we usually don't capture the temperature or humidity 
                                  // pairs, but we can capture temperature + humidity, so this should be left 
                                  // at '2'
                               
// See my notes:
// ONE - short, 0.554 ms average   // 11.22.15 0.52ms +/- 0.01 -> 31/35
// ZERO - long, 1.37 ms average    // 11.22.15 1.31ms +/- 0.01 -> 81/86
// GAP - medium, 0.938 ms average  // 11.22.15 1.02ms +/- 0.01 -> 63/68
// The values of numbers defined below depend on the timer prescaler value (they express
//   ms in terms of clock ticks). I have this working now with a prescaler of 256 (rather 
//   than 64 as distributed), (i.e. a 0.16 ms tick), so these numbers express a range of 
//   pulse widths in ms as VALUE*(4/256)=TIME(ms). 
// But CRITICAL THAT THE RANGES BE SET WIDE ENOUGH! If too tight, will not capture anything!
//   Actual values reported by ISR for ONE: 29-32  (Saleae baseline peak width 0.52*64 -> 33)
//   Actual values reported by ISR for ZERO: 79-82 (Saleae baseline peak width 1.31*64 -> 84)
//Note that the spread is larger than measured (Saleae) Avg +/- 3 Sig (above). Set the 
//   values below based on the actual CapturedPeriod reported in the ISR + what works.
// 0.5 ms high is a one; 0.476 actual average [worked: 27/39]
#define MIN_ONE 27		// minimum length of '1' (was 31)   Tested: 26
#define MAX_ONE 39		// maximum length of '1' (was 38)   Tested: 34
// 1.3 ms high is a zero; 1.26 actual average [worked: 77/91]
#define MIN_ZERO 77		// minimum length of '0'  (was 78)  Tested: 77
#define MAX_ZERO 91		// maximum length of '0'  (was 92)  Tested: 85
// 1 ms between bits;                         [worked: 50/70]
#define MIN_WAIT 50		// minimum interval since end of last bit  (was 52) Tested: 61
#define MAX_WAIT 70		// maximum interval since end of last bit  (was 65) Tested: 73
//
// Also, note that a good read will have a gap prior to the packet of 27.903 ms +/- 0.157, and a 
// good read will have a gap after the packet as well... 

// ========= end 433 Mhz defines =========

// ========= i2c data structure & variables ===============

// for WorkspsaceVariableStruct, WorkspaceUnion and wksp, see comment
// at http://dsscircuits.com/articles/arduino-i2c-slave-guide#comment-464
// Note interesting issue of alignment; expect AVR 1 byte aligment, makes
// things easier. ** Important - this does not necessarily hold for other 
// boards (e.g. Photon)! 
struct WorkspaceVariablesStruct {
    byte status;
    float temperature;
    float humidity;
    float dewpoint;
    byte config;
    byte fill[2];
    // up to this point we need 16 bytes
    };

// define register that will hold values and be sent by i2c
union WorkspaceUnion {
    struct WorkspaceVariablesStruct var;
    byte registerMap[REG_MAP_SIZE];
    };

// access workspace.variables.[status, temperature, etc..]
union WorkspaceUnion wksp;

// identification value
byte identiValue = 0x99;

// data status flag (possible future use)
byte dataStatus = 1;

// dataStatus modifiers -
byte haveTemperature = 0x02;
byte haveHumidity = 0x04;

// ========= end i2c data structure & variables ===========

// ========= 433 Mhz variables ===========

// these need to be "volatile"? 
//     Variables only used outside an ISR should not be volatile.
//     Variables only used inside an ISR should not be volatile.
//     Variables used both inside and outside an ISR should be volatile.
unsigned int CapturedTime;
// dmf initialize PreviousCapturedTime
unsigned int PreviousCapturedTime = 0;
unsigned int CapturedPeriod;
unsigned int BitCount;

byte j;
float tempC;			  /* temperature in deg C */
float tempF;			  /* temperature in deg F */
float dpC;			  /* dewpoint (deg C) */
float dpF;                        /* dewpoint (deg F) */
float h;			  /* relative humidity */
byte DataPacket[PACKET_SIZE];	  /* actively loading packet */
// dmf 6.4.16 - FinishedPacket should be volatile
// dmf 4.14.20 - Definition as volatile is causing issues; test change
// volatile byte FinishedPacket [NUMBER_OF_PACKETS][PACKET_SIZE]; /* fully read packet */
byte FinishedPacket [NUMBER_OF_PACKETS][PACKET_SIZE]; /* fully read packet */

byte PacketBitCounter;
// dmf initialized ReadingPacket
boolean ReadingPacket = 0;
// dmf added Capturing flag
boolean Capturing = 0;
// dmf added volatile
// 11.21.15 changed volatile boolean to int to allow indexing packet array
volatile int PacketDone = 0;
// flags for captured values
boolean newTemp = false;
boolean newHumidity = false;
boolean newDewPoint = false;

byte mask;		    /* temporary mask byte */
byte CompByte;		    /* byte containing the last 8 bits read */

byte savedDDRB;        /* saved register settings */
byte savedPORTB;
byte savedTCCR1A;
byte savedTCCR1B;
byte savedTIMSK1;
byte savedDDRD; 

elapsedMillis timeElapsed;   /* add timer to control upload period */ 
elapsedMillis connectionTime;   /* add timer for connection activity */ 

// ========= end 433 Mhz variables =======

// ========= i2C inline functions ========

// data available interrupt functions -
inline void noDataAvailable(){ digitalWrite(INTERRUPT_PIN, HIGH); }
inline void dataIsAvailable(){ digitalWrite(INTERRUPT_PIN, LOW); }

// ========= end i2c inline functions ====

// ======= FUNCTIONS ======
void sendDataValues(); 
void configureReceiverTimerInterrupt(); 
void parsePacket(byte *Packet);
float dewpoint(float T, float h);


// ======= end FUNCTIONS =====

// ================== SETUP and LOOP ===========================

void setup() {

    RED_TESTLED_OFF();    // initialize the LED port as OFF

    #ifdef DEBUG
      Serial.begin( BAUD_RATE );
      Serial.println( "La Crosse weather station capture begin" );
      // add an 'alive' indicator (for TEST CODE here)
      pinMode(13, OUTPUT); 
    #endif

    // ==== configure for i2c interrupt ====

    pinMode(INTERRUPT_PIN, OUTPUT);
    // pull HIGH when new data is available.
    // See https://community.particle.io/t/internally-pulled-up-interrupt-issue-and-work-around/14717/3
    // "Are you using the below pinMode=>attachInterrupt way?
    // - pin mode INPUT_PULLUP => should use FALLING interrupt mode
    // - pin mode INPUT_PULLDOWN => should use RISING interrupt mode"
    // Here, 5V INTERRUPT_PIN coupled via N-Mosfet to 3V3 pin on Photon.
    // LOW shuts off the Mosfet, so Photon pin is HIGH. Interrupt on falling
    // when INTERRUPT_PIN goes HIGH, Mosfet conducts and grounds the Photon pin.
    noDataAvailable();

    // ==== configure for i2c ====

    // define the I2C slave address
    Wire.begin(SLAVE_ADDRESS);

    // define the ISR that will respond to request
    Wire.onRequest(sendDataValues);

    // constant value
    wksp.registerMap[REG_MAP_SIZE - 1] = identiValue;

    // ==== configure for 433 receiver ISR ====
  
    /* Initialize the 433 Mhz Interrupt register settings */
    cli();        // disable global interrupts
    configureReceiverTimerInterrupt();
    sei();        // enable interrupts
  
}

void loop() {

    #ifdef DEBUG
      // Serial.println("in loop");
      // Serial.print(" dataStatus ");
      // Serial.println(dataStatus);
      digitalWrite(13, HIGH);
     // RED_TESTLED_ON();
      delay (500);
      digitalWrite(13, LOW);
     // RED_TESTLED_OFF();
      delay (1000);
    #endif
    
    // state of the loop is controlled by port register settings, which
    // are swapped to either a) respond to 433 Data interrupt (set in
    // configureReceiverTimerInterrupt()), or b) respond to I2C interrupt
    // (default settings). Don't want these two states to overlap, to avoid,
    // e.g., receiver data interrupts preventing communication via I2C

    // initialize with data Interrupt toggled HIGH
    noDataAvailable();

    // dataStatus flags state: 0 - re-starting 433 ISR collection (occurs following
    // response to I2C request and data transfer); 1 - actively collecting data;
    // 3 - have a temperature; 5 - have a humidity; 7 - both temperature & humidity
    if (dataStatus == 0) {
        cli();                  // disable global interrupts
        #ifdef DEBUG
          Serial.println("in dataStatus reset"); 
        #endif
        // set the flag
        dataStatus = 1;
        // and reset the newData flags
        newTemp = false;
        newHumidity = false;
        newDewPoint = false;
        sei();
    }

    // just loop and let the interrupt service routine capture packets OR respond to I2C.

    // Packets are flagged with a constant two-byte header at the beginning.
    //   Information is encoded in the length of the pulses (short -> '1', long -> '0'),
    //   in the seperation between pulses, and as the total number of pulses encoding
    //   the packet. Also, packets are transmitted twice, seperated by a known time
    //   between packets; finally, the temperature packet pair, and the humidity packet
    //   pair follow each other with a known interval between them. We can try to
    //   incorporate all this information.
      
    // PROBLEM: temperature and humidity are sent sequentially,
    //   so, if spend time processing a Temperature packet,
    //   won't be around to capture the Humidity packet!
    //   11.22.15 - Solved by adding packet[] array
    
    // 11.22.15 In a noisy environment (?) (e.g. with antenna), the lead-in to the first
    //   packet is often a mess (monitored using Saleae),while the second packet of each
    //   pair is clean. Reconfigured the ISR to treat Rising_Edge > MAX_WAIT as possible
    //   start of following packet, rather than resetting the Capture flag (which rejects
    //   that edge as completely invalid).
 
    // got a packet; receiver ISR sets PacketDone; if a packet is done, parse the packet.
    if (PacketDone == NUMBER_OF_PACKETS) {      // have a bit string that's ended

        cli();                                  // disable global interrupts
        
        #ifdef DEBUG
          Serial.println("in PacketDone "); 
        #endif
        
        // process the packet
        for (int i=0; i < NUMBER_OF_PACKETS; i++) {
            // just dump all the packets for now - will segregate if we've captured both
            //   temperature and humidity packets. Future: can try to match packet pairs
            //   to validate the measurements.
            parsePacket(FinishedPacket[i]);
        }

        #ifdef DEBUG
          Serial.print("have a newTemp ");
          Serial.println(newTemp);
          Serial.print("have a newHumidity ");
          Serial.println(newHumidity);
        #endif

        // reset count
        PacketDone = 0;

        // update dataStatus flag
        if (newTemp) dataStatus |= haveTemperature;
        if (newHumidity) dataStatus |= haveHumidity;

        // if both Temperature and Humidity are updated, configure for transfer
        if (newTemp && newHumidity) {

            #ifdef DEBUG
              Serial.print("The temperature is: ");
              Serial.println(tempF); 
              Serial.print("The humidity is: ");
              Serial.println(h); 
            #endif
        
            // map the data to transfer registers, exploiting the
            // data structure and the byte register union
            wksp.var.status = dataStatus;
            wksp.var.temperature = tempF;
            wksp.var.humidity = h;
            wksp.var.dewpoint = dpF;

            // re-enable interrupts
            sei();
            // and set the interrupt pin LOW (to tell the Photon there's something to read)
            dataIsAvailable();

        } else {
            // to continue receiving data re-enable interrupts
            sei();
        }
    }
}

// ================== end SETUP and LOOP =======================

// ============= Data processing methods ================

//
//  parsePacket
//
// Parse a raw data string
void parsePacket(byte *Packet) {

  byte chksum;

  #ifdef DEBUG
    Serial.println("in parsePacket"); 
  #endif

  chksum = 0x0A;
  for (j=0; j<(PACKET_SIZE-1); j++) {
    chksum += Packet[j];
  }

  if ((chksum & 0x0F) == Packet[PACKET_SIZE-1]) { /* checksum pass */

    /* check for bad digits and make sure that most significant digits repeat */
    if ((Packet[3]==Packet[6]) && (Packet[4]==Packet[7]) && (Packet[3]<10) && (Packet[4]<10) && (Packet[5]<10)) {
      
      /* process a temperature packet */
      if (Packet[0]==0) {		
	
        tempC=(Packet[3]*10-50 + Packet[4] + ( (float) Packet[5])/10);
	    tempF=tempC*9/5 + 32;
          
        tempF = tempF + SENSORCORRECTIONF;
          
        /* determine dewpoint - requires a prior humidity measurement */ 
	    dpC=dewpoint(tempC,h);
        dpF=dpC*9/5 + 32;
          
        /* sanity check the Temperature and DewPoint */
        if (tempF > -40 && tempF < 120) newTemp = true; 
        if (dpF > 32 && dpF < 120) newDewPoint = true; 
        
      } else {
        
        /* process a humidity packet */ 
	    if (Packet[0]==0x0E) {
	  
          h=(Packet[3]*10 + Packet[4]);
            
          /* sanity check the Humidity */
          if (h > 0 && h < 100) newHumidity = true;
            
        } else  {
	
          /* process a custom packet */
          if (Packet[0]==0x0B) {
	        tempC=(Packet[3]*10-50 + Packet[4] + ( (float) Packet[5])/10);
	        tempF=tempC*9/5 + 32;
              
          }
        }
      }
    } else {
      
      /* this is an unknown packet, ignore it */
      #ifdef DEBUG
      Serial.println("Fail secondary data check.");
      #endif
    }
  } else {
      
    /* checksum fail; this is a corrupted or bad packet, ignore it */ 
    #ifdef DEBUG
    Serial.print("chksum = 0x");
    Serial.print(chksum,HEX);
    Serial.print(" data chksum = 0x");
    Serial.println(Packet[PACKET_SIZE-1],HEX);
    #endif
  }
  
}

// Convert temperature/humidity data to dewpoint
float dewpoint(float T, float h) {
    float td;
    // Simplified dewpoint formula from Lawrence (2005), doi:10.1175/BAMS-86-2-225
    td = T - (100-h)*pow(((T+273.15)/300),2)/5 - 0.00135*pow(h-84,2) + 0.35;
    return td;
}

// ============= end Data processing methods ============

// ============= 433 Receiver Interrupt Config ==========

//
//  configureReceiverTimerInterrupt
//
// configure the ports and timer registers
void configureReceiverTimerInterrupt() {
    
  // Configuration specific: 
  // INDICATOR LED on digital pin 2 
  //   DDRD (port D Register)
  //   PORTD2 (pin 2 of port D)
  // RECEIVER INPUT on digital pin 8 
  //   DDRB (port B Register)
  //   PORTB (port B Status)
  //   PORTB0 (pin 0 of port B)
  
  // This sets the INPUT/OUTPUT states of the PORT B pins (8-13). I do not know
  // why this specifically is necessary here.
  DDRB = 0x2F;             // B00101111
  // This sets PB0 to INPUT.  DDB0 refers to bit 0 (PB0) of the DDRB register. The
  // expression (1<<DDB0) sets that value to '1', ~() inverts it to '0', and
  // DDRB &= ~(1<<DDB0) sets the DDRB register value for PB0 to '0' (i.e. INPUT).
  DDRB  &= ~(1<<DDB0);     // PBO(ICP1) input
  // This sets the state of PB0 to LOW (i.e. pullup is disabled). PORTB0 refers to
  // bit 0 (PB0) of the PORT register, and configuration is as above (set to '0')
  PORTB &= ~(1<<PORTB0);   // ensure pullup resistor is also disabled

  // Set up timer1 for RF signal detection. There are two registers, TCCR1A and TCCR1B,
  // that control Timer1. Set all values of register TCCR1A to '0' defaults -
  TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
  // TCCR1B: The Timer/Counter Control register is used to set the timer mode,
  // prescaler and other options. First, set values to '0' defaults -
  TCCR1B = B00000000;
  // Then set the bits that control 'Input Capture Noise Canceler' ICNC1, and
  // the timer prescaler (set using CS10, CS11, and CS12).
  // dmf 9.1.15 Important change: Prescaler of 256 (CS12 set) rather than 64 (CS11,
  // CS10 set)
  TCCR1B = ( _BV(ICNC1) | _BV(CS12) ); // 256 prescaler (0.016 ms tick)
  // CS12, CS11, CS10: Clock Select bits; These three bits control the prescaler
  // of timer/counter 1 and the connection to an external clock on Pin T1. Here,
  // setting Bits CS11 and CS10 to '1', so that we have 0, 1, 1 - divide clock by 64.
  // The Uno system clock is 16 Mhz. 1Mhz is a period of 0.001 ms. Here, the timer
  // clock is being set at 16/64 = 1/4 Mhz, with a period of 0.004 ms. The constants
  // MIN_ONE, MAX_ONE, MIN_ZERO, MAX_ZERO correspond to ticks of the Tclock,
  // so 135 * 0.004 ms = 0.54ms as the minimum interval for the '1' bit, and so on.
  SET_INPUT_CAPTURE_RISING_EDGE();
  // This sets the ICES1 bit (INPUT CAPTURE EDGE SELECT, bit 6) of TCCR1B.
    
  // Timer1 Input Capture Interrupt Enable (ICIE1), do not need an Overflow Interrupt
  // Enable (TOIE1)
  TIMSK1 = ( _BV(ICIE1) );
  // dmf 11.1.15 The register TIMSK1 is the Timer/Counter1 Interrupt Mask Register.
  // It controls which interrupts the timer can trigger.
   
  // setup PORTD to control the LED(s)
  // Configuration using PORTD Pin 7
  //  DDRD |= B11000000;
  // Configuration using PORTD PIN 2 [5.18.16 - actually, this sets up PINS D2 and D6]
  DDRD |= B01000100;
}

// ============= end 433 Receiver Interrupt Config ======

// ============= Interrupt service routine ==============

//
// Interrupt service routine
//
ISR( TIMER1_CAPT_vect )
// This is an Interrupt routine (ISR - Interrupt Service Routine)
// It should be short, fast and should have no Serial.print statements!
{ 
  
  // Immediately grab the current capture time in case it triggers again and
  // overwrites ICR1 with an unexpected new value.
  CapturedTime = ICR1;
  // ICR1:  The Input Capture register can be used to measure the time between
  // pulses on the external ICP pin (Input Capture Pin).
  // How this pin is connected to ICR is set with the ICNC and ICES bits in TCCR1A.
  // When the edge selected is detected on
  // the ICP, the contents of TCNT1 are transferred to the ICR and an interrupt is
  // triggered.
  // TCNT1:  Most important is the Timer/Counter Register (TCNT1) itself. This is
  // what all timer modes base on. It counts
  // System Clock ticks, prescaled system clock or from the external pin.
    
  // Measure the time between edges (will be pulse width IF the CapturedPeriodWasHigh)
  CapturedPeriod = (CapturedTime - PreviousCapturedTime);
  // Cases: 1.  !Capturing AND Rising_Edge  - possible initial bit, start Capturing
  //           [!Capturing AND Falling_Edge  - shouldn't happen, Falling_Edge only when Capturing]
  //
  //        2.  Capturing AND Period < MIN_ONE  - too short, reject and reset !Capturing
  //        3.  Capturing AND Falling_Edge AND Period > MAX_ZERO - too long, reject and reset !Capturing
  //        4.  Capturing AND Rising_Edge AND Period > MAX_WAIT - too long, reject, but 
  //             restart Capturing (since may be next packet)
  //
  //        5.  Capturing AND Falling_Edge AND Period < MAX_ONE - it's a ONE
  //        6.  Capturing AND Falling_Edge AND Period > MIN_ZERO - it's a ZERO
  //
  //        7.  Capturing AND Falling_Edge AND Anything Else - unknown, reject and reset !Capturing 
  
  RED_TESTLED_ON();
                
  if (!Capturing) {                     // Searching for an edge, and may have found it...
            
      // rising edge encountered, start capturing
      Capturing = 1;
      
      // flag to pick up the next edge
      SET_INPUT_CAPTURE_FALLING_EDGE();
      
      // initialize packet variables
      CompByte = 0xFF;
      ReadingPacket = 0;
      PacketBitCounter = 0;
      
      // save the timing data
      PreviousCapturedTime     = CapturedTime;
      
      // and we're done...
        
  } else {                              // Otherwise, capturing has already started, so evaluate the
                                        // timings, and capture the bit if it's valid
                                                                       
      // first, handle two exclusion conditions
      if (CapturedPeriod < MIN_ONE) {
                            
          // reset Capturing
          Capturing = 0;
          // reset for rising edge
          SET_INPUT_CAPTURE_RISING_EDGE();
          // and reject...
          PreviousCapturedTime = CapturedTime; 
          
      } else if (INPUT_CAPTURE_IS_FALLING_EDGE() && CapturedPeriod > MAX_ZERO) {
                  
          // reset Capturing
          Capturing = 0;
          // reset for rising edge
          SET_INPUT_CAPTURE_RISING_EDGE();
          // and reject...
          PreviousCapturedTime = CapturedTime; 

      } else if (INPUT_CAPTURE_IS_RISING_EDGE()){
      // next handle a rising edge (2 cases: capture the falling edge or restart capturing)
          
          if (CapturedPeriod > MAX_WAIT) {         // reject edges that took too long, but since may 
                                                   // be start of second packet, re-initialize packet 
                                                   // capture
                        
            // flag to pickup the falling edge 
            SET_INPUT_CAPTURE_FALLING_EDGE();
            
            // re-initialize packet variables 
            CompByte = 0xFF;
            ReadingPacket = 0;
            PacketBitCounter = 0;

            // save timing data and continue...
            PreviousCapturedTime = CapturedTime; 
                          
          } else if (CapturedPeriod < MIN_WAIT) {    // Rising edge occurs too soon
          
            // reset Capturing
            Capturing = 0;
            // reset for rising edge
            SET_INPUT_CAPTURE_RISING_EDGE();
            // and reject...
            PreviousCapturedTime = CapturedTime;
          
          } else {                              // otherwise, good edge, setup to capture the rest of the bit
                                
              // flag to pickup the falling edge
              SET_INPUT_CAPTURE_FALLING_EDGE();
              // save the timing data
              PreviousCapturedTime     = CapturedTime;
              // and we're done...
            }
          
      } else if (CapturedPeriod > MAX_ONE && CapturedPeriod < MIN_ZERO) {      // test falling edge exclusion (between 0, 1)
      // handle a final exclusion condition...
      // the rest of the events processed now are INPUT_CAPTURE_IS_FALLING_EDGE()
      // here, falling edge occurs >MAX_ONE and <MIN_ZERO after rising edge, so not-to-spec
            
          // reset Capturing
          Capturing = 0;
          // reset for rising edge
          SET_INPUT_CAPTURE_RISING_EDGE();
          // and reject...
          PreviousCapturedTime = CapturedTime; 
          
      } else {                                    // otherwise, good bit, add to packet
      // finally, process the candidate bit values (2 cases: '1', '0'), and decide about packet processing
                    
          if (CapturedPeriod < MAX_ONE ){               // it's a ONE
          // bit has been tested as >MIN_ONE (in exclusions) and <MAX_ONE (here)
          
              // if we've identififed a packet, add the ONE
              if (ReadingPacket) {	/* record the bit as a one */
                  mask = (1 << (3 - (PacketBitCounter & 0x03)));
                  DataPacket[(PacketBitCounter >> 2)] |= mask;
                  PacketBitCounter++;
              } else {		  /* still looking for valid packet data */
                  // CompByte is acting as a stack accumulating all valid bits encountered.
                  if (CompByte != 0xFF) {	/* don't bother recording if no zeros recently */
                      CompByte = ((CompByte << 1) | 0x01); /* push one on the end */
                  }
              }
          } else if (CapturedPeriod > MIN_ZERO) {       // it's a ZERO
          // bit has been tested as <MAX_ZERO (in exclusions) and >MIN_ZERO (here)
          
              // if we've identififed a packet, add the ZERO
              if (ReadingPacket) {	/* record the bit as a zero */
                  mask = (1 << (3 - (PacketBitCounter & 0x03)));
                  DataPacket[(PacketBitCounter >> 2)] &= ~mask;
                  PacketBitCounter++;
              } else {		      /* still looking for valid packet data */
                  // CompByte is acting as a stack accumulating all valid bits encountered.
                  CompByte = (CompByte << 1); /* push zero on the end */
              }
          }
          
          // With a ONE or ZERO bit in hand, check whether we're DONE with reading the current
          // packet, or whether we need to START reading a new packet.
          if (ReadingPacket) {
            if (PacketBitCounter == (4*PACKET_SIZE)) { /* done reading packet */
                            
                // dmf 11.21.15 handle multiple packets, index using PacketDone
                // PacketDone should index from 0 to (NUMBER_OF_PACKETS-1), 
                // PacketDone == NUMBER_OF_PACKETS flags processing
                
                // Handle possible error - 
                if (PacketDone < NUMBER_OF_PACKETS) {
                   memcpy(&FinishedPacket[PacketDone],&DataPacket,PACKET_SIZE);
                   PacketDone += 1;
                }
                
                ReadingPacket = 0;
                PacketBitCounter = 0;
                Capturing = 0;
            }
          } else {
            /* Check whether we have the start of a data packet */
            if (CompByte == PACKET_START) {
             
                CompByte=0xFF;		/* reset comparison byte */
                /* set a flag and start recording data */
                ReadingPacket = 1;
            }
          }
        
          // finally, flag to pickup the next rising edge
          SET_INPUT_CAPTURE_RISING_EDGE();
          // save the timing data
          PreviousCapturedTime     = CapturedTime;
          // and we're done...
      }
  }
  RED_TESTLED_OFF();
}

// ============= end Interrupt service routine ==========

// ============= setup I2C handling =====================

// send temperature and humidity values to the I2C master
void sendDataValues() {
    
   Wire.write(wksp.registerMap, REG_MAP_SIZE);
    // flag dataStatus here (rather than in loop())
    // to allow possible future loop '|=' increment.
    dataStatus = 0;
}

// ============ end I2C handling =======================
