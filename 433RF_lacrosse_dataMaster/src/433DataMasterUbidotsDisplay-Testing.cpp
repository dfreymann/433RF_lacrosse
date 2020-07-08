/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/freymann/Dropbox/Electronics/_CODE/PlatformIO/433RF_lacrosse/433RF_lacrosse_dataMaster/src/433DataMasterUbidotsDisplay-Testing.ino"
// dmf-photon-6-DisplayUbidots
// dmf 12.4.15+
//
// 6.3.16 - currently configured on dmf-photon-6
//
// dmf 5.15.16 code revised for I2C 'display' master / '433 receiver' slave configuration
//
// Request LaCrosse 433 temp/humidity data from ExtraCore slave using I2c, display locally
//   using a Sharp Memory Display (Adafruit https://www.adafruit.com/products/1393), and
//   upload periodically to Ubidots.
//
// 12.9.15: test http.response status for good request. Status is set to '-1'
//   in HttpClient, and returns the response value. Ubidots returns '200' for
//   a good request. See:
//   http://ubidots.com/docs/api/v1_6/data/get_variables_id_values.html
//
// 12.8.15 Revision to clean up the demo code and try to fix some display bugs
//
// Initiated from example code available:
//   https://community.particle.io/t/ubidots-and-the-particle-photon/17933/13
// Working in concert with code _433RF_MultiPacket_lacrosse_WIFI_Metro
//   running on a Metro/CC3000 sheild/433 receiver to upload data to Ubidots

// comment out for normal build
#define doDEBUG

// libraries (must be local and must be included in project folder)
#include "do_Photon_mfGFX.h"              // modified from Adafruit_mfGFX
                                          // note: will mod fonts (do_fonts)
#include "do_Photon_SharpMem.h"           // modified from Adafruit_SharpMem
#include "elapsedMillis.h"                // https://github.com/pkourany/elapsedMillis
#include "HttpClient.h"                   // https://github.com/nmattisson/HttpClient
// note that Wire is included in application.h
#include "application.h"                  // Particle default
void setup(void);
void loop(void);
int uploadValue (String variableId, float uploadValue);
float truncateFloat(float theValue);
void setDataFlag();
void testdrawline();
#line 35 "/Users/freymann/Dropbox/Electronics/_CODE/PlatformIO/433RF_lacrosse/433RF_lacrosse_dataMaster/src/433DataMasterUbidotsDisplay-Testing.ino"

// ============= Ubidots Defines =======================

// Ubidots requires specific variable IDs for upload
// These are specific for the device (Metro with CC3000 shield) and
// the variables (temperature, humidity)
#define VARIABLE_IDTEMP "56492b8b76254252b588e625"
#define VARIABLE_IDHUMI "56492ba676254253dc4aa3e6"
#define VARIABLE_IDDEWP "5650e94e7625426feb167612"

// Ubidots account:
// #define TOKEN "34WGjY1Q703dFW4eNQaJoDrBMKXHKF9EzWd7XiTbRvU0jcS59QxHnrEesMrX"
// 1.8.20 updated ubidots token - 
#define TOKEN "A1E-aS8AZa3WTRuZFh22Lkro5FVT529bVL"


// Ubidots returns '200' for a good request; returns '201' for good send.
# define GOODRESPONSE 201

// ============= End Ubidots Defines ===================

// ============= I2C Defines ===========================

#define SLAVE_ADDRESS 0x29      // this address is defined by the Slave: Wire.begin(SLAVE_ADDRESS)
#define REG_MAP_SIZE 14         // number of bytes in the data structure (2xbyte, 3xfloat(4 byte))
#define INTERRUPT_PIN A0        // Slave will pull this pin low when new data is available for request

// ============= End I2C Defines =======================

// ============= Font Defines ==========================

// Fonts available are defined in three files: do_Photon_fonts.h,
//      do_Photon_fonts.cpp, and do_Photon_mfGFX.cpp -
//
//  GLCDFONT        selected using  GLCDFONT
//  TIMESNEWROMAN8  selected using  TIMESNR_8
//  CENTURYGOTHIC8  selected using  CENTURY_8
//  ARIAL8          selected using  ARIAL_8
//  COMICSANSMS8    selected using
//  TESTFONT        selected using  TEST
//  CALIBRI18       selected using  CALIBRI_18
//  CALIBRI38       selected using  CALIBRI_X_38    // character "x" only
//  CALIBRI48       selected using  CALIBRI_48
//  CALIBRId48      selected using  CALIBRI_dig_48  // digit 0-9 only
//  VERDANA88       selected using  VERDANA_dig_88  // digits 0-9 only + ".","/"
//  VERDANA120      selected using  VERDANA_dig_120 // digits 0-9 only

#define BLACK 0
#define WHITE 1

// ============== End Font Defines ====================

// ============== Http Client Setup ===================

HttpClient http;

// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Content-Type", "application/json" },
    { "X-Auth-Token" , TOKEN },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};
http_request_t request;
http_response_t response;

// ============= End Http Client Setup ===============

// ============ Display Configuration ===============

// Particle Photon SPI pins
// SCK  - default, defined as A3 for Photon)
// MOSI - default, defined as A5 for Photon)
// SS   - user selectable
#define SS A2

Adafruit_SharpMem display(SCK, MOSI, SS);

// ============ End Display Configuration ============

// ============ i2C Data Structure ===================

// see http://forum.arduino.cc/index.php?topic=45445.30
float Temperature, Humidity, Dewpoint;
byte Status, Config;

// for WorkspsaceVariableStruct, WorkspaceUnion and wksp, see comment
// at http://dsscircuits.com/articles/arduino-i2c-slave-guide#comment-464
// Note interesting issue of alignment; expect AVR 1 byte aligment, makes
// things easier.
struct WorkspaceVariablesStruct {
    byte status;
    float temperature;
    float humidity;
    float dewpoint;
    byte config;
    byte fill[2];
// up to this point we need 16 bytes
// note that for the Photon, in contrast to the Arduino code for
// the ExtraCore, we need the __attribute__((packed)); here. This
// allows one byte mapping of the variables to match the AVR code.
} __attribute__((packed));

// define register that will hold values and be sent by i2c
union WorkspaceUnion {
    struct WorkspaceVariablesStruct var;
    byte registerMap[REG_MAP_SIZE];
};

// access workspace.variables.[status, temperature, etc..]
union WorkspaceUnion wksp;

// ============  end i2c Data Structure ==============

// ============= Flow Control Variables ==============

// flag set by slave indicating data available
volatile bool dataIsAvailable = FALSE;

// keep track of whether the screen needs to be updated
float savedTemperature = 0.0;
float savedHumidity = 0.0;
bool resetScreen = FALSE;

// define the elapsed timer (for sends to Ubidots)
elapsedMillis timeElapsed;
// and set the interval (in ms)
unsigned int sendInterval=300000;    // 5 minutes

// also indicate if the numbers are stale (>1hr)
elapsedMillis timeSinceLast;
unsigned int staleInterval = 3600000;   // 1 hour

// ============ End Flow Control Variables ==========

// ============ Setup () and Loop () =================

void setup(void) {

    // I2C configuration for communication with Slave
    Wire.begin();

    // note: see https://community.particle.io/t/internally-pulled-up-interrupt-issue-and-work-around/14717/3
    // "Are you using the below pinMode=>attachInterrupt way?
    // - pin mode INPUT_PULLUP => should use FALLING interrupt mode
    // - pin mode INPUT_PULLDOWN => should use RISING interrupt mode"
    pinMode(INTERRUPT_PIN,INPUT_PULLUP);
    // setDataFlag() is the (very simple) ISR for the data interrupt
    attachInterrupt(INTERRUPT_PIN, setDataFlag, FALLING);

    // HTTP configuration for transfer to Ubidots
    // setup request.hostname and request.port here,
    // however, request.path will be assigned in function
    request.hostname = "things.ubidots.com";
    request.port = 80;

    // SPI configuration for Display
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);    // BitOrder is manipulated when communicating with display
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // Initialize the hardware ports since cannot be done in class instantation above
    display.init();
    // start & clear the display
    display.begin();
    display.clearDisplay();
    // draw many lines graphic at startup
    testdrawline();

    #ifdef doDEBUG
        Serial.begin(9600);
        Serial.println("\nI2C input from slave ");
        // I'm alive flasher
        pinMode(D7, OUTPUT);
    #endif

    // force an Http send on first read
    timeElapsed = sendInterval;

    // stale timer initialized
    timeSinceLast = 0;
}

void loop(void) {

    // truncated floats
    float iTemperature;
    float iHumidity;
    float iDewpoint;
    // display strings
    String outputTemperature;
    String outputHumidity;

    //  loop logic -
    //  1) wait for I2C interrupt to signal data available
    //  2) new measurement available
    //      a) request new measurement data
    //      b) check if the values changed from previous
    //      c) if yes, display new measurement data
    //      d) if sufficient time elapsed, upload data to Ubidots
    //  5) if current data is stale, update display with "<1hr"
    //  6) refresh the display

    #ifdef doDEBUG
        digitalWrite(D7, HIGH);
        delay(500);
        digitalWrite(D7, LOW);
        delay(500);
    #endif

    // respond to new data (flagged by ISR when I2C interrupt pulled low)
    if (dataIsAvailable){

        // Apparently, this always returns TRUE; (future) If needed,
        // a failed request (bad data) can be detected as all 'FFFF's (HIGH)
        if (Wire.requestFrom(SLAVE_ADDRESS,REG_MAP_SIZE)) {

            // grab the full registerMap
            for (int i = 0; i < REG_MAP_SIZE; i++) {
                wksp.registerMap[i] = Wire.read();
            }

            #ifdef doDEBUG
                Serial.print("status 1 ");
                Serial.println(wksp.var.status, HEX);
            #endif

            // future - check Status to know what values
            // have been updated
            // for now - if Status is '0' this is a 'hardware glitch'
            // return (see Slave code); use the conditional to ignore it.
            if (Status = wksp.var.status) {

                // Data structure has been defined using the
                // WorkspaceVariablesStruct and WorkspaceUnion declarations
                Temperature = wksp.var.temperature;
                Humidity = wksp.var.humidity;
                Dewpoint = wksp.var.dewpoint;
                Config = wksp.var.config;

                // future - check for bad values (e.g. FFFFFFFF)

                #ifdef doDEBUG
                    Serial.print("status 2 ");
                    Serial.println(Status, HEX);
                    Serial.print("temp ");
                    Serial.println(Temperature);
                    Serial.print("humid ");
                    Serial.println(Humidity);
                    Serial.print("dew ");
                    Serial.println(Dewpoint);
                    Serial.print("config ");
                    Serial.println(Config, HEX);
                #endif

                // if sufficient time elapsed, upload new data to Ubidots
                if (timeElapsed > sendInterval) {

                    #ifdef doDEBUG
                        Serial.println("in http send ");
                    #endif

                    // Send the measurements to Ubidots...
                    uploadValue (VARIABLE_IDTEMP, Temperature);
                    uploadValue (VARIABLE_IDHUMI, Humidity);
                    uploadValue (VARIABLE_IDDEWP, Dewpoint);

                    // (future) can test value of int uploadValue() == GOODRESPONSE
                    // Ubidots returns int response.status = 201 for good upload

                    // reset request timer
                    timeElapsed = 0;
                }

                // truncate to 1 significant digit so that new value test is stable
                iTemperature = truncateFloat(Temperature);
                iHumidity = truncateFloat(Humidity);
                iDewpoint = truncateFloat(Dewpoint);

                // check if the values are new
                if (iTemperature != savedTemperature) {
                    savedTemperature = iTemperature;
                    resetScreen = TRUE;
                }
                if (iHumidity != savedHumidity) {
                    savedHumidity = iHumidity;
                    resetScreen = TRUE;
                }

                // write new values and labels to the display buffer
                if (resetScreen) {

                    // output string formatted with 1 significant digit
                    // see https://community.particle.io/t/issues-with-converting-float-to-string-with-sprintf
                    outputTemperature = String(Temperature,1);
                    // output string formatted to 0 significant digits
                    outputHumidity = String(Humidity,0);

                    // initialize
                    display.clearDisplay();

                    // constant text labels
                    display.setTextSize(1);
                    display.setFont(ARIAL_8);
                    display.setTextColor(BLACK);

                    display.setCursor(5,5);
                    display.print("Temperature");

                    display.setCursor(7,65);
                    display.print("Humidity");

                    // temperature display
                    display.setTextSize(2);         // will change this once have font
                    display.setFont(CALIBRI_18);    // will change this once have font

                    display.setCursor(7,18);
                    display.print(outputTemperature);

                    // humidity display
                    display.setTextSize(1);

                    display.setCursor(50,70);
                    display.print(outputHumidity);
                    display.print("%");

                    // degrees symbol (small font)
                    display.setCursor(82,14);
                    display.print("o");

                    // reset
                    resetScreen = FALSE;

                    // reset stale timer
                    timeSinceLast = 0;
                }

                // and reset the I2C interrupt flag
                dataIsAvailable = 0;

            } else {

                // ignore and reset the flag
                dataIsAvailable = 0;
            }

        } else {
            // failed
        }
    // end of response to new data
    }

    // check whether the numbers are old
    // future: seems like this could be better handled with
    // a timer interrupt (that's reset when new data available)
    if (timeSinceLast > staleInterval) {

        // if so, adjust the display
        display.setTextSize(1);
        display.setFont(ARIAL_8);

        display.setCursor(7,80);
        display.print(">1 hr");

        timeSinceLast = 0;
    }

    // display has to be refreshed every 500 ms
    display.refresh();
}

// =================== Functions ==========================

int uploadValue (String variableId, float uploadValue) {                  // [*** WILL REVISE FOR PUT ***]

    request.path = "/api/v1.6/variables/" + variableId + "/values";
    request.body = "{\"value\":" + String(uploadValue,1) + "}";
    http.post(request, response, headers);

    #ifdef doDEBUG
        Serial.println(request.path);
        Serial.println(request.body);
        Serial.print("Application>\tResponse status: ");
        Serial.println(response.status);
        Serial.print("Application>\tHTTP Response Body: ");
        Serial.println(response.body);
    #endif

    return(response.status); // < Ubidots returns "response status" of 201 for good upload

}

// limit to one significant digit
float truncateFloat(float theValue) {

    float realValue;
    int   intValue;

    intValue = 10*theValue + 0.5;
    realValue = (float(intValue)/10);

    return(realValue);
}

// ISR for the I2C Data Available interrupt pin
void setDataFlag(){
    dataIsAvailable = TRUE;
}

// ============= GRAPHICS test called at startup ===========

void testdrawline() {
  for (uint8_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  for (uint8_t i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (uint8_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int8_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int8_t i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int8_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (uint8_t i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, BLACK);
    display.refresh();
  }
  for (uint8_t i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  delay(250);
}
