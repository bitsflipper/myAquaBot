// AquaponicsBot Control System
//
// by: Myron Richard Dennison
//
// Features:
//    1. local/remote light switch 
//    2. auto grow light (using timestamp)
//    3. ambient temperature monitoring and logging (DHT11)
//    4. humidity sensor monitoring and logging (DHT11)
//    5. water temperature monitoring and logging (DS18B20)
//    6. water flow rate monitoring and logging
//    7. water pH level monitoring and logging
//    8. dissolved oxygen monitoring and logging
//
// Pin Assignments
//    Digital  
//    0
//    1                      
//    2       		     - DS18S20 Signal pin               
//    3                      -  
//    4, 5, 6                - RGB LED      
//    7, 8, 9, 10, 11, 12    - LCD
//    13                     - Pushbutton switch
//    Analog
//    0                      - Output LED
//    1                      - 
//    2                      - Flow Rate data pin
//    3                      - pH level sensor data pin
//    4                      - LDR
//    5                      - DHT data pin
//

//---------------------------------------->
// Libraries
//---------------------------------------->
#include <OneWire.h>                 // DS18B20
#include <idDHT11.h>                 // DHT sensor
#include <Process.h>                 // Linux processes
#include <FileIO.h>                  // SD card communication
#include <Bridge.h>                  // To pass information between two processors
                                     // The Bridge library simplifies communication between the ATmega32U4 and the AR9331.
#include <LiquidCrystal.h>

//---------------------------------------->
// define sensors
//---------------------------------------->
const int AMBIENT = 1;
const int WATER = 2;
const int HUMIDITY = 3;
const int FLOWRATE = 4; 
const int PH = 5;
const int DO = 6;

//---------------------------------------->
// I/O Pin Assignments
//---------------------------------------->
const int inputSwitchPin = 13;      // connect switch to digital pin 13 
const int outputLedPin = A0;        // connect LED to digital pin A0
const int ldrPin = A4;              // connect LDR  to analog pin A4
const int ledPinR = 4;              // connect tri-color Red pin to digital pin 4
const int ledPinG = 5;              // connect tri-color Green pin to digital pin 5
const int ledPinB = 6;              // connect tri-color Blue pin to digital pin 6
const int DS18S20_Pin = 2;          // DS18S20 Signal pin on digital 2
const int FLOWSENSORPIN = A2;       // water flow sensor data pin
const int phLevelSensorPin = A3;    // water pH level sensor data pin

//---------------------------------------->
// Initialize LCD controls and data lines
//---------------------------------------->
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//---------------------------------------->
// Initialize DS18S20
//---------------------------------------->
OneWire ds(DS18S20_Pin);     

//---------------------------------------->
// Initialize DHT11 
//---------------------------------------->
// External Interrupts: 3 (interrupt 0), 2 (interrupt 1), 0 (interrupt 2), 1 (interrupt 3) and 7 (interrupt 4). 
// These pins can be configured to trigger an interrupt on a low value, a rising or falling edge, or a change in value.
//
int idDHT11pin = 3;                 // HW interrupt pin for the DHT sensor
int idDHT11intNumber = 0;           // External HW interrupt number for pin 3
//
// ISR callback wrapper
void dht11_wrapper();               // must be declared before the instantiating the DHT object
//
// Instantiate the sensor object
idDHT11 DHT11(idDHT11pin,idDHT11intNumber,dht11_wrapper);

//---------------------------------------->
// Timer variables
//---------------------------------------->
long lcdReadingPreviousMillis = 0;  // last lcd update  
long wtcReadingPreviousMillis = 0;  // last wtc update
long updateGAEpreviousMillis = 0;   // last GAE update
long dhtSensorPreviousMillis = 0;   // last DHT sensor update
long flowRatePreviousMillis = 0;    // last flow rate update
long doPreviousMillis = 0;    // last do update
long waterPhPreviousMillis = 0;     // last pH level update
long lcdHeaderPreviousMillis = 0;   // last lcd header update
long lcdErrorPreviousMillis = 0;    // last lcd header update

//---------------------------------------->
// Global Variables
//---------------------------------------->
//
// pH sensor variables
unsigned long int avgValue;         //Store the average value of the sensor feedback
float b;
int buf[10],temp;
float ph;                           // water pH level

// light sensor variables
bool alert = false;                 // initialize alert to false
int digitalLdrVal;                  // mapped raw LDR values

// rh sensor variables
float rh;                           // relative humidity

// temperature sensors variables
float atc;                          // temperature as Celsius

//float atf;                        // temperature as Fahrenheit
float wtc;                          // water temperature as Celsius

// flow rate sensor variables
float fr;                           // water flow rate

// dissolved oxygen sensor variables
float d_oxygen;

// push button switch variables
volatile int currentSwitchState = LOW;
volatile int previousSwitchState = LOW;
volatile int outputLedPinState = LOW;

// other global variables
String connection = ""; 
String error;                       // holds the ERROR message
int hours, minutes, seconds;        // hold the current time
bool initialCheck = true;           // first check
bool adlawan = true;                // between sunrise and sunset
int sensor = AMBIENT;

//---------------------------------------->
// Initialize flow rate sensor
//---------------------------------------->
//
// count how many pulses!
volatile uint16_t pulses = 0;
// track the state of the pulse pin
volatile uint8_t lastflowpinstate;
// you can try to keep time of how long it is between pulses
volatile uint32_t lastflowratetimer = 0;
// and use that to calculate a flow rate
volatile float flowrate;
// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  }
  
  if (x == HIGH) {
    //low to high transition!
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}


void setup()
{
  Serial.begin(9600);
  
  Serial.println("Starting bridge...\n");
  digitalWrite(13, LOW);   
  Bridge.begin();                     // make contact with the linux processor
  digitalWrite(13, HIGH);             // Led on pin 13 turns on when the bridge is ready
  delay(2000);                 	      // wait 2 seconds  

  FileSystem.begin();                 // initializes SD card
  setupLog();                 	      // setup system log
  
  // Configures the specified pins to behave either as an input or an output.
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinB, OUTPUT); 
  pinMode(outputLedPin, OUTPUT); 
  pinMode(inputSwitchPin, INPUT); 
  pinMode(DS18S20_Pin, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(FLOWSENSORPIN, INPUT);
  
  digitalWrite(FLOWSENSORPIN, HIGH);
  lastflowpinstate = digitalRead(FLOWSENSORPIN);
  useInterrupt(true);
   
  digitalWrite(outputLedPin, LOW);    // LED initial state is OFF    

  // Setup LCD's initial display
  lcd.begin(16, 2);		      // set up lCD's number of columns and rows
  lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
  lcd.print("  BITSFLIPPER   ");  
  lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
  lcd.print("    STUDIO      ");
  
  delay(5000);
  
  // Setup LCD's initial display
  lcd.begin(16, 2);		      // set up lCD's number of columns and rows
  lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
  lcd.print("AQUAPONICS DAQ &");  
  lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
  lcd.print(" CONTROL SYSTEM ");
  
  delay(5000);
}

// This wrapper is in charge of calling 
// mus be defined like this for the lib work
void dht11_wrapper() {
  DHT11.isrCallback();
} 

void loop()
{   
  char atemp_str[10];
  char wtemp_str[10];
  char rh_str[10];
  char fr_str[10];
  char do_str[10];
  char ph_str[10];

  unsigned long currentMillis = millis();  // current time
  switchStateChange();
  // initial time check for the auto grow light
  if(initialCheck)
    growLight();  

  if(currentMillis - dhtSensorPreviousMillis > 2000){
    DHT11Sensor();
    dhtSensorPreviousMillis = currentMillis;
  }
  
  if(currentMillis - wtcReadingPreviousMillis > 4000){
    wtc = getWTemp(); 
    wtcReadingPreviousMillis = currentMillis;
  }
  
  if(currentMillis - flowRatePreviousMillis > 6000){
    //fr = flowrate;
    fr = 3.0;
    flowRatePreviousMillis = currentMillis;
  }
  
  if(currentMillis - doPreviousMillis > 8000){
    d_oxygen = 6.0;
    doPreviousMillis = currentMillis;
  }
  
  if(currentMillis - waterPhPreviousMillis > 10000){
    getWaterPHLevel();
    waterPhPreviousMillis = currentMillis;
  } 
  
  // display sensor readings on the LCD
  lcdDisplay();    
  
  digitalWrite(ledPinG, LOW);    
  if(currentMillis - updateGAEpreviousMillis > 60000){
    growLight();     
    //digitalWrite(ledPinG, HIGH);
    //connection = getGoogleAppEngineResponse("http://bitsflipperstudio.appspot.com/adacs/testConnection");
    connection = updateGAEServer("http://myaquabot.appspot.com/handlers/testConnection");
    if (connection == "Ok"){ 
      digitalWrite(ledPinR, LOW);
      digitalWrite(ledPinG, HIGH);
      String atc_s = dtostrf((int)atc, 3, 1, atemp_str);
      String wtc_s = dtostrf((int)wtc, 3, 1, wtemp_str);
      String rh_s = dtostrf((int)rh, 3, 1, rh_str);
      String do_s = dtostrf((int)d_oxygen, 3, 1, do_str);
      String fr_s = dtostrf((int)fr, 3, 1, fr_str);
      String ph_s = dtostrf((int)ph, 3, 1, ph_str); 
      //String postData = updateGAEServer("http://remoteaquaponicsdaqsystem.appspot.com/handlers/dht?ATemp=" + ambientTemp + "&WTemp=28"+ "&RH=" + rh + "&FR=120");
      //String postData = updateGAEServer("http://bitsflipperstudio.appspot.com/handlers/sensors?ATemp=" + ambientTemp + "&RH=" + rh);
      String postData = updateGAEServer("http://myaquabot.appspot.com/handlers/sensors?ATemp=" + atc_s + "&WTemp=" + wtc_s +  "&RH=" + rh_s + "&FR=3.0" + fr_s + "&PH=" + ph_s + "&DO=6.0");
      if (postData == "Ok"){
        digitalWrite(ledPinG, LOW);
        digitalWrite(ledPinR, LOW);
      }
      else{
        digitalWrite(ledPinR, HIGH);
        // Setup LCD's initial display
        lcd.begin(16, 2);		      // set up lCD's number of columns and rows
        lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
        lcd.print("ERROR: FAILED TO");  
        lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
        lcd.print("UPDATE GAESERVER");
      }            
    } 
    else{
      digitalWrite(ledPinR, HIGH);
      // Setup LCD's initial display
      lcd.begin(16, 2);		      // set up lCD's number of columns and rows
      lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
      lcd.print("ERROR: FAILED TO");  
      lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
      lcd.print("CONNECT TO GAE S");                 
    }
    updateGAEpreviousMillis = currentMillis;
  }     
}

// ----------------------------------------------------------------
void switchStateChange(){
  // reads and assign the value of inputSwitchPin
  currentSwitchState = digitalRead(inputSwitchPin);  

  //Check if the input just went from LOW to HIGH
  if((currentSwitchState == HIGH) && (previousSwitchState == LOW)){
    if(outputLedPinState == LOW)
      outputLedPinState = HIGH;        // toggle outputLedPinState
    else
      outputLedPinState = LOW;         // toggle outputLedPinState
    
    // sets the ouput LED and the grow LED
    digitalWrite(outputLedPin, outputLedPinState);  
    digitalWrite(ledPinB, !outputLedPinState);
  } 
  previousSwitchState = currentSwitchState;     
}

// ----------------------------------------------------------------
void growLight(){
  /*
  // check the 10-bit mapped input reading from the LDR
  digitalLdrVal = analogRead(ldrPin);
  if((digitalLdrVal <= 200) && (digitalRead(ledPinB) == LOW))
    digitalWrite(ledPinB, HIGH);
  else if((digitalLdrVal > 200) && (digitalRead(ledPinB) == HIGH))
    digitalWrite(ledPinB, LOW);*/
  // Replaced LDR with timestamp
  getHourMinSec();
  if(adlawan && (digitalRead(ledPinB) == HIGH))
    digitalWrite(ledPinB, LOW);
  else if(!adlawan && (digitalRead(ledPinB) == LOW)){
    digitalWrite(ledPinB, HIGH);
  }
  else{
    alert = true;
    error = "Timestamp ERROR!";
  }
    
  initialCheck = false;    
}

// ----------------------------------------------------------------
// Update system stats LED.
// Red: Alarm Detected
// Green: All sensor readings are within the limits 
// ----------------------------------------------------------------
void updateSystemStatusLED(){
  // check for Critical conditions and ERROR
  if(!alert)
    digitalWrite(ledPinR, LOW);
  else
    digitalWrite(ledPinR, HIGH);  
}

// ----------------------------------------------------------------
// Display
// ----------------------------------------------------------------
void lcdDisplay(){  
  unsigned long currentMillis = millis();  // current time
  if(currentMillis - lcdHeaderPreviousMillis > 60000) {
    // Setup LCD's initial display
    lcd.begin(16, 2);		      // set up lCD's number of columns and rows
    lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
    lcd.print("  BITSFLIPPER   ");  
    lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
    lcd.print("    STUDIO      ");
  
    delay(5000);
    
    // Setup LCD's initial display
    lcd.begin(16, 2);		              // set up lCD's number of columns and rows
    lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
    lcd.print("AQUAPONICS DAQ &");  
    lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
    lcd.print(" CONTROL SYSTEM ");
    
    delay(5000);
    
    /*lcd.begin(16, 2);		              // set up lCD's number of columns and rows
    lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
    lcd.print("AT:   WT:   R:  ");  
    lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
    lcd.print("FR:   PH:   M:  "); */
    
    // save the last time temp was updated 
    lcdHeaderPreviousMillis = currentMillis;  
  }
  
  if(currentMillis - lcdReadingPreviousMillis > 2000) {
    if(atc > 32 || wtc > 32){
      alert = true;
      updateSystemStatusLED();
      error = "OVERTEMP: " + (String)atc + "°C";     
      lcd.begin(16, 2);		              // set up lCD's number of columns and rows
      lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
      lcd.print("ALARM DETECTED!!");  
      lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
      lcd.print(error);
      log(error);
    }
    else if(ph < 5){
      alert = true;
      updateSystemStatusLED();
      error = "LOW pH LEVEL: " + (String)ph;     
      lcd.begin(16, 2);		              // set up lCD's number of columns and rows
      lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
      lcd.print("ALARM DETECTED!!");  
      lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
      lcd.print(error);
      log(error);
    }
    else{      
      alert = false;
      updateSystemStatusLED();  
      
      switch (sensor){
        case AMBIENT: 
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print(">>AMBIENT TEMP<<");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)atc);
          lcd.print((char)223);
          lcd.print("C");
          sensor = WATER;
          break;
       case WATER:           
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print(">>>WATER TEMP<<<");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)wtc);
          lcd.print((char)223);
          lcd.print("C");
          sensor = HUMIDITY;
          break;
       case HUMIDITY: 
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print(">>>>HUMIDITY<<<<");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)rh);
          lcd.print("%");
          sensor = FLOWRATE;
          break; 
        case FLOWRATE: 
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print(">>>FLOW RATE<<<<");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)fr);
          lcd.print("L/min");
          sensor = PH;
          break; 
        case PH: 
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print(">WATER pH LEVEL<");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)ph);
          sensor = DO;
          break;   
        case DO: 
          lcd.begin(16, 2);                   // set up lCD's number of columns and rows
          lcd.clear();			      // clears the LCD screen and position the cursor on the upper-left corner of the screen
          lcd.print("DISSOLVED OXYGEN");  
          lcd.setCursor(0, 1);		      // set the location at which the subsequent text written to the LCD will be displayed
          lcd.print("Reading: ");
          lcd.print((int)d_oxygen);
          lcd.print("mg/L");
          sensor = AMBIENT;
          break;   
      }
      /*
      // set the location at which the subsequent text written to the LCD will be displayed
      lcd.setCursor(3, 0);
      lcd.print((int)atc);
      //lcd.print((char)223);
      lcd.setCursor(9, 0);
      lcd.print((int)wtc);
      lcd.setCursor(14, 0);
      lcd.print((int)rh);
      lcd.setCursor(3, 1);
      lcd.print((int)fr);	
      lcd.setCursor(9, 1);
      lcd.print((int)ph);
      lcd.setCursor(14, 1);
      lcd.print((int)moisture);  */
    }
    
    // save the last time temp was updated 
    lcdReadingPreviousMillis = currentMillis;
  }   
}

// ----------------------------------------------------------------
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void DHT11Sensor(){  
  DHT11.acquire();
  while (DHT11.acquiring())
    ;
  int result = DHT11.getStatus();
  switch (result)
  {
    case IDDHTLIB_OK: 
      Serial.println("OK");       
      alert = false;
      break;
    case IDDHTLIB_ERROR_CHECKSUM: 
      Serial.println("Error\n\r\tChecksum error"); 
      error = "CHECKSUM ERROR";
      alert = true;
      break;
    case IDDHTLIB_ERROR_ISR_TIMEOUT: 
      Serial.println("Error\n\r\tISR Time out error"); 
      error = "ISR TIMEOUT";
      alert = true;
      break;
    case IDDHTLIB_ERROR_RESPONSE_TIMEOUT: 
      Serial.println("Error\n\r\tResponse time out error"); 
      error = "RESPONSE TIMEOUT";
      alert = true;
      break;
    case IDDHTLIB_ERROR_DATA_TIMEOUT: 
      Serial.println("Error\n\r\tData time out error"); 
      error = "DATA TIMEOUT";
      alert = true;
      break;
    case IDDHTLIB_ERROR_ACQUIRING: 
      Serial.println("Error\n\r\tAcquiring"); 
      error = "ACQUIRING";
      alert = true;
      break;
    case IDDHTLIB_ERROR_DELTA: 
      Serial.println("Error\n\r\tDelta time to small"); 
      error = "DELTA TIME TOO SMALL";
      alert = true;
      break;
    case IDDHTLIB_ERROR_NOTSTARTED: 
      Serial.println("Error\n\r\tNot started"); 
      error = "NOT STARTED";
      alert = true;
      break;
    default: 
      Serial.println("Unknown error"); 
      error = "UNKNOWN ERROR";
      alert = true;
      break;
  }
  Serial.print("Humidity (%): ");
  Serial.println(DHT11.getHumidity(), 2);

  Serial.print("Temperature (°C): ");
  Serial.println(DHT11.getCelsius(), 2);

  //Serial.print("Temperature (oF): ");
  //Serial.println(DHT11.getFahrenheit(), 2);

  //Serial.print("Temperature (K): ");
  //Serial.println(DHT11.getKelvin(), 2);

  //Serial.print("Dew Point (oC): ");
  //Serial.println(DHT11.getDewPoint());

  //Serial.print("Dew Point Slow (oC): ");
  //Serial.println(DHT11.getDewPointSlow());  
  
  atc = DHT11.getCelsius();        
  rh = DHT11.getHumidity();
}

// This method processes the HTTP response from the server
String updateGAEServer(String request){
  String response = "";
  
  // Launch "curl" command to send HTTP request to GAE server
  // curl is command line program for transferring data using different internet protocols
  Process sendRequest;		        // Create a process 
  sendRequest.begin("curl");	        // start with "curl" command
  sendRequest.addParameter(request);    // Add the server URL command as parameter to "curl"
  sendRequest.run();		        // Run the process and wait for its termination

  // Captures the HTTP response
  while (sendRequest.available() > 0) {
    char x = sendRequest.read();
    response += x;
    Serial.print(x); 
  }
  // Ensure the last bit of data is sent.
  Serial.flush();   
  return response;  
}

// Setup system log
void setupLog(){
  if(FileSystem.exists("/mnt/sda1/log.txt")){      
    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File logFile = FileSystem.open("/mnt/sda1/log.txt", FILE_APPEND);
    if(logFile){
      logFile.print("\n");
      logFile.print("Restarted: " + getTimeStamp());
      logFile.close();
    }
    //else
      //do something here
  }
  else{
    Process createFile;
    createFile.runShellCommand("touch /mnt/sda1/log.txt"); 
    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File logFile = FileSystem.open("/mnt/sda1/log.txt", FILE_APPEND);
    logFile.print("Created: " + getTimeStamp());
    logFile.close();
  }
}

// Get timestamp
String getTimeStamp() {
  String result;
  Process time;
  time.begin("date");
  time.addParameter("+%D-%T");  
  time.run(); 

  while(time.available()>0) {
    char c = time.read();
    if(c != '\n')
      result += c;
  }
  return result;
}

// Get the current time(hours; minutes; seconds)
void getHourMinSec() {
  String result;
  Process time;
  time.begin("date");
  time.addParameter("+%T");  
  time.run(); 
  
   //if there's a result from the time process, parse it
   while (time.available() > 0) {
    // get the result of the time process (should be hh:mm:ss):
    String timeString = time.readString();

    // find the colons:
    int firstColon = timeString.indexOf(":");
    int secondColon = timeString.lastIndexOf(":");

    // get the substrings for hour, minute second:
    String hourString = timeString.substring(0, firstColon);
    String minString = timeString.substring(firstColon + 1, secondColon);
    String secString = timeString.substring(secondColon + 1);

    // convert to ints,saving the previous second:
    hours = hourString.toInt();
    minutes = minString.toInt();
    seconds = secString.toInt();
  }  
  if(hours >= 6 && hours <= 18)
    adlawan = true;
  else
    adlawan = false;
}


// Log System Status
void log(String status){
  if(FileSystem.exists("/mnt/sda1/log.txt")){      
    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File logFile = FileSystem.open("/mnt/sda1/log.txt", FILE_APPEND);
    if(logFile){
      logFile.print("\n");
      logFile.print(getTimeStamp() + "  " + status);
      logFile.close();
    }
    //else
      //do something here
  }
  else{
    Process createFile;
    createFile.runShellCommand("touch /mnt/sda1/log.txt"); 
    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File logFile = FileSystem.open("/mnt/sda1/log.txt", FILE_APPEND);
    logFile.print("Created: " + getTimeStamp());
    logFile.close();
  }
}

float getWTemp(){
 //returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !ds.search(addr)) {
   //no more sensors on chain, reset search
   ds.reset_search();
   //return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   //return -1000;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   //return -1000;
 }

 ds.reset();
 ds.select(addr);
 ds.write(0x44,1);               // start conversion, with parasite power on at the end

 byte present = ds.reset();
 ds.select(addr);  
 ds.write(0xBE);                 // Read Scratchpad

 
 for (int i = 0; i < 9; i++) {   // we need 9 bytes
  data[i] = ds.read();
 }
 
 ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 if((TemperatureSum < 0) || (TemperatureSum > 50))
   TemperatureSum = wtc;
 
 return TemperatureSum; 
}

void getFlowRate(){
 // if a plastic sensor use the following calculation
  // Sensor Frequency (Hz) = 7.5 * Q (Liters/min)
  // Liters = Q * time elapsed (seconds) / 60 (seconds/minute)
  // Liters = (Frequency (Pulses/second) / 7.5) * time elapsed (seconds) / 60
  // Liters = Pulses / (7.5 * 60)
  //float liters = pulses;
  //liters /= 7.5;
  //liters /= 60.0;

  // if a brass sensor use the following calculation
  float liters = pulses;
  liters /= 8.1;
  liters -= 6;
  liters /= 60.0;

  Serial.print(liters); Serial.println(" Liters");    
}

// Get walter pH level
void getWaterPHLevel(){
  //Get 10 sample value from the sensor for smooth the value
  for(int i=0;i<10;i++){ 
    buf[i]=analogRead(phLevelSensorPin);
    delay(10);
  }
  //sort the analog from small to large
  for(int i=0;i<9;i++){
    for(int j=i+1;j<10;j++){
      if(buf[i]>buf[j]){
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  ph = phValue;
  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");  
}
