#include <Adafruit_GPS.h>
#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO         false
#define gpsSerial       Serial1


// Buzzer Pinout
#define BUZZER_CTR  2

// 124x68 OLED
#define OLED_MOSI   8
#define OLED_CLK    9
#define OLED_DC     10
#define OLED_CS     11
#define OLED_RESET  12

#define LATITUDE        32.232462
#define LONGITUDE       -110.951030

#define LATITUDE_ERROR  0
#define LONGITUDE_ERROR 0

// Geographical Constants
#define EARTH_RADIUS  3958.756 // miles


Adafruit_GPS GPS(&gpsSerial);
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// this keeps track of whether we're using the interrupt
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//char ssid[] = "William's iPhone";
//char pass[] = "b5yi5913vuvjz";
//int status = WL_IDLE_STATUS;

void setup()  
{
  // Setup Buzzer Pinout
  pinMode(BUZZER_CTR,OUTPUT);
  
  // Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  gpsSerial.begin(9600);
  
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

  // Start displaySerial for the Adafruit 124x68 OLED display
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC);
  display.display();
  delay(2000);
  display.clearDisplay();
  // Display has been initialized
  

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  gpsSerial.println(PMTK_Q_RELEASE);

  // Connect to WiFi
//  Serial.println("Attempting to start WiFi-Connection");
//  while(status != WL_CONNECTED) {
//    Serial.println(status);
//    Serial.println("...");
//    status = WiFi.begin(ssid, pass);
//    delay(10000);
//    Serial.println(status);
//  }
//
//  Serial.println(WiFi.localIP());
}

#ifdef __AVR__
  SIGNAL(TIMER0_COMPA_vect) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
  }

  void useInterrupt(boolean v) {
    if (v) {
      // Timer0 is already used for millis() - we'll just interrupt somewhere
      // in the middle and call the "Compare A" function above
      OCR0A = 0xAF;
      TIMSK0 |= _BV(OCIE0A);
      usingInterrupt = true;
    } else {
      // do not call the interrupt function COMPA anymore
      TIMSK0 &= ~_BV(OCIE0A);
      usingInterrupt = false;
    }
  }
#endif //#ifdef__AVR__

float deg2rad(float deg) {

  return (deg * (PI/180));
}

float toDecimalDegrees(float nmeaVal, char direction) {
  float degree = (int) nmeaVal/100;
  float decimal = (nmeaVal - (degree * 100))/60;
  float answer = degree + decimal;
  if(direction == 'S' || direction == 'W') return answer * -1;
  else return answer;
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);

    if (GPS.fix) {
      display.print("Latitude: ");
      display.print(toDecimalDegrees(GPS.latitude + LATITUDE_ERROR, GPS.lat), 4);
      display.println(GPS.lat);
      display.print("Longitude: ");
      display.print(toDecimalDegrees(GPS.longitude + LONGITUDE_ERROR, GPS.lon), 4);
      display.println(GPS.lon);
      display.print("Altitude: ");
      display.println(GPS.altitude, 4);
      display.print("Speed (knots): ");
      display.println(GPS.speed);
      display.print("Satellites: ");
      display.println(GPS.satellites);

      float lat1 =   deg2rad(LATITUDE);
      float lat2 =  deg2rad(toDecimalDegrees(GPS.latitude + LATITUDE_ERROR, GPS.lat));
      float latDiff = deg2rad(toDecimalDegrees(GPS.latitude + LATITUDE_ERROR, GPS.lat) - LATITUDE);
      float longDiff = deg2rad(toDecimalDegrees(GPS.longitude - LONGITUDE_ERROR, GPS.lon) - LONGITUDE);

      //float distance = acos(sin(latOrg)*sin(latNew))+cos(latOrg)*cos(latNew)*cos(longNew-longOrg) * EARTH_RADIUS;
      float a = sin(latDiff/2)*sin(latDiff/2)+cos(lat1)*cos(lat2)*sin(longDiff/2)*sin(longDiff/2);
      float c = 2*atan2(sqrt(a),sqrt((1-a)));
      float distance = EARTH_RADIUS * c;
      distance *= 1609.34;
      display.print("Distance to target:\n    ");
      display.print(distance / 1000);
      display.print("km");

      if(distance < 10) {
        tone(2,300);
      }
    } else {
      display.println("Searching for GPS signal...");
    }
    display.display();
  }
}
