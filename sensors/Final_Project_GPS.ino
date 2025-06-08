#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS TX (transmit) pin to D10 (Arduino RX)
// Connect the GPS RX (receive) pin to D9 (Arduino TX)
SoftwareSerial mySerial(10, 9);
Adafruit_GPS GPS(&mySerial);

float startLat, startLon, startTrackAngle;
int degrees;
float minutes;

void setup() {
  Serial.begin(9600);

  GPS.begin(9600);  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);   // set update rate (1, 2, 5, or 10Hz)

  delay(50);

  Serial.println("Searching for GPS satellite fix...");
  unsigned long startTime = millis();
  while ((millis() - startTime) < 600000)
  {
    GPS.read();
//      Serial.println(".");
    if (GPS.newNMEAreceived()) {
      Serial.println("New NMEA message received.");

      if (GPS.parse(GPS.lastNMEA())) {
        Serial.println("Message parsed.");
        if (GPS.fix) {
          Serial.println("Satellite fix.");

          startLat = GPS.latitude;
          degrees = (int(startLat) / 100);
          minutes = startLat - degrees * 100;
          startLat = minutes / 60.0; // + degrees;
          if (GPS.lat == 'S') startLat = -startLat;

          startLon = GPS.longitude;
          degrees = (int(startLon) / 100);
          startLon = (startLon - degrees * 100) / 60.0; // + degrees;
          if (GPS.lon == 'W') startLon = -startLon;

          startTrackAngle = GPS.angle; 
          Serial.print("Starting Location found.");
          Serial.println(" ");
          Serial.print(startLat, 6);
          Serial.print(", ");
          Serial.println(startLon, 6);
          break;
        }
        else {
          Serial.println("No fix yet. Searching...");
        }
      }
    }
  }

  Serial.println("");
  Serial.println("");
  Serial.println("X, Y, Theta");
}


void loop() {
  float currentLat, currentLon, x, y, r, theta;
  GPS.read();

  if (GPS.newNMEAreceived()) {
    //    Serial.println("New NMEA message received.");
    if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    //    Serial.println("Message parsed.");
    if (GPS.fix) {
      //      Serial.println("Satellite fix.");
      currentLat = GPS.latitude;
      degrees = (int(currentLat) / 100);
      currentLat = (currentLat - degrees * 100) / 60.0;// + degrees;
      if (GPS.lat == 'S') currentLat = -currentLat;

      currentLon = GPS.longitude;
      degrees = (int(currentLon) / 100);
      currentLon = (currentLon - degrees * 100) / 60.0;// + degrees;
      if (GPS.lon == 'W') currentLon = -currentLon;

      y = distance(startLat, currentLat, startLon, startLon);
      if (currentLat < startLat) y = -y;
      x = distance(startLat, startLat, startLon, currentLon);
      if (currentLon < startLon) x = -x;
      r = sqrt(x * x + y * y);
      theta = atan2(x, y) * 180 / M_PI;

      float currentSpeedMetersPerSec = GPS.speed * 0.514444;  // Convert to meters per second
      float currentTrackAngle = GPS.angle;

      Serial.println("********************************************************************************");
      Serial.print("start lat: ");
      Serial.println(startLat);
      Serial.print("start lon: ");
      Serial.println(startLon);
      Serial.print("start track angle: ");
      Serial.println(startTrackAngle);
      Serial.print("current lat: ");
      Serial.println(currentLat);
      Serial.print("current lon: ");
      Serial.println(currentLon);
      Serial.print("current track angle: ");
      Serial.println(currentTrackAngle);
      Serial.print("current speed: ");
      Serial.println(currentSpeedMetersPerSec);

      float z0Global, z1Global, z2Global = gpsToXY(currentLat, currentLon, currentTrackAngle, startLat, startLon, startTrackAngle);
      //      Serial.print(x);
      //      Serial.print(", ");
      //      Serial.println(y);


      Serial.println(" ");
      Serial.println(" ");

      Serial.print("X: ");
      Serial.println(z0Global);
      Serial.print("Y: ");
      Serial.println(z1Global);
      Serial.print("Theta: ");
      Serial.println(z2Global);
      //      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      delay(500);
    }
  }
}


float distance(float lat1, float lat2, float lon1, float lon2) {
  //  Input arguments: starting and ending latitude and longitude, degrees
  float a, c, d;
  unsigned long const R = 6371e3; // Earth radius, m
  lat1 = M_PI / 180 * lat1; //  Convert lat & lon angles to radians
  lat2 = M_PI / 180 * lat2;
  lon1 = M_PI / 180 * lon1;
  lon2 = M_PI / 180 * lon2;

  a = sq( sin((lat2 - lat1) / 2.) ) +  //  haversine formula, http://www.movable-type.co.uk/scripts/latlong.html
      sq( sin((lon2 - lon1) / 2.) ) *
      cos((lat1)) * cos((lat2));
  c = 2 * atan2( sqrt(a), sqrt(1 - a));
  d = R * c;
  return d; //  distance in meters
}


float gpsToXY(float currentLatitude, float currentLongitude, float currentTrackAngle, float refLatitude, float refLongitude, float refTrackAngle) {
  unsigned long const EARTH_RADIUS = 6371e3;  // Earth radius, m

  float latRad = currentLatitude * M_PI / 180.0;
  float lonRad = currentLongitude * M_PI / 180.0;
  float refLatRad = refLatitude * M_PI / 180.0;
  float refLonRad = refLongitude * M_PI / 180.0;

  float xG = EARTH_RADIUS * (lonRad - refLonRad) * cos(refLatRad);
  float yG = EARTH_RADIUS * (latRad - refLatRad);
  float thetaG = currentTrackAngle - refTrackAngle;

  return xG, yG, thetaG;
}