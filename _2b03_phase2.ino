#include <TimeLib.h>

// Change these values based on the location
#define LATITUDE_DEGREES 43.2557 // north
#define LONGITUDE_DEGREES 79.8711 // west, should this be negative?
#define DELTA_GMT -4 // From GMT to EST

#define DEGREES_TO_RADS M_PI / 180
#define RADS_TO_DEGREES 180 / M_PI

#define LATITUDE_RADS LATITUDE_DEGREES * DEGREES_TO_RADS
#define LONGITUDE_RADS LATITUDE_DEGREES * DEGREES_TO_RADS

#define SLEEP_INTERVAL 10000 // value in ms


void setup() {
  Serial.begin(9600);
  Serial.println("+---------------------------------------------------+");
  Serial.println("| Starting Azimuth and Zenith Angle Calculations    |");
  Serial.println("+---------------------------------------------------+");
  Serial.println("| Static Variables:                                 |");
  Serial.println("|                                                   |");
  Serial.print("|     Latitude: ");
  Serial.print(LATITUDE_DEGREES);
  Serial.print(" deg                         |\n");
  Serial.print("|     Longitude: ");
  Serial.print(LONGITUDE_DEGREES);
  Serial.print(" deg                        |\n");
  Serial.print("|     Delta GMT: ");
  Serial.print(DELTA_GMT);
  Serial.print("                                 |\n");
  Serial.println("|                                                   |");
  Serial.println("+---------------------------------------------------+");
  Serial.println("");

  // todo: Reset to initial home position
}

void loop() {
 time_t currentDateTime = now();
 Serial.print("Current time = ");
 Serial.print(hour(currentDateTime));
 Serial.print(":");
 Serial.print(minute(currentDateTime));
 Serial.print(":");
 Serial.print(second(currentDateTime));
 Serial.print("\n");

 Serial.print("Zenith angle = ");
 double zenith = getZenithAngleRads(currentDateTime);
 Serial.print(RADS_TO_DEGREES * zenith);
 Serial.print(" deg\n");

 Serial.print("Azimuth angle = ");
 double azimuth = getAzimuthAngleRads(currentDateTime);
 Serial.print(RADS_TO_DEGREES * azimuth);
 Serial.print(" deg\n");

 Serial.println("");

 // todo: Move motors as required

 delay(SLEEP_INTERVAL);
}


int getDayOfYear(time_t now) {
  int m = month();
  int d = day(now);
  
  if (m == 1) { // Jan
    return d;
  } else if (m == 2) { // Feb
    return 31 + d;
  } else if (m == 3) { // Mar
    return 59 + d;
  } else if (m == 4) { // Apr
    return 90 + d;
  } else if (m == 5) { // May
    return 120 + d;
  } else if (m == 6) { // June
    return 151 + d;
  } else if (m == 7) { // July
    return 181 + d;
  } else if (m == 8) { // Aug
    return 212 + d;
  } else if (m == 9) { // Sept
    return 243 + d;
  } else if (m == 10) { // Oct
    return 273 + d;
  } else if (m == 11) { // Nov
    return 304 + d;
  } else { // Dec
    return 334 + d;
  }
}

// LSTM
double getLocalStandardMeridianDegrees() {
  return 15 * DELTA_GMT; // this is currently in degrees
}

// EoT
double getEquationOfTime(int d) { // d = day of the year (Jan 1st = 1)
  double b = (360 / 365) * (d - 81);
  return 9.87 * sin(2 * b) - 7.53 * cos(b) - 1.5 * sin(b); // todo: Should verify that this is supposed to be in rads
}

// TC
double getTimeCorrectionFactor(int d) {
  double eot = getEquationOfTime(d);
  double lstm = getLocalStandardMeridianDegrees();

  return 4 * (LONGITUDE_DEGREES - lstm) + eot;
}

// LST
double getLocalStandardTime(time_t currentDateTime) {
  int d = getDayOfYear(currentDateTime);
  int h = hour(currentDateTime);
  int m = minute(currentDateTime);
  int s = second(currentDateTime);

  double localTime = h + m / 60 + s / 3600;

  return localTime + getTimeCorrectionFactor(d) / 60;
}

// HRA
double getHourAngleRads(time_t currentDateTime) {
  return DEGREES_TO_RADS * 15 * (getLocalStandardTime(currentDateTime) - 12);
}

// del
double getDeclinationAngleRads(int d) {
  return -23.45 * DEGREES_TO_RADS * cos( (d + 10) * 360 / 365 );
}

// alpha
double getElevationAngleRads(int d) {
  double del = getDeclinationAngleRads(d);

  return asin( sin(del * DEGREES_TO_RADS) * sin(LATITUDE_DEGREES * DEGREES_TO_RADS) + cos(del) * cos(LATITUDE_DEGREES * DEGREES_TO_RADS) ); 
}

// z thingy?
double getZenithAngleRads(time_t currentDateTime) {
  int d = getDayOfYear(currentDateTime);
  return M_PI_2 - getElevationAngleRads(d);
}

// Azimuth
double getAzimuthAngleRads(time_t currentDateTime) {
  int d = getDayOfYear(currentDateTime);
  double del = getDeclinationAngleRads(d);
  double alpha = getElevationAngleRads(d);

  return acos( ( sin(del) * cos(LATITUDE_RADS) - cos(del) * sin(LATITUDE_RADS) * cos(getHourAngleRads(currentDateTime)) ) / cos(alpha) );
}
