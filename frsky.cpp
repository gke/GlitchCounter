#include "Arduino.h"

#define FSHUB_SENTINEL       0x5e

//_________________________________________________________________________________

// Dummy externals

#define GRAVITY_MPS_S_R (1.0/9.81)
#define RadiansToDegrees(v) ((v*PI)/180.0)
#define Limit(v,l,h) (0)

enum {X,Y,Z};
enum {Roll, Pitch, Yaw};

float Acc[3], Rate[3], Angle[3], Heading, DensityAltitude, OriginAltitude, ROC, MPU6XXXTemperature, BatteryVolts, BatteryCurrent;
uint8_t BatteryCellCount;
int32_t BaroTemperature, BaroPressure;
float BatteryChargeUsedmAH, BatteryCapacitymAH;

struct {
  boolean BaroActive, GPSValid, HaveGPS, OriginValid;
} F;

struct {
  float Distance,Bearing,Elevation,Hint;
} Nav;

enum {NorthC, EastC};

struct {
  uint8_t noofsats, fix;
  float heading;
  float altitude;
  float gspeed;
  struct {
    uint32_t Raw;
  } C[3];
} GPS;

//_________________________________________________________________________________

enum {
  // Data IDs  (BP = before decimal point; AP = after decimal point)

  FSHUB_ID_GPS_ALT_BP = 0x01,
  FSHUB_ID_TEMP1 = 0x02, // barometer deg C
  FSHUB_ID_RPM = 0x03,
  FSHUB_ID_FUEL = 0x04,
  FSHUB_ID_TEMP2 = 0x05,
  FSHUB_ID_VOLTS = 0x06, // cells

  // 0x07
  // 0x08

  FSHUB_ID_GPS_ALT_AP = 0x09,
  FSHUB_ID_BARO_ALT_BP = 0x10,

  // 0x0A
  // 0x0B
  // 0x0C
  // 0x0D
  // 0x0E
  // 0x0F // seems to be emitted when there is a buffer overrun in the Rx.

  FSHUB_ID_GPS_SPEED_BP = 0x11,
  FSHUB_ID_GPS_LONG_BP = 0x12,
  FSHUB_ID_GPS_LAT_BP = 0x13,
  FSHUB_ID_GPS_COURS_BP = 0x14,

  FSHUB_ID_GPS_DAY_MONTH = 0x15,
  FSHUB_ID_GPS_YEAR = 0x16,
  FSHUB_ID_GPS_HOUR_MIN = 0x17,
  FSHUB_ID_GPS_SEC = 0x18,

  FSHUB_ID_GPS_SPEED_AP = 0x19, // +8 from BP
  FSHUB_ID_GPS_LONG_AP = 0x1A,
  FSHUB_ID_GPS_LAT_AP = 0x1B,
  FSHUB_ID_GPS_COURS_AP = 0x1C,

  UAVX_ID_GPS_STAT = 0x1d,

  // 0x1e

  // 0x1f
  // 0x20

  FSHUB_ID_BARO_ALT_AP = 0x21,

  FSHUB_ID_GPS_LONG_EW = 0x22,
  FSHUB_ID_GPS_LAT_NS = 0x23,

  FSHUB_ID_ACCEL_X = 0x24, // m/s2
  FSHUB_ID_ACCEL_Y = 0x25,
  FSHUB_ID_ACCEL_Z = 0x26,

  FSHUB_ID_CURRENT = 0x28,

  UAVX_ID_WHERE_DIST = 0x29, // metres the aircraft is way
  UAVX_ID_WHERE_BEAR = 0x2a, // bearing (deg) to aircraft
  UAVX_ID_WHERE_ELEV = 0x2b, // elevation (deg) of the aircraft above the horizon
  UAVX_ID_WHERE_HINT = 0x2c, // which to turn to come home intended for voice guidance

  UAVX_ID_COMPASS = 0x2d, // deg
  // 0x2e
  // 0x2f

  FSHUB_ID_VARIO = 0x30, // cm/sec

  //--------------------

  // UAVX user defined

  UAVX_ID_GYRO_X = 0x31, // deg/sec
  UAVX_ID_GYRO_Y = 0x32,
  UAVX_ID_GYRO_Z = 0x33,

  UAVX_ID_PITCH = 0x34, // deg
  UAVX_ID_ROLL = 0x35,

  UAVX_ID_MAH = 0x36, // mAH battery consumption

  //--------------------

  FSHUB_ID_VFAS = 0x39,
  FSHUB_ID_VOLTS_BP = 0x3A,
  FSHUB_ID_VOLTS_AP = 0x3B,
  FSHUB_ID_FRSKY_LAST = 0x3F,

};

extern void TxChar(uint8_t s, uint8_t v);


uint32_t FrSkyDLinkuS;

uint16_t MakeFrac(float v, uint16_t s) {
  return (abs((int32_t)(v * s)) % s);
} // MakeFrac

void TxFrSkyHubHeader(uint8_t s) {
  TxChar(s, FSHUB_SENTINEL);
} // TxFrSkyHeader

void TxFrSky(uint8_t s, uint8_t data) {
  // byte stuffing
  if (data == 0x5e) {
    TxChar(s, 0x5d);
    TxChar(s, 0x3e);
  } else if (data == 0x5d) {
    TxChar(s, 0x5d);
    TxChar(s, 0x3d);
  } else
    TxChar(s, data);
} // TxFrSky


void TxFrSky16(uint8_t s, int16_t a) {
  TxFrSky(s, a);
  TxFrSky(s, (a >> 8) & 0xff);
} // TxFrSky16


void TxFrSkyHubPacket(uint8_t s, uint8_t appID, int16_t v) {
  TxFrSkyHubHeader(s);
  TxFrSky(s, appID);
  TxFrSky16(s, v);
} // TxFrSkyHubPacket

void TxFrSkyHubPacketPair(uint8_t s, uint8_t id1, uint8_t id2, int16_t v, int8_t frac) {
  TxFrSkyHubPacket(s, id1, v);
  TxFrSkyHubPacket(s, id2, MakeFrac(v, frac));
}

void TxFrSkyHubAcc(uint8_t s) {
  uint8_t a;

  for (a = X; a <= Z; a++)
    TxFrSkyHubPacket(s, FSHUB_ID_ACCEL_X + a, Acc[a] * GRAVITY_MPS_S_R
        * 1000.0f);
} // TxFrSkyHubAcc

void TxFrSkyHubGyro(uint8_t s) {
  uint8_t a;

  for (a = Pitch; a <= Yaw; a++)
    TxFrSkyHubPacket(s, UAVX_ID_GYRO_X + a, RadiansToDegrees(Rate[a]));
} // TxFrSkyHubAcc

void TxFrSkyHubAttitude(uint8_t s) {
  uint8_t a;

  for (a = Pitch; a <= Roll; a++)
    TxFrSkyHubPacket(s, UAVX_ID_PITCH + a, RadiansToDegrees(Angle[a]));

} // TxFrSkyHubAttitude

void TxFrSkyHubBaro(uint8_t s) {

  TxFrSkyHubPacketPair(s, FSHUB_ID_BARO_ALT_BP, FSHUB_ID_BARO_ALT_AP,
      DensityAltitude - OriginAltitude, 10);
} //  TxFrSkyHubBaro

void TxFrSkyHubVario(uint8_t s) {
  TxFrSkyHubPacket(s, FSHUB_ID_VARIO, ROC * 100.0f);
} //  TxFrSkyHubBaro

void TxFrSkyHubTemperature1(uint8_t s) {
  TxFrSkyHubPacket(s, FSHUB_ID_TEMP1, BaroTemperature);
} // TxFrSkyHubTemperature1

void TxFrSkyHubFuel(uint8_t s) {
  TxFrSkyHubPacket(
      s,
      FSHUB_ID_FUEL,
      Limit(100 * ( 1.0f - BatteryChargeUsedmAH / BatteryCapacitymAH), 0, 100));
} // TxFrSkyHubFuel

void TxFrSkyHubmAH(uint8_t s) {
  TxFrSkyHubPacket(s, UAVX_ID_MAH, BatteryChargeUsedmAH);
} // TxFrSkyHubmAH

void TxFrSkyHubTemperature2(uint8_t s) {
  TxFrSkyHubPacket(s, FSHUB_ID_TEMP2, MPU6XXXTemperature);
} // TxFrSkyHubTemperature2

void TxFrSkyHubTime(uint8_t s) {
  uint32_t seconds = millis() / 1000;
  uint8_t minutes = (seconds / 60) % 60;

  // if we fly for more than an hour, something's wrong anyway
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_HOUR_MIN, minutes << 8);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_SEC, seconds % 60);
} // TxFrSkyHubTime

void TxFrSkyHubWhere(uint8_t s) {
  if ((Nav.Distance >= 0.0) && (Nav.Distance < 32000.0f)) {
    TxFrSkyHubPacket(s, UAVX_ID_WHERE_DIST, Nav.Distance);
    TxFrSkyHubPacket(s, UAVX_ID_WHERE_BEAR, RadiansToDegrees(Nav.Bearing));
    TxFrSkyHubPacket(s, UAVX_ID_WHERE_ELEV, RadiansToDegrees(Nav.Elevation));
    TxFrSkyHubPacket(s, UAVX_ID_WHERE_HINT, RadiansToDegrees(Nav.Hint));
  }
}

// FrSky uses NMEA form rather than computationally sensible decimal degrees

typedef struct {
  uint16_t bp, ap;
} pair_rec;

static void GPStoDDDMM_MMMM(int32_t L, pair_rec * c) {
  uint32_t d, mf, dm, m;

  L = abs(L);
  d = L / 10000000L;
  dm = (L % 10000000L) * 60;
  m = dm / 10000000L;
  mf = dm - m * 10000000L;

  c->bp = d * 100 + m;
  c->ap = mf / 1000L; // limited precision
}

void TxFrSkyHubGPSStat(uint8_t s) {
  TxFrSkyHubPacket(s, UAVX_ID_GPS_STAT, GPS.noofsats * 1000 + GPS.fix * 100
      + (F.GPSValid & 1) * 10 + (F.OriginValid & 1));
} // TxFrSkyHubGPSStat

void TxFrSkyHubGPSCoords(uint8_t s) {
  pair_rec c;

  GPStoDDDMM_MMMM(GPS.C[NorthC].Raw, &c);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_BP, c.bp);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_AP, c.ap);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_NS, GPS.C[NorthC].Raw < 0 ? 'S' : 'N');

  GPStoDDDMM_MMMM(GPS.C[EastC].Raw, &c);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_BP, c.bp);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_AP, c.ap);
  TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_EW, GPS.C[EastC].Raw < 0 ? 'W' : 'E');

} // TxFrSkyHubGPS

void TxFrSkyHubGPSSpeed(uint8_t s) {
  TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_SPEED_BP, FSHUB_ID_GPS_SPEED_AP,
      GPS.gspeed * 3.6f, 10);
} // TxFrSkyHubGPSSpeed

void TxFrSkyHubGPSAlt(uint8_t s) {
  TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_ALT_BP, FSHUB_ID_GPS_ALT_AP,
      GPS.altitude, 10);
} // TxFrSkyHubGPSAlt

void TxFrSkyHubCellVoltages(uint8_t s) {

  static uint16_t currentCell = 0;
  uint32_t cellVoltage;
  uint16_t payload;

  // A cell packet is formated this way: https://github.com/jcheger/frsky-arduino/blob/master/FrskySP/FrskySP.cpp
  // content    | length
  // ---------- | ------
  // volt[id]   | 12-bits
  // celltotal  | 4 bits
  // cellid     | 4 bits

  cellVoltage = (BatteryVolts * 500.0f) / BatteryCellCount;

  payload = ((cellVoltage & 0x0ff) << 8) | (currentCell << 4) | ((cellVoltage
      & 0xf00) >> 8);

  TxFrSkyHubPacket(s, FSHUB_ID_VOLTS, payload);

  if (++currentCell >= BatteryCellCount)
    currentCell = 0;

} // TxFrSkyHubCellVoltages

void TxFrSkyHubVoltage(uint8_t s) {
  TxFrSkyHubPacketPair(s, FSHUB_ID_VOLTS_BP, FSHUB_ID_VOLTS_AP, BatteryVolts
      * 0.5f, 100);
} // TxFrSkyHubVoltage

void TxFrSkyHubCurrent(uint8_t s) {
  TxFrSkyHubPacket(s, FSHUB_ID_CURRENT, BatteryCurrent * 10);
} // TxFrSkyHubCurrent

void TxFrSkyHubGPSHeading(uint8_t s) {

  TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_COURS_BP, FSHUB_ID_GPS_COURS_AP,
      RadiansToDegrees(GPS.heading), 10);
  //RadiansToDegrees(Heading), 10);

} // TxFrSkyHubGPSHeading

void TxFrSkyHubCompassHeading(uint8_t s) {

  TxFrSkyHubPacket(s, UAVX_ID_COMPASS, RadiansToDegrees(Heading));
} // TxFrSkyHubCompassHeading


void SendFrSkyHubTelemetry(uint8_t s) {
  static uint8_t FrameCount = 0;

  if (++FrameCount == 40) { // FRAME 3 every 8 seconds
    //TxFrSkyHubTime(s); // 2
    TxFrSkyHubTemperature1(s); // 1
    //TxFrSkyHubTemperature2(s); // 1
    //TxFrSkyHubFuel(s);
    TxChar(s, FSHUB_SENTINEL);

    FrameCount = 0;

  } else if ((FrameCount % 5) == 0) { // FRAME 2 every second
    if (F.GPSValid) {
      if (F.OriginValid)
        TxFrSkyHubWhere(s); // 4
      TxFrSkyHubGPSSpeed(s); // 2
      TxFrSkyHubGPSAlt(s); // 2
      TxFrSkyHubGPSHeading(s); // 2
      TxFrSkyHubGPSCoords(s); // 6
      TxChar(s, FSHUB_SENTINEL);
    }
    TxFrSkyHubGPSStat(s); // 1
    //TxFrSkyHubCompassHeading(s); // 2 2-> 17
  } else { // FRAME 1 every 200mS
    TxFrSkyHubBaro(s); // 2
    TxFrSkyHubVario(s); // 1
    TxFrSkyHubVoltage(s); // 2
    TxFrSkyHubCellVoltages(s); // 1
    TxFrSkyHubCurrent(s); // 1
    TxFrSkyHubmAH(s); // 1
    //TxFrSkyHubAcc(s); // 3 could add for coordinated turns?
    TxFrSkyHubGyro(s); // 3
    TxFrSkyHubAttitude(s); // 2 ~ 14
    TxChar(s, FSHUB_SENTINEL);
  }

} // TxFrSkyHubTelemetry

//_____________________________________________________________________________________

// Specific to Glitch Counter

extern uint32_t GlitchCount[];

void SendFrSkyGlitchTelemetry(uint8_t s) {

    TxFrSkyHubPacket(s, FSHUB_ID_TEMP1, GlitchCount[0]); 
    TxFrSkyHubPacket(s, FSHUB_ID_TEMP2, GlitchCount[1]);
    TxChar(s, FSHUB_SENTINEL);
}
