// Universal? Glitch Counter
// Prof Greg Egan August 2019

//____________________________________________________________________________________
#define RIGOROUS // interval between frames checked and counted as glitches
//#define FAST_PW_DISPLAY // display pulse width immediately it changes
//#define FULL_DISPLAY // comment to suppress all except PW, Count and max fail time.
#define RECOVERY_PULSES 20 // number of correct pulses to recover @~22mS - mainly PPM

// comment/uncomment ONE of the below to suit your battery units 1/10 of a volt
#define BATTERY_MIN 38 // 1s LiPo
//#define BATTERY_MIN 44 // 4s NiMH?

//#define R1 1000 // tweak to calibrate volts - it is not the resistor value ;)
#define R1 900 // tweak to calibrate volts - it is not the resistor value ;)

//____________________________________________________________________________________


#define BATTERY_UPDATE_MS 1000
#define TELEMETRY_UPDATE_MS 1000

#include "SSD1X06.h"

#define NO_CHANGE_TIMEOUT_MS 20000 // mS.
#define INIT_FRAMES 60 // number of initial  frames to allow filters to settle
#define FRAME_MARGIN_US 500
#define MARGIN_US  200 // could set zero or less for FrSky CPPM
#define MIN_WIDTH_US (1000-MARGIN_US) // temporarily to prevent wraparound 900
#define MAX_WIDTH_US (2000+MARGIN_US)

#define FRAME_TIMEOUT_US 25000L
#define SIGNAL_TIMEOUT_US  (FRAME_TIMEOUT_US*5)

#define Limit(i,l,u)   (((i) < l) ? l : (((i) > u) ? u : (i)))
#define Limit1(i,l)   (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))

#define C 1
#define T 0

volatile boolean InGlitch[2], Signal[2];
volatile uint32_t PrevEdge[2], Edge[2], FailsafeStartuS[2], FailTimeuS[2], MaxFailTimeuS[2];
uint32_t NominalFrameIntervaluS[2], MinFrameIntervaluS[2], MaxFrameIntervaluS[2], FrameIntervaluS[2];
uint32_t LastFrameuS[2];
uint32_t GlitchCount[2], PWErrorCount[2], FrameErrorCount[2], FailsafeCount[2], Recover[2];
uint32_t WidthuS[2];

void refresh_oled(void) {
  uint8_t c;

  SSD1X06::start();
  delay(300);
  SSD1X06::fillDisplay(' ');
  delay(2000);
  SSD1X06::displayString6x8(0, 0, F("GLITCH COUNTER"), 0);

  SSD1X06::displayString6x8(2, 0, F("?TEST 0"), 0);
  SSD1X06::displayString6x8(4, 0, F("?SPCM 0"), 0);

#if defined(FULL_DISPLAY)
  SSD1X06::displayString6x8(6, 0, F("FS="), 0);
  SSD1X06::displayString6x8(6, 7, F("PE="), 0);
  SSD1X06::displayString6x8(6, 14, F("FE="), 0);


  SSD1X06::displayString6x8(7, 10, F("FI="), 0);
#endif
  SSD1X06::displayString6x8(7, 0, F("PW="), 0);

  delay(2000); // start up message

}


#define analogpin A0
const int32_t vrefdecivolts = 33;
const int R2 = 1000;
int batteryscale, decivolts;

void updateBattery() {
  static uint32_t updateBatterymS = 0;

  if (millis() > updateBatterymS) {
    updateBatterymS = millis() + BATTERY_UPDATE_MS;
    decivolts = (decivolts * 3 + analogRead(analogpin) / batteryscale) >> 2;
    SSD1X06::displayReal32(0, 15, decivolts, 1, 'V');
  }
}

extern void SendFrSkyGlitchTelemetry(uint8_t s);

void TxChar(uint8_t s, uint8_t v) {
  Serial.write(v);
}

void updateTelemetry(void) {
  static uint32_t FrSkyUpdatemS = 0;
  uint8_t c;

  if (millis() >= FrSkyUpdatemS) {

    for (c = 0; c < 2; c++)
      GlitchCount[c] = FailsafeCount[c] + PWErrorCount[c] + FrameErrorCount[c];

    FrSkyUpdatemS = millis() + TELEMETRY_UPDATE_MS;
    SendFrSkyGlitchTelemetry(0);
  }
}

void updateOLED() {
  static uint32_t GlitchCountP[2] = {0, 0};
  static uint32_t FailsafeCountP[2] = {0, 0};
  static uint32_t  PWErrorCountP[2] = {0, 0};
  static uint32_t  FrameErrorCountP[2] = {0, 0};
  static uint32_t   MaxFailTimeuSP[2] = {0, 0};
  static boolean SignalP[2] = {true, true};
  static uint32_t updateOLEDmS = 0;
  static uint32_t FastP = 0;
  uint8_t c;

#if defined(FAST_PW_DISPLAY)
  if (WidthuS[0] != FastP) {
    SSD1X06::displayString6x8(7, 3, F("    "), 0);
    SSD1X06::displayInt32(7, 3,  Limit(WidthuS[0], 0, 9999));
    FastP = WidthuS[0];
  }
#endif

  if (millis() > updateOLEDmS) {
    updateOLEDmS = millis() + 1000;

    for (c = 0; c < 2; c++) {

      // line 2
      if (Signal[c] != SignalP[c]) {
        if (Signal[c])
          SSD1X06::displayString6x8(2 + (c * 2), 0, F("*"), 0);
        else
          SSD1X06::displayString6x8(2 + (c * 2), 0, F("?"), 0);
        SignalP[c] = Signal[c];
      }
      // line 3+
      GlitchCount[c] = FailsafeCount[c] + PWErrorCount[c] + FrameErrorCount[c];
      if (GlitchCount[c] != GlitchCountP[c])
        SSD1X06::displayInt32(2 + (c * 2), 6,  Limit(GlitchCount[c], 0, 9999));
      if (MaxFailTimeuS[c] != MaxFailTimeuSP[c]) {
        SSD1X06::displayString6x8(2 + (c * 2), 13, F("        "), 0);
        SSD1X06::displayReal32(2 + (c * 2), 13, MaxFailTimeuS[c] / 100000L, 1, 's');
      }
    }

#if defined(FULL_DISPLAY)

    // line 6
    if (FailsafeCount[T] > FailsafeCountP[T])
      SSD1X06::displayInt32(6, 3,  Limit(FailsafeCount[T], 0, 999));
    if (PWErrorCount[T] > PWErrorCountP[T])
      SSD1X06::displayInt32(6, 10,  Limit(PWErrorCount[T], 0, 999));
    if (FrameErrorCount[T] > FrameErrorCountP[T])
      SSD1X06::displayInt32(6, 17,  Limit(FrameErrorCount[T], 0, 999));

    // line 7
    SSD1X06::displayString6x8(7, 13, F("      "), 0);
    SSD1X06::displayInt32(7, 13,  Limit(FrameIntervaluS[T], 0, 999999));

#if !defined(FAST_PW_DISPLAY)
    SSD1X06::displayString6x8(7, 3, F("    "), 0);
    SSD1X06::displayInt32(7, 3,  Limit(WidthuS[T], 0, 9999));
#endif
#else
#if !defined(FAST_PW_DISPLAY)
    SSD1X06::displayString6x8(7, 3, F("    "), 0);
    SSD1X06::displayInt32(7, 3,  Limit(WidthuS[T], 0, 9999));
#endif
    SSD1X06::displayString6x8(7, 8, F("    "), 0);
    SSD1X06::displayInt32(7, 8,  Limit(WidthuS[C], 0, 9999));
#endif

    FailsafeCountP[T] = FailsafeCount[T];
    PWErrorCountP[T] = PWErrorCount[T];
    FrameErrorCountP[T] = FrameErrorCount[T];
    MaxFailTimeuSP[T] = MaxFailTimeuS[T];

  }

}

void CaptureFrameInterval(uint8_t c) {

  NominalFrameIntervaluS[c] = (NominalFrameIntervaluS[c] + (Edge[c] - LastFrameuS[c])) / 2;
  MinFrameIntervaluS[c] = NominalFrameIntervaluS[c] - FRAME_MARGIN_US;
  MaxFrameIntervaluS[c] = NominalFrameIntervaluS[c] + FRAME_MARGIN_US;
  MaxFailTimeuS[c] = 0;
  FailsafeCount[c] = FrameErrorCount[c] = PWErrorCount[c] = 0;
  FailsafeStartuS[c] = LastFrameuS[c] = PrevEdge[c] = Edge[c];

}
boolean WidthOK(uint32_t pw, uint32_t l, uint32_t h) {
  return (pw > l) && (pw < h);
}

boolean IntervalOK(uint8_t c, uint32_t interval) {
  return (interval > MinFrameIntervaluS[c]) && (interval < MaxFrameIntervaluS[c]);
}

void StartGlitchTimer(uint8_t c, uint32_t * count) {
  if (!InGlitch[c]) {
    *count += 1;
    Recover[c] = RECOVERY_PULSES;
    FailsafeStartuS[c] = Edge[c];
    InGlitch[c] = true;
  }
}

#define test_interrupt_pin (2)
#define control_interrupt_pin (3)


void updateControlRxState() {
  static uint8_t Startup = 100;

  Edge[C] = micros();

  if (Startup > 0) {
    if (digitalRead(control_interrupt_pin)) { // start of pulse

      Startup--;
      CaptureFrameInterval(C);

    }
  } else {
    if (digitalRead(control_interrupt_pin)) {// leading edge of pulse

      PrevEdge[C] = Edge[C];
      LastFrameuS[C] = Edge[C];

    } else {

      WidthuS[C] = Edge[C] - PrevEdge[C];
      PrevEdge[C] = Edge[C];

      if (WidthOK(WidthuS[C], MIN_WIDTH_US, MAX_WIDTH_US)) {
        if (WidthOK(WidthuS[C], 800, 1200)) {
          // if (Recover[C] > 0)
          // Recover[C]--;
          // else
          InGlitch[C] = false;
        } else
          StartGlitchTimer(C, &FailsafeCount[C]);
      } else
        StartGlitchTimer(C, &PWErrorCount[C]);
    }

    Signal[C] = true;

  }

  if (InGlitch[C]) {
    FailTimeuS[C] = (Edge[C] - FailsafeStartuS[C]);
    if (FailTimeuS[C] > MaxFailTimeuS[C]) MaxFailTimeuS[C] = FailTimeuS[C];
  }

}

void updateTestRxState() {
  static uint8_t Startup = 100;

  Edge[T] = micros();

  if (Startup > 0) {
    if (digitalRead(test_interrupt_pin)) { // start of pulse

      Startup--;
      CaptureFrameInterval(T);

    }
  } else {

    if (digitalRead(test_interrupt_pin)) {

      FrameIntervaluS[T] = Edge[T] - LastFrameuS[T];

      if (!IntervalOK(T, FrameIntervaluS[T]))
#if defined(RIGOROUS)
        StartGlitchTimer(T, &FrameErrorCount[T]);
#else
        FrameErrorCount[T]++;
#endif
      LastFrameuS[T] = PrevEdge[T] = Edge[T];

    } else { // start of pulse

      WidthuS[T] = Edge[T] - PrevEdge[T];
      PrevEdge[T] = Edge[T];

      if (WidthOK(WidthuS[T], MIN_WIDTH_US, MAX_WIDTH_US)) {
        if (WidthOK(WidthuS[T], 800, 1200)) {

          if (Recover[T] > 0)
            Recover[T]--;
          else
            InGlitch[T] = false;

        } else
          StartGlitchTimer(T, &FailsafeCount[T]);
      } else
        StartGlitchTimer(T, &PWErrorCount[T]);
    }

    Signal[T] = true;

  }

  if (InGlitch[T]) {
    FailTimeuS[T] = (Edge[T] - FailsafeStartuS[T]);
    if (FailTimeuS[T] > MaxFailTimeuS[T]) MaxFailTimeuS[T] = FailTimeuS[T];
  }

}

void setup() {
  uint8_t c;

  analogReference(EXTERNAL); // MUST BE CALLED BEFORE analogRead otherwise smoke!

  batteryscale = (vrefdecivolts * R1) / (R1 + R2);
  decivolts = analogRead(analogpin) / batteryscale;

  Serial.begin(9600);

  refresh_oled();

  for (c = 0; c < 2; c++) {
    FrameIntervaluS[c] =  LastFrameuS[T] = 0;
    PWErrorCount[c] = FrameErrorCount[c] = FailsafeCount[c] = Recover[c] = 0;
    FailsafeStartuS[c] =  MaxFailTimeuS[c] = 0;
    NominalFrameIntervaluS[c] = 22000;
    WidthuS[c] = 0;
    Signal[c] = InGlitch[c] = false;
  }

  pinMode(control_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(control_interrupt_pin), updateControlRxState, CHANGE);
  pinMode(test_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(test_interrupt_pin), updateTestRxState, CHANGE);

  interrupts();
}

void loop() {

  Signal[C] = (micros() < (LastFrameuS[C] + SIGNAL_TIMEOUT_US));
  Signal[T] = (micros() < (LastFrameuS[T] + SIGNAL_TIMEOUT_US));

  updateOLED();

  updateTelemetry();
  updateBattery();


}
