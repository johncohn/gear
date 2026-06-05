/******************************************************************************
  gears.ino

  Version: 5.3
  Last Modified: 2026-06-05

  Overview:
    An interactive kinetic sculpture controller. An STHS34PF80 IR presence
    sensor detects a person's distance (up to ~3m). That signal drives 16
    continuous-rotation servos via a PCA9685 PWM driver — each servo has an
    individually configured direction and speed ramp so the whole assembly
    responds differently to the same distance reading. When nobody is present
    all servos are stopped; they spin up as a person approaches, with
    alternating directions and layered speed responses across the 16 channels.

  Hardware:
    Adafruit QT Py RP2040
    STEMMA QT / Qwiic chain on Wire1 (I2C):
      0x5A  SparkFun STHS34PF80 human presence & motion sensor
      0x40  Adafruit PCA9685 16-channel 12-bit PWM servo driver
    16x continuous-rotation servos on PCA9685 channels 0–15

  Required Libraries (install via Arduino Library Manager):
    SparkFun STHS34PF80 Arduino Library
    Adafruit PWM Servo Driver Library

  Terminal / Serial Monitor:
    The live display uses ANSI escape codes to redraw in-place and will NOT
    render correctly in the Arduino IDE Serial Monitor. Use instead:
      macOS:   screen /dev/cu.usbmodem<PORT> 115200   (exit: Ctrl-A then K)
      Windows: PuTTY or CoolTerm in VT100/ANSI mode at 115200 baud

  Signal Calibration (STHS34PF80 raw presenceVal on this unit):
    ~6000  at 1 ft
    ~2100  at 3 ft
    ~0     at ~2 m (signal crosses zero; negative values beyond this)
    MAX_SIGNAL = 10000 gives headroom for sub-1ft distances

  Servo Config Quick Reference (see ServoConfig table below):
    changeover : emaVal where this servo is stopped. 0 = stopped when nobody
                 detected, spins up as person approaches.
    slope      : PWM counts per raw emaVal unit.
                 Positive → CW when close.  Negative → CCW when close.
                 Larger magnitude → faster / longer-range response.

  Changelog:
    v5.3 (2026-06-05) - Fix: use abs(emaVal) in slope-intercept formula so
                        far-range negative sensor readings map the same as
                        positive ones. 8+/8- slopes guarantee always-mixed
                        CW/CCW at any distance; all 16 channels reverse
                        somewhere in the 3m→1ft detection range.
    v5.2 (2026-06-05) - Restore slope-intercept formula: slope*(emaVal-changeover)
                        gives true direction reversal at each servo's changeover
                        point. Changeovers spread 100-5200 so cascade of reversals
                        sweeps through all 16 channels as person walks in from 3m.
                        Immediate shutdown (no countdown) prevents idle spinning;
                        EMA 300ms smoothing debounces dropouts.
    v5.1 (2026-06-05) - All changeovers=0: speed proportional to signal so servos
                        are always stopped at idle and ramp up immediately from
                        any detectable distance. PRESENCE_CUTOFF 30→75 to reject
                        thermal background noise. SHUTDOWN_DELAY 5s→3s, ramp 3s→1.5s.
                        Alternating CW/CCW pairs; uniform slope=0.050 as baseline.
    v5.0 (2026-06-05) - Auto-shutdown after 5s no presence; 3s soft-on ramp when
                        waking; changeover points spread across detection range so
                        each servo pair reverses at a different distance; status
                        line in display: [OFF], [RAMP xx%], [ACTIVE], countdown.
    v4.3 (2026-05-31) - Sensor is now optional. If STHS34PF80 not found, runs
                        a 6-second CW/CCW sweep on all PCA9685 channels to
                        verify servo driver communication without the sensor.
    v4.2 (2026-05-31) - Set all changeovers to 0: servos stopped when nobody
                        present, spin up as person approaches. Varied slope
                        magnitudes for layered activation at different ranges.
    v4.1 (2026-05-31) - ANSI terminal display: 16 centered servo bars + distance
                        bar, redrawn in place at 2Hz.
    v4.0 (2026-05-31) - Add PCA9685 16-channel PWM servo driver.
    v3.3 (2026-05-31) - Lower EMA_ALPHA 0.25→0.1 (~300ms smoothing).
    v3.2 (2026-05-31) - Fixed MAX_SIGNAL=10000 (calibrated: 1ft≈6000, 3ft≈2100).
    v3.0 (2026-05-31) - Use abs(emaVal) so negative far-range values are used.
    v2.4 (2026-05-31) - ASCII bar graph distance display.
    v1.0 (2023-09-19) - Initial release.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
******************************************************************************/

#include "SparkFun_STHS34PF80_Arduino_Library.h"
#include "Adafruit_PWMServoDriver.h"
#include <Wire.h>

// ---------------------------------------------------------------------------
// DISTANCE SENSOR SETTINGS
// ---------------------------------------------------------------------------

// EMA smoothing on raw signal. 0.1 at 30Hz ≈ 300ms.
#define EMA_ALPHA       0.1f

// abs(raw) at closest expected range = 0 bars (calibrated: 1ft≈6000, 3ft≈2100).
#define MAX_SIGNAL      10000

// Total bar chars for the serial display (fits on a 120-col line).
#define MAX_BARS        100

// abs(emaVal) below this = nobody home.
#define PRESENCE_CUTOFF  75

// Soft-on ramp duration when waking from shutdown (ms).
#define SOFTON_MS          1500

// ---------------------------------------------------------------------------
// SERVO SETTINGS  (apply to all channels)
// ---------------------------------------------------------------------------

#define SERVO_FREQ   50      // Hz — standard for analog servos
#define SERVO_STOP   307     // PCA9685 counts for 1500µs (stopped)
#define SERVO_MIN    150     // counts for ~750µs  (full speed one way)
#define SERVO_MAX    460     // counts for ~2250µs (full speed other way)

// ---------------------------------------------------------------------------
// PER-CHANNEL SERVO CONFIGURATION TABLE
//
//   changeover : emaVal at which this servo passes through SERVO_STOP (zero
//                speed). Below changeover → one direction; above → reverse.
//                Each pair reverses at a different distance as person approaches.
//                Calibrated range: ~75 (first detection) to ~6000 (1 foot away).
//
//   slope      : PWM counts per emaVal unit, centered on changeover.
//                Positive → CCW when far (emaVal<changeover), CW when close.
//                Negative → CW when far, CCW when close.
//                Sized so every channel reaches full speed at ~1 ft (emaVal≈6000).
// ---------------------------------------------------------------------------

#define NUM_CHANNELS 16

struct ServoConfig { float changeover; float slope; };

ServoConfig channels[NUM_CHANNELS] = {
  //  ch   changeover   slope
  //
  //  8 positive slopes + 8 negative slopes = ALWAYS half CW, half CCW at any distance.
  //  Changeovers spread 100→5200 so reversals cascade as person walks in from 3m:
  //    Far (~3m, abs≈100-300): most channels in "far" direction, ch0/1 just reversed.
  //    Mid (~3ft, abs≈2100):   roughly half have reversed, complex mix.
  //    Close (~1ft, abs≈6000): all reversed — +slopes CW, -slopes CCW, all near full speed.
  //  Formula uses abs(emaVal) so far-range negative sensor readings map correctly.
  //  Slopes sized so each channel reaches full speed at abs(emaVal)=6000 (~1 ft).
  //
  /*  0 */ {  100,  0.026f },   // reverses just above detection threshold
  /*  1 */ {  100, -0.026f },
  /*  2 */ {  500,  0.028f },
  /*  3 */ {  500, -0.028f },
  /*  4 */ { 1000,  0.031f },
  /*  5 */ { 1000, -0.031f },
  /*  6 */ { 1700,  0.036f },
  /*  7 */ { 1700, -0.036f },
  /*  8 */ { 2600,  0.045f },
  /*  9 */ { 2600, -0.045f },
  /* 10 */ { 3500,  0.061f },
  /* 11 */ { 3500, -0.061f },
  /* 12 */ { 4400,  0.096f },
  /* 13 */ { 4400, -0.096f },
  /* 14 */ { 5200,  0.191f },   // reverses only at ~1 ft
  /* 15 */ { 5200, -0.191f },
};

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------

STHS34PF80_I2C       mySensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

int16_t presenceVal       = 0;
float   emaVal            = 0.0f;
bool    emaReady          = false;
bool    sensorFound       = false;
int     sessionMaxBars    = 0;
int     servoPwm[NUM_CHANNELS];

bool          systemActive    = false;
unsigned long wakeTimeMs      = 0;   // when the most recent wake-up started

unsigned long lastPrintMs     = 0;

// ---------------------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------------------

void i2cScan()
{
    Serial.println("Scanning I2C bus (Wire1 / STEMMA QT)...");
    int devicesFound = 0;
    for (byte addr = 1; addr < 127; addr++)
    {
        Wire1.beginTransmission(addr);
        byte error = Wire1.endTransmission();
        if (error == 0)
        {
            Serial.print("  0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x5A) Serial.print("  <-- STHS34PF80");
            if (addr == 0x40) Serial.print("  <-- PCA9685");
            Serial.println();
            devicesFound++;
        }
    }
    Serial.print("  Total: ");
    Serial.println(devicesFound);
}

void stopAllServos()
{
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
        pwm.setPWM(ch, 0, SERVO_STOP);
}

void updateServos()
{
    unsigned long now     = millis();
    bool          present = (abs(emaVal) >= PRESENCE_CUTOFF);

    // Immediate shutdown when presence is lost; soft-on ramp when waking.
    // EMA smoothing (300ms) already debounces momentary dropouts.
    if (!systemActive && present)
    {
        systemActive = true;
        wakeTimeMs   = now;
    }
    else if (systemActive && !present)
    {
        systemActive = false;
    }

    // Soft-on ramp: 0.0 → 1.0 over SOFTON_MS after wake
    float speedScale = systemActive
        ? constrain((float)(now - wakeTimeMs) / SOFTON_MS, 0.0f, 1.0f)
        : 0.0f;

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        int pwmVal;
        if (speedScale == 0.0f)
        {
            pwmVal = SERVO_STOP;
        }
        else
        {
            // abs(emaVal) so far-range negative readings work the same as positive
            float target = channels[ch].slope * (abs(emaVal) - channels[ch].changeover);
            pwmVal = SERVO_STOP + (int)(speedScale * target);
            pwmVal = constrain(pwmVal, SERVO_MIN, SERVO_MAX);
        }
        pwm.setPWM(ch, 0, pwmVal);
        servoPwm[ch] = pwmVal;
    }
}

// ANSI display — use an ANSI-capable terminal, NOT Arduino Serial Monitor.
// On macOS:  screen /dev/cu.usbmodem<XXXX> 115200
// On Windows: PuTTY or CoolTerm with 115200 baud
void printDisplay()
{
    const int HALF = 20;   // chars on each side of the center line

    // Clear screen and jump to top-left
    Serial.print("\033[2J\033[H");

    // --- Servo bars ---
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        // Normalize pwmVal to -HALF..+HALF
        int p = servoPwm[ch];
        int bars;
        if (p >= SERVO_STOP)
            bars = (int)((float)(p - SERVO_STOP) / (SERVO_MAX - SERVO_STOP) * HALF);
        else
            bars = (int)((float)(p - SERVO_STOP) / (SERVO_STOP - SERVO_MIN) * HALF);
        bars = constrain(bars, -HALF, HALF);

        Serial.print("ch");
        if (ch < 10) Serial.print(" ");
        Serial.print(ch);
        Serial.print("  ");

        // Left side: positions -HALF..-1, '#' if position >= bars (fills toward center)
        for (int i = -HALF; i < 0; i++)
            Serial.print(i >= bars ? '#' : ' ');

        Serial.print('|');

        // Right side: positions 1..HALF, '#' if position <= bars
        for (int i = 1; i <= HALF; i++)
            Serial.print(i <= bars ? '#' : ' ');

        Serial.println();
    }

    // --- Distance bar ---
    Serial.println();
    float signal   = abs(emaVal);
    float logRatio = log(max(1.0f, signal)) / log((float)MAX_SIGNAL);
    int   cur      = constrain((int)(MAX_BARS * (1.0f - logRatio)), 0, MAX_BARS);
    if (cur > sessionMaxBars) sessionMaxBars = cur;

    if (sensorFound)
    {
        Serial.print("dist  ");
        for (int i = 0; i < cur;              i++) Serial.print('#');
        for (int i = cur; i < sessionMaxBars; i++) Serial.print('-');
        if (sessionMaxBars > cur)                  Serial.print('|');
        Serial.print("  raw:");
        Serial.print(presenceVal);
        Serial.println();

        Serial.println();
        if (!systemActive)
        {
            Serial.println("  [OFF — no presence]");
        }
        else
        {
            unsigned long elapsed = millis() - wakeTimeMs;
            if (elapsed < SOFTON_MS)
            {
                int pct = (int)(elapsed * 100 / SOFTON_MS);
                Serial.print("  [RAMP ");
                Serial.print(pct);
                Serial.println("%]");
            }
            else
            {
                Serial.println("  [ACTIVE]");
            }
        }
    }
    else
    {
        Serial.println("  [SWEEP TEST — no sensor connected]");
    }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("----------------------------------------");
    Serial.println("gears.ino v5.3");
    Serial.println("----------------------------------------");

    Wire1.begin();
    i2cScan();

    // -- Distance sensor (optional — skipped if not connected) --
    Serial.println("[INIT] Starting STHS34PF80...");
    if (mySensor.begin(STHS34PF80_I2C_ADDRESS, Wire1) == false)
    {
        Serial.println("[WARN] STHS34PF80 not found — running PCA9685 sweep test.");
        sensorFound = false;
    }
    else
    {
        delay(1000);
        mySensor.setTmosODR(STHS34PF80_TMOS_ODR_AT_30Hz);
        Serial.println("[INIT] STHS34PF80 ready, ODR=30Hz.");
        sensorFound = true;
    }

    // -- Servo driver --
    Serial.println("[INIT] Starting PCA9685...");
    if (!pwm.begin())
    {
        Serial.println("[ERROR] PCA9685 not found — check wiring. Halting.");
        while (1);
    }
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    stopAllServos();
    Serial.println("[INIT] PCA9685 ready, all servos stopped.");

    Serial.println("----------------------------------------");
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------

// No-sensor test: slowly ramps all servos CW then CCW then stops, proving
// the PCA9685 is receiving commands. One full cycle every ~6 seconds.
void runSweepTest()
{
    unsigned long t   = millis();
    unsigned long pos = t % 6000;   // 6-second cycle

    int pwmVal;
    if      (pos < 1000) pwmVal = SERVO_STOP;                              // 0–1s:  stopped
    else if (pos < 2500) pwmVal = map(pos, 1000, 2500, SERVO_STOP, SERVO_MAX); // 1–2.5s: ramp CW
    else if (pos < 3000) pwmVal = SERVO_MAX;                               // 2.5–3s: full CW
    else if (pos < 4500) pwmVal = map(pos, 3000, 4500, SERVO_MAX, SERVO_MIN); // 3–4.5s: ramp CCW
    else if (pos < 5000) pwmVal = SERVO_MIN;                               // 4.5–5s: full CCW
    else                 pwmVal = map(pos, 5000, 6000, SERVO_MIN, SERVO_STOP); // 5–6s:  return to stop

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        pwm.setPWM(ch, 0, pwmVal);
        servoPwm[ch] = pwmVal;
    }

    if (millis() - lastPrintMs >= 500)
    {
        lastPrintMs = millis();
        printDisplay();
    }
}

void loop()
{
    if (!sensorFound)
    {
        runSweepTest();
        return;
    }

    sths34pf80_tmos_drdy_status_t dataReady;
    mySensor.getDataReady(&dataReady);

    if (dataReady.drdy == 1)
    {
        mySensor.getPresenceValue(&presenceVal);

        if (!emaReady) { emaVal = (float)presenceVal; emaReady = true; }
        else           { emaVal = EMA_ALPHA * (float)presenceVal + (1.0f - EMA_ALPHA) * emaVal; }

        updateServos();

        if (millis() - lastPrintMs >= 500)
        {
            lastPrintMs = millis();
            printDisplay();
        }
    }
}
