/******************************************************************************
  gears.ino

  Version: 4.2
  Last Modified: 2026-05-31

  Changelog:
    v4.2 (2026-05-31) - Set all changeovers to 0: servos stopped when nobody
                        present, spin up as person approaches. Varied slope
                        magnitudes for layered activation at different ranges.
    v4.1 (2026-05-31) - ANSI terminal display: 16 centered servo bars + distance
                        bar, redrawn in place at 2Hz. Requires ANSI terminal
                        (screen/PuTTY/CoolTerm), not Arduino Serial Monitor.
    v4.0 (2026-05-31) - Add PCA9685 16-channel PWM servo driver. Each channel
                        configured via ServoConfig table (changeover + slope).
                        All servos stop when presence signal drops below
                        PRESENCE_CUTOFF. Fixed duplicate EMA update bug.
    v3.3 (2026-05-31) - Lower EMA_ALPHA 0.25→0.1 for ~300ms smoothing to
                        suppress occasional noise spikes at far range.
    v3.2 (2026-05-31) - Remove autoscaling. Fixed MAX_SIGNAL=10000 based on
                        real calibration: 1ft≈6000, 3ft≈2100.
    v3.0 (2026-05-31) - Use abs(emaVal) so negative raw values (far range)
                        contribute to the log-scale bar correctly.
    v2.4 (2026-05-31) - Replace numeric output with ASCII bar graph.
    v1.0 (2023-09-19) - Initial release (SparkFun example, basic readings).

  Hardware:
    Adafruit QT Py RP2040
    STEMMA QT chain on Wire1:
      0x5A  STHS34PF80 human presence sensor
      0x40  PCA9685 16-channel PWM servo driver

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

// abs(emaVal) below this = nobody home → stop all servos.
// Raise to stop servos sooner (closer); lower to keep running at greater range.
#define PRESENCE_CUTOFF 30

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
//   changeover : emaVal at which this servo is stopped (crosses SERVO_STOP).
//                Higher value = servo activates when person is closer.
//                Lower value  = servo activates when person is farther away.
//                Calibrated range: ~100 (very far) to ~6000 (1 foot away).
//
//   slope      : PWM counts per unit of (emaVal - changeover).
//                Positive → spins forward when person is CLOSER than changeover.
//                Negative → spins forward when person is FARTHER than changeover.
//                Larger magnitude → faster speed ramp.
//                Typical range: ±0.03 to ±0.15
// ---------------------------------------------------------------------------

#define NUM_CHANNELS 16

struct ServoConfig { float changeover; float slope; };

ServoConfig channels[NUM_CHANNELS] = {
  //  ch   changeover   slope
  //  All changeovers = 0 so every servo starts stopped when nobody is
  //  present and spins up as emaVal rises (person gets closer).
  //  Alternating slope signs → alternating CW/CCW directions.
  //  Larger slope magnitude → reaches full speed at greater distance.
  /*  0 */ { 0,  0.015f },   // gentlest — full speed only when very close
  /*  1 */ { 0, -0.015f },
  /*  2 */ { 0,  0.025f },
  /*  3 */ { 0, -0.025f },
  /*  4 */ { 0,  0.035f },
  /*  5 */ { 0, -0.035f },
  /*  6 */ { 0,  0.050f },
  /*  7 */ { 0, -0.050f },
  /*  8 */ { 0,  0.070f },
  /*  9 */ { 0, -0.070f },
  /* 10 */ { 0,  0.090f },
  /* 11 */ { 0, -0.090f },
  /* 12 */ { 0,  0.120f },
  /* 13 */ { 0, -0.120f },
  /* 14 */ { 0,  0.150f },   // most aggressive — full speed at mid-range
  /* 15 */ { 0, -0.150f },
};

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------

STHS34PF80_I2C       mySensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

int16_t presenceVal       = 0;
float   emaVal            = 0.0f;
bool    emaReady          = false;
int     sessionMaxBars    = 0;
int     servoPwm[NUM_CHANNELS];   // last computed PWM per channel, for display
unsigned long lastPrintMs = 0;

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
    bool present = (abs(emaVal) >= PRESENCE_CUTOFF);

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        int pwmVal;
        if (!present)
        {
            pwmVal = SERVO_STOP;
        }
        else
        {
            pwmVal = SERVO_STOP + (int)(channels[ch].slope * (emaVal - channels[ch].changeover));
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

    Serial.print("dist  ");
    for (int i = 0; i < cur;              i++) Serial.print('#');
    for (int i = cur; i < sessionMaxBars; i++) Serial.print('-');
    if (sessionMaxBars > cur)                  Serial.print('|');
    Serial.print("  raw:");
    Serial.print(presenceVal);
    if (abs(emaVal) < PRESENCE_CUTOFF) Serial.print("  [no presence]");
    Serial.println();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("----------------------------------------");
    Serial.println("gears.ino v4.0");
    Serial.println("----------------------------------------");

    Wire1.begin();
    i2cScan();

    // -- Distance sensor --
    Serial.println("[INIT] Starting STHS34PF80...");
    if (mySensor.begin(STHS34PF80_I2C_ADDRESS, Wire1) == false)
    {
        Serial.println("[ERROR] STHS34PF80 not found — check wiring. Halting.");
        while (1);
    }
    delay(1000);
    mySensor.setTmosODR(STHS34PF80_TMOS_ODR_AT_30Hz);
    Serial.println("[INIT] STHS34PF80 ready, ODR=30Hz.");

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

void loop()
{
    sths34pf80_tmos_drdy_status_t dataReady;
    mySensor.getDataReady(&dataReady);

    if (dataReady.drdy == 1)
    {
        mySensor.getPresenceValue(&presenceVal);

        if (!emaReady) { emaVal = (float)presenceVal; emaReady = true; }
        else           { emaVal = EMA_ALPHA * (float)presenceVal + (1.0f - EMA_ALPHA) * emaVal; }

        updateServos();

        // Redraw display at 2Hz
        if (millis() - lastPrintMs >= 500)
        {
            lastPrintMs = millis();
            printDisplay();
        }
    }
}
