/******************************************************************************
  gears.ino

  Version: 2.8
  Last Modified: 2026-05-31

  Changelog:
    v2.8 (2026-05-31) - Widen to 100-bar / 120-col. Lower CLOSE_SIGNAL to 50
                        so far-distance low-signal values spread across more
                        of the bar instead of saturating near max.
    v2.7 (2026-05-31) - Invert bar: far = more #, close = fewer #. Scale
                        to 60 chars so 3m fits on 80-col line with raw value.
    v2.6 (2026-05-31) - Drop mm conversion entirely. Bar shows raw signal
                        strength (# per 25 raw units). More # = closer/stronger.
                        | marks session max signal. Goal: see if sensor detects
                        at 3m.
    v2.5 (2026-05-31) - Cap distMm at MAX_RANGE_MM (3m) so bad calibration
                        can't make a 40-segment bar. Add | at high water mark.
                        Restore raw presenceVal to help calibrate SCALE.
    v2.4 (2026-05-31) - Replace numeric output with ASCII bar graph.
                        # = current distance (one per 25cm), - = gap to
                        high water mark. Redraws at 2Hz. mm values appended.
    v2.3 (2026-05-31) - Remove hand/no-hand gating entirely. Always show
                        distance estimate. EMA runs continuously. Max recap
                        every 3 seconds.
    v2.2 (2026-05-31) - Replace \r approach (unreliable in Serial Monitor)
                        with throttled println: max 5Hz updates while moving,
                        session max recap every 3 seconds, clean lost line.
    v2.0 (2026-05-31) - Lower DETECT_THRESHOLD to 5. When below threshold,
                        print raw presenceVal once/sec for range calibration.
                        Print raw alongside distance when tracking.
    v1.8 (2026-05-31) - Apply EMA to raw presenceVal (linear space) before
                        converting to mm, eliminating noise amplification
                        through the reciprocal at large distances. Added
                        DETECT_THRESHOLD for hand-present gating; resets
                        cleanly when hand leaves. 1mm print threshold.
    v1.7 (2026-05-31) - Replace rolling average with EMA (alpha=0.15) for
                        smooth continuous tracking. Print only when smoothed
                        value shifts >25mm. Removed pres_flag gate so reading
                        is continuous regardless of internal threshold.
    v1.6 (2026-05-31) - Convert presence to mm via inverse scale factor
                        (PRESENCE_SCALE_MM / presenceVal). Rolling 8-sample
                        average at 30Hz (~267ms smoothing). Print only when
                        averaged distance changes more than DEADZONE_MM (50mm).
                        Removed Motion! print.
    v1.5 (2026-05-31) - Set ODR to max 30Hz for fastest motion tracking.
                        Loop now only prints presence when value changes;
                        removed heartbeat and per-frame status spam.
    v1.4 (2026-05-31) - Fix begin() call: library signature is
                        begin(uint8_t devAddr, TwoWire&), so must pass
                        STHS34PF80_I2C_ADDRESS before Wire1.
    v1.3 (2026-05-31) - Target board is Adafruit QT Py RP2040. STEMMA QT
                        connector uses Wire1 (not Wire), so switched all I2C
                        calls to Wire1. Also kept while(!Serial) guard for
                        native USB.
    v1.2 (2026-05-31) - Added while(!Serial) guard after Serial.begin() so
                        output isn't lost on native USB boards before the
                        serial connection is established.
    v1.1 (2026-05-31) - Added verbose debug output: I2C bus scan at startup,
                        per-step [INIT] logging, [HEARTBEAT] every 5s,
                        [STATUS] flag dump on every data-ready event, and
                        tagged [PRESENCE] / [MOTION] / [TEMP] readings.
    v1.0 (2023-09-19) - Initial release (SparkFun example, basic readings).

  Read human presence detection values from the STHS34PF80 sensor, print them
  to terminal. Prints raw IR presence (cm^-1), if motion was detected, and
  temperature in degrees C.

  Based on SparkFun STHS34PF80 Arduino Library example by
  Madison Chodikov @ SparkFun Electronics
  https://github.com/sparkfun/SparkFun_STHS34PF80_Arduino_Library

  Development environment specifics:

  IDE: Arduino 2.2.1
  Hardware Platform: SparkFun RedBoard Qwiic
  SparkFun Human Presence and Motion Sensor - STHS34PF80 (Qwiic) Version: 1.0
  SparkFun Qwiic Mini Human Presence and Motion Sensor - STHS34PF80 Version: 1.0

  Hardware Connections:
  Use a Qwiic cable to connect from the RedBoard Qwiic to the STHS34PF80 breakout (QWIIC).
  You can also choose to wire up the connections using the header pins like so:

  ARDUINO --> STHS34PF80
  SDA (A4) --> SDA
  SCL (A5) --> SCL
  3.3V --> 3.3V
  GND --> GND

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include "SparkFun_STHS34PF80_Arduino_Library.h"
#include <Wire.h>

STHS34PF80_I2C mySensor;

// EMA smoothing on raw signal. 0.25 at 30Hz ≈ 100ms.
#define EMA_ALPHA    0.25f

// presenceVal that maps to 0 bars (closest detectable range).
// Raise if bar never empties up close; lower if it saturates before 3m.
#define CLOSE_SIGNAL 50

// Total bar width. Leaves ~20 chars for "  raw:XXXXX" on a 120-col line.
#define MAX_BARS     100

int16_t presenceVal    = 0;
float   emaVal         = 0.0f;
bool    emaReady       = false;
int     sessionMaxBars = 0;
unsigned long lastPrintMs = 0;

// Scan I2C bus and print all found device addresses
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
            Serial.print("  I2C device found at address 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x5A) Serial.print("  <-- STHS34PF80 (expected)");
            Serial.println();
            devicesFound++;
        }
        else if (error == 4)
        {
            Serial.print("  Unknown error at address 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (devicesFound == 0)
        Serial.println("  No I2C devices found! Check wiring.");
    else
        Serial.print("  Total devices found: ");
        Serial.println(devicesFound);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { ; }  // wait for USB serial port to connect (needed on native USB boards)
    Serial.println("----------------------------------------");
    Serial.println("STHS34PF80 Example 1: Basic Readings");
    Serial.println("----------------------------------------");

    Serial.println("[INIT] Starting I2C on Wire1 (STEMMA QT connector)...");
    Wire1.begin();
    Serial.println("[INIT] Wire1 started.");

    i2cScan();

    Serial.println("[INIT] Attempting mySensor.begin(Wire1)...");
    if(mySensor.begin(STHS34PF80_I2C_ADDRESS, Wire1) == false)
    {
      Serial.println("[ERROR] mySensor.begin() failed!");
      Serial.println("[ERROR] Check: is sensor powered? Is SDA/SCL connected?");
      Serial.println("[ERROR] Expected sensor I2C address: 0x5A");
      Serial.println("[ERROR] Halting.");
      while(1);
    }

    Serial.println("[INIT] Sensor connected successfully!");
    Serial.println("[INIT] Waiting 1 second for sensor to stabilize...");
    delay(1000);
    mySensor.setTmosODR(STHS34PF80_TMOS_ODR_AT_30Hz);
    Serial.println("[INIT] ODR set to 30Hz (max).");
    Serial.println("[INIT] Setup complete. Entering loop.");
    Serial.println("----------------------------------------");
}

void loop()
{
  sths34pf80_tmos_drdy_status_t dataReady;
  mySensor.getDataReady(&dataReady);

  if(dataReady.drdy == 1)
  {
    mySensor.getPresenceValue(&presenceVal);

    if(!emaReady) { emaVal = (float)presenceVal; emaReady = true; }
    else          { emaVal = EMA_ALPHA * (float)presenceVal + (1.0f - EMA_ALPHA) * emaVal; }

    if(!emaReady) { emaVal = (float)presenceVal; emaReady = true; }
    else          { emaVal = EMA_ALPHA * (float)presenceVal + (1.0f - EMA_ALPHA) * emaVal; }

    if(millis() - lastPrintMs >= 500)
    {
      lastPrintMs = millis();

      // Invert: far away (low signal) = many #, close (high signal) = few #
      int cur = MAX_BARS - constrain((int)(emaVal * MAX_BARS / CLOSE_SIGNAL), 0, MAX_BARS);
      if(cur > sessionMaxBars) sessionMaxBars = cur;

      // # = current distance, - = gap to furthest point reached, | = high water mark
      for(int i = 0; i < cur;              i++) Serial.print('#');
      for(int i = cur; i < sessionMaxBars; i++) Serial.print('-');
      if(sessionMaxBars > cur)                  Serial.print('|');

      Serial.print("  raw:");
      Serial.println(presenceVal);
    }
  }
}

