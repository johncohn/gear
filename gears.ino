/******************************************************************************
  gears.ino

  Version: 1.6
  Last Modified: 2026-05-31

  Changelog:
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

// Calibration: distance_mm ≈ PRESENCE_SCALE / presenceVal (inverse relationship).
// Hold an object at a known distance and adjust until readings match.
#define PRESENCE_SCALE_MM  50000.0f

// Rolling average window (samples at 30Hz, so 8 = ~267ms of smoothing)
#define AVG_WINDOW  8

// Only print when the averaged distance shifts by more than this
#define DEADZONE_MM  50.0f

int16_t presenceVal = 0;
float   avgBuffer[AVG_WINDOW] = {0};
int     avgIndex  = 0;
int     avgCount  = 0;
float   lastPrintedAvg = 0.0f;

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
    sths34pf80_tmos_func_status_t status;
    mySensor.getStatus(&status);

    if(status.pres_flag == 1)
    {
      mySensor.getPresenceValue(&presenceVal);

      if(presenceVal > 0)
      {
        float distMm = PRESENCE_SCALE_MM / (float)presenceVal;

        avgBuffer[avgIndex] = distMm;
        avgIndex = (avgIndex + 1) % AVG_WINDOW;
        if(avgCount < AVG_WINDOW) avgCount++;

        float sum = 0;
        for(int i = 0; i < avgCount; i++) sum += avgBuffer[i];
        float avg = sum / (float)avgCount;

        if(fabs(avg - lastPrintedAvg) > DEADZONE_MM)
        {
          Serial.print("Distance: ");
          Serial.print(avg, 1);
          Serial.println(" mm");
          lastPrintedAvg = avg;
        }
      }
    }
  }
}

