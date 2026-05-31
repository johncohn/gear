/******************************************************************************
  gears.ino

  Version: 1.1
  Last Modified: 2026-05-31

  Changelog:
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

// Values to fill with presence and motion data
int16_t presenceVal = 0;
int16_t motionVal = 0;
float temperatureVal = 0;

unsigned long lastHeartbeat = 0;
unsigned long loopCount = 0;

// Scan I2C bus and print all found device addresses
void i2cScan()
{
    Serial.println("Scanning I2C bus...");
    int devicesFound = 0;
    for (byte addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
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
    Serial.println("----------------------------------------");
    Serial.println("STHS34PF80 Example 1: Basic Readings");
    Serial.println("----------------------------------------");

    Serial.println("[INIT] Starting I2C (Wire.begin)...");
    Wire.begin();
    Serial.println("[INIT] I2C started.");

    i2cScan();

    Serial.println("[INIT] Attempting mySensor.begin()...");
    if(mySensor.begin() == false)
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
    Serial.println("[INIT] Setup complete. Entering loop.");
    Serial.println("----------------------------------------");
}

void loop()
{
  loopCount++;

  // Print a heartbeat every 5 seconds so we know the loop is running
  if (millis() - lastHeartbeat >= 5000)
  {
    lastHeartbeat = millis();
    Serial.print("[HEARTBEAT] Loop running, iteration=");
    Serial.print(loopCount);
    Serial.print(", uptime=");
    Serial.print(millis() / 1000);
    Serial.println("s");
  }

  sths34pf80_tmos_drdy_status_t dataReady;
  mySensor.getDataReady(&dataReady);

  if(dataReady.drdy == 1)
  {
    Serial.println("[DATA] New data ready from sensor.");
    sths34pf80_tmos_func_status_t status;
    mySensor.getStatus(&status);

    Serial.print("[STATUS] pres_flag=");
    Serial.print(status.pres_flag);
    Serial.print("  mot_flag=");
    Serial.print(status.mot_flag);
    Serial.print("  tamb_shock_flag=");
    Serial.println(status.tamb_shock_flag);

    if(status.pres_flag == 1)
    {
      mySensor.getPresenceValue(&presenceVal);
      Serial.print("[PRESENCE] ");
      Serial.print(presenceVal);
      Serial.println(" cm^-1");
    }

    if(status.mot_flag == 1)
    {
      Serial.println("[MOTION] Motion Detected!");
    }

    if(status.tamb_shock_flag == 1)
    {
      mySensor.getTemperatureData(&temperatureVal);
      Serial.print("[TEMP] ");
      Serial.print(temperatureVal);
      Serial.println(" °C");
    }
  }
}

