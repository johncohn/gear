/******************************************************************************
  gears.ino

  Version: 5.9
  Last Modified: 2026-10-14

  Overview:
    An interactive kinetic sculpture controller. An STHS34PF80 IR presence
    sensor detects a person's distance (up to ~3m). That signal drives 16
    continuous-rotation servos via a PCA9685 PWM driver — each servo has an
    individually configured direction and speed ramp so the whole assembly
    responds differently to the same distance reading. When nobody is present
    all servos are stopped; they spin up as a person approaches, with
    alternating directions and layered speed responses across the 16 channels.
    The sculpture only runs while the sensor sees MOVEMENT: if the signal stays
    steady for IDLE_TIMEOUT_MS it eases to a stop and stays still and quiet.

  Hardware:
    Adafruit QT Py RP2040
    STEMMA QT / Qwiic chain on Wire1 (I2C):
      0x5A  SparkFun STHS34PF80 human presence & motion sensor
      0x40  Adafruit PCA9685 16-channel 12-bit PWM servo driver
    16x continuous-rotation servos on PCA9685 channels 0–15

  Required Libraries (install via Arduino Library Manager):
    SparkFun STHS34PF80 Arduino Library
    Adafruit PWM Servo Driver Library
    Adafruit NeoPixel

  Status LED (QT Py on-board NeoPixel) — readable from across the room:
    dim blue   STILL, room quiet
    yellow     STILL, but motion is getting near the wake threshold
    green      ACTIVE (servos running)
    purple     STOPPING (easing down to still)

  Unattended test log:
    While STILL, the max motion level is recorded for each minute (last 30
    minutes kept) along with the time and motion level of every wake-up.
    Both are shown at the bottom of the terminal display, so you can leave
    the room, come back, connect, and read what happened while you were away.
    Lives in RAM: it survives plugging/unplugging USB while on battery, but
    is cleared if the board loses all power or is reset.

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
    Distance is turned into an "approach" position from 0.0 (edge of
    detection, ~3m) to 1.0 (~1 ft), on a log scale so the walk in is spread
    evenly (raw signal grows roughly exponentially as you get closer).
    changeover : approach position (0..1) where this servo stops and reverses.
    dir        : +1 → CW when close, CCW when far.  -1 → the opposite.
    top        : this servo's own top speed (0..1 of MAX_SPEED), varied per
                 channel so the gears never all run at the same speed.

  Changelog:
    v5.9 (2026-10-14) - More speed variation as someone approaches. Old
                        mapping gave every channel the same speed at ~1 ft and
                        crowded most of the change into the last metre; small
                        commands also sat in the servos' dead band. Now: log-
                        scale approach position (0..1), changeovers spread
                        evenly 0.10..0.80, per-channel top speed 0.35..1.0,
                        SERVO_DEADBAND offset so small speeds visibly move,
                        MAX_SPEED 0.25→0.5 (fastest pair; most run slower).
                        Display shows approach %.
    v5.8 (2026-10-14) - Self-calibrating wake threshold: learns the empty-room
                        motion noise while STILL (slow-rising, slow-decaying
                        peak tracker) and wakes at NOISE_MARGIN x that noise,
                        clamped to MOTION_WAKE_MIN..MOTION_WAKE_MAX. Keep-alive
                        is half the wake threshold. No on-site tuning needed.
    v5.7 (2026-10-14) - On-board NeoPixel shows STILL/ACTIVE/STOPPING state.
                        Unattended log: per-minute max motion while STILL
                        (30 min) and recent wake-up events, shown in display.
    v5.6 (2026-10-14) - Still when nobody is moving. Previously any steady
                        signal above PRESENCE_CUTOFF (background thermal drift,
                        or the negative undershoot after someone leaves, made
                        positive by abs()) kept servos turning. Now waking
                        requires movement: motion = |emaVal - emaSlow| (fast vs
                        ~3s average). Wake at MOTION_WAKE, stay awake while
                        motion >= MOTION_KEEPALIVE, go idle after
                        IDLE_TIMEOUT_MS without it. Smooth activity envelope
                        (SOFTON_MS up, SOFTOFF_MS down) then PWM full-off.
                        Speed/direction mapping while active is unchanged.
                        Display shows motion level and idle countdown.
    v5.5 (2026-10-14) - MAX_SPEED 0.5→0.25 (quarter of full range). setup()
                        waits at most 3s for USB serial so the board runs on
                        battery/charger power. Distance-bar high-water mark now
                        resets each time someone is newly detected. Display
                        redraws in place (cursor home + clear-to-EOL) instead
                        of clearing the whole screen, to stop flicker.
    v5.4 (2026-06-05) - Fix idle spinning: PRESENCE_CUTOFF 75→150 (clear of
                        background noise); signalFade multiplier (0→1 over
                        FADE_RANGE=400 above cutoff) tapers speeds at detection
                        edge — provides soft entry and natural soft exit as person
                        walks away. speedScale = wakeRamp × signalFade × MAX_SPEED.
                        MAX_SPEED=0.5 halves all servo speeds globally.
                        PCA9685 full-off (setPWM 4096) when speed=0 removes PWM
                        signal entirely so no calibration drift can spin servos.
                        Display shows combined speed% while active.
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
#include <Adafruit_NeoPixel.h>
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

// abs(emaVal) below this = nobody home; must be clearly above background noise.
#define PRESENCE_CUTOFF    150

// abs(emaVal) at ~1 ft: approach position 1.0 (calibrated: 1ft≈6000).
#define CLOSE_SIGNAL       6000

// Approach distance (0..1) over which speedScale fades in from the edge of
// detection. Soft entry from far range, soft exit as the person walks away.
#define FADE_X             0.12f

// Time-based soft-on ramp when waking (ms). Guards against sudden start
// if person is already close when system first detects them.
#define SOFTON_MS          1500

// --- Movement gate: the sculpture only runs while something is moving. ---
// motion = |emaVal - emaSlow|, where emaSlow is a ~3s average of the signal.
// A steady signal (empty room, slow thermal drift, someone who has left)
// gives motion ≈ 0.
//
// The wake threshold calibrates itself: while STILL, noiseLevel follows the
// empty-room motion peaks (rises slowly so a person's 1-2s approach barely
// moves it, decays slowly so a quiet room lowers it again). Wake threshold =
// NOISE_MARGIN x noiseLevel, kept within MOTION_WAKE_MIN..MOTION_WAKE_MAX.
// Keep-alive threshold (to stay awake once running) = half the wake threshold.
#define EMA_SLOW_ALPHA     0.01f   // ~3.3s at 30Hz
#define NOISE_INITIAL      40.0f   // starting noise guess → wake at 120 from boot
#define NOISE_MARGIN       3.0f    // wake when motion is this many x the noise
#define NOISE_RISE         0.002f  // per-sample pull toward a higher peak (~17s)
#define NOISE_DECAY        0.99992f // per-sample decay (halves in ~5 min)
#define MOTION_WAKE_MIN    60      // never wake on less than this
#define MOTION_WAKE_MAX    400     // never require more than this to wake
#define IDLE_TIMEOUT_MS    6000    // no motion this long → ease to a stop
#define SOFTOFF_MS         1000    // ramp-down time when going idle

// Global speed ceiling: 1.0 = full range (SERVO_MIN..SERVO_MAX),
// 0.5 = half. Only channels with top=1.0 reach it; the rest run slower.
#define MAX_SPEED          0.5f

// Below this fraction of full speed a servo is held stopped (no creeping).
#define MIN_SPEED          0.02f

// ---------------------------------------------------------------------------
// SERVO SETTINGS  (apply to all channels)
// ---------------------------------------------------------------------------

#define SERVO_FREQ   50      // Hz — standard for analog servos
#define SERVO_STOP   307     // PCA9685 counts for 1500µs (stopped)
#define SERVO_MIN    150     // counts for ~750µs  (full speed one way)
#define SERVO_MAX    460     // counts for ~2250µs (full speed other way)
#define SERVO_DEADBAND 6     // counts (~30µs) either side of stop where
                             // continuous servos barely turn; skipped over so
                             // the slowest commanded speeds still visibly move

// ---------------------------------------------------------------------------
// PER-CHANNEL SERVO CONFIGURATION TABLE
//
//   changeover : approach position (0 = edge of detection ~3m, 1 = ~1 ft)
//                where this servo passes through stop and reverses.
//   dir        : +1 → CCW when far (before changeover), CW when close.
//                -1 → CW when far, CCW when close.
//   top        : this channel's top speed as a fraction of MAX_SPEED.
//
//   Speed grows linearly with distance from the changeover, reaching `top`
//   at whichever end (0 or 1) is farther from it.
// ---------------------------------------------------------------------------

#define NUM_CHANNELS 16

struct ServoConfig { float changeover; int8_t dir; float top; };

ServoConfig channels[NUM_CHANNELS] = {
  //  8 dir=+1 and 8 dir=-1 → always a CW/CCW mix at any distance.
  //  Changeovers spread evenly 0.10→0.80 so one pair reverses every ~10% of
  //  the walk in. Pair partners get different top speeds so no two gears
  //  match, and fast/slow alternate down the table.
  //
  //  ch          changeover  dir   top
  /*  0 */ {  0.10f,  +1,  1.00f },   // reverses just after first detection
  /*  1 */ {  0.10f,  -1,  0.45f },
  /*  2 */ {  0.20f,  +1,  0.60f },
  /*  3 */ {  0.20f,  -1,  0.90f },
  /*  4 */ {  0.30f,  +1,  0.80f },
  /*  5 */ {  0.30f,  -1,  0.40f },
  /*  6 */ {  0.40f,  +1,  0.50f },
  /*  7 */ {  0.40f,  -1,  1.00f },
  /*  8 */ {  0.50f,  +1,  0.95f },   // reverses about halfway in
  /*  9 */ {  0.50f,  -1,  0.55f },
  /* 10 */ {  0.60f,  +1,  0.45f },
  /* 11 */ {  0.60f,  -1,  0.85f },
  /* 12 */ {  0.70f,  +1,  0.70f },
  /* 13 */ {  0.70f,  -1,  0.35f },
  /* 14 */ {  0.80f,  +1,  0.40f },   // reverses only when quite close
  /* 15 */ {  0.80f,  -1,  0.75f },
};

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------

STHS34PF80_I2C       mySensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

int16_t presenceVal       = 0;
float   emaVal            = 0.0f;
float   emaSlow           = 0.0f;   // slow (~3s) average, for motion detection
float   motionLevel       = 0.0f;   // |emaVal - emaSlow|
bool    emaReady          = false;
bool    sensorFound       = false;
int     sessionMaxBars    = 0;
int     servoPwm[NUM_CHANNELS];

bool          systemActive     = false;
unsigned long lastMotionMs     = 0;   // last time motion >= keepalive threshold
float         noiseLevel       = NOISE_INITIAL;  // learned empty-room motion peak
float         wakeThreshold    = NOISE_MARGIN * NOISE_INITIAL;
unsigned long lastUpdateMs     = 0;
float         activity         = 0.0f; // 0 = still, 1 = fully awake (ramped)
float         currentSpeedScale = 0.0f;
float         approachPos      = 0.0f; // 0 = edge of detection, 1 = ~1 ft

// --- Unattended test log (RAM) ---
#define LOG_MINUTES  30
#define LOG_WAKES    8
int           stillMaxLog[LOG_MINUTES];   // per-minute max motion while STILL; -1 = no still time
int           stillLogCount   = 0;        // entries filled (≤ LOG_MINUTES)
int           stillLogHead    = 0;        // next slot to write
float         minuteStillMax  = -1.0f;    // running max for the current minute
unsigned long minuteStartMs   = 0;
unsigned long wakeTimes[LOG_WAKES];
int           wakeMotion[LOG_WAKES];
int           wakeCount       = 0;        // total wakes since boot

Adafruit_NeoPixel statusLed(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

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
        pwm.setPWM(ch, 0, 4096);   // full-off: remove PWM signal
}

// Records the empty-room motion log and sets the status LED colour.
void logAndShowStatus(unsigned long now)
{
    bool still = (!systemActive && activity == 0.0f);

    if (still && motionLevel > minuteStillMax) minuteStillMax = motionLevel;
    if (now - minuteStartMs >= 60000UL)
    {
        stillMaxLog[stillLogHead] = (int)minuteStillMax;   // -1 if never still
        stillLogHead = (stillLogHead + 1) % LOG_MINUTES;
        if (stillLogCount < LOG_MINUTES) stillLogCount++;
        minuteStillMax = -1.0f;
        minuteStartMs  = now;
    }

    uint32_t color;
    if (systemActive)                          color = statusLed.Color(0, 40, 0);   // green
    else if (!still)                           color = statusLed.Color(25, 0, 30);  // purple
    else if (motionLevel >= wakeThreshold / 2) color = statusLed.Color(40, 25, 0);  // yellow
    else                                       color = statusLed.Color(0, 0, 8);    // dim blue
    statusLed.setPixelColor(0, color);
    statusLed.show();
}

void printDuration(unsigned long ms)
{
    unsigned long sec = ms / 1000;
    Serial.print(sec / 3600);  Serial.print('h');
    if ((sec / 60) % 60 < 10) Serial.print('0');
    Serial.print((sec / 60) % 60); Serial.print('m');
    if (sec % 60 < 10) Serial.print('0');
    Serial.print(sec % 60); Serial.print('s');
}

void printLog()
{
    Serial.println("\033[K");
    Serial.print("  Up ");
    printDuration(millis());
    Serial.println("\033[K");

    Serial.print("  Max motion per minute while STILL (newest first, -- = not still):\033[K\n  ");
    if (stillLogCount == 0) Serial.print("(first minute not finished yet)");
    for (int i = 0; i < stillLogCount; i++)
    {
        int v = stillMaxLog[(stillLogHead - 1 - i + LOG_MINUTES) % LOG_MINUTES];
        if (v < 0) Serial.print("--");
        else       Serial.print(v);
        Serial.print(i % 15 == 14 ? "\033[K\n  " : " ");
    }
    Serial.println("\033[K");

    Serial.print("  Wake-ups: ");
    Serial.print(wakeCount);
    Serial.println(" total. Most recent (time ago, motion at wake):\033[K");
    int shown = min(wakeCount, LOG_WAKES);
    for (int i = 0; i < shown; i++)
    {
        int idx = (wakeCount - 1 - i) % LOG_WAKES;
        Serial.print("    ");
        printDuration(millis() - wakeTimes[idx]);
        Serial.print(" ago   motion ");
        Serial.print(wakeMotion[idx]);
        Serial.println("\033[K");
    }
}

void updateServos()
{
    unsigned long now    = millis();
    float         signal = abs(emaVal);
    unsigned long dt     = now - lastUpdateMs;
    lastUpdateMs = now;

    // Learn the empty-room noise while fully still.
    if (!systemActive && activity == 0.0f)
    {
        if (motionLevel > noiseLevel) noiseLevel += NOISE_RISE * (motionLevel - noiseLevel);
        else                          noiseLevel *= NOISE_DECAY;
        wakeThreshold = constrain(NOISE_MARGIN * noiseLevel,
                                  (float)MOTION_WAKE_MIN, (float)MOTION_WAKE_MAX);
    }

    // Movement gate: wake on clear motion, stay awake while motion continues,
    // go idle once nothing has moved for IDLE_TIMEOUT_MS.
    if (!systemActive)
    {
        if (motionLevel >= wakeThreshold)
        {
            systemActive   = true;
            lastMotionMs   = now;
            sessionMaxBars = 0;   // new visitor: reset distance high-water mark
            wakeTimes[wakeCount % LOG_WAKES]  = now;
            wakeMotion[wakeCount % LOG_WAKES] = (int)motionLevel;
            wakeCount++;
        }
    }
    else
    {
        if (motionLevel >= wakeThreshold / 2) lastMotionMs = now;
        if (now - lastMotionMs >= IDLE_TIMEOUT_MS) systemActive = false;
    }

    // activity: 0→1 over SOFTON_MS when awake (no sudden lurch if the person
    // is already close), 1→0 over SOFTOFF_MS when going idle (gentle stop).
    if (systemActive) activity += (float)dt / SOFTON_MS;
    else              activity -= (float)dt / SOFTOFF_MS;
    activity = constrain(activity, 0.0f, 1.0f);

    logAndShowStatus(now);

    // approachPos: log-scale 0→1 from PRESENCE_CUTOFF (~3m) to CLOSE_SIGNAL
    // (~1 ft), so each step of the walk in changes the gears about equally.
    approachPos = (signal > PRESENCE_CUTOFF)
        ? constrain(log(signal / PRESENCE_CUTOFF) /
                    log((float)CLOSE_SIGNAL / PRESENCE_CUTOFF), 0.0f, 1.0f)
        : 0.0f;

    // signalFade: 0→1 over the first FADE_X of the approach. Soft entry at
    // far range and a natural soft exit as the person walks away.
    float signalFade = constrain(approachPos / FADE_X, 0.0f, 1.0f);

    currentSpeedScale = activity * signalFade * MAX_SPEED;

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        if (currentSpeedScale == 0.0f)
        {
            // Full-off: remove PWM signal entirely so no calibration drift can spin the servo
            pwm.setPWM(ch, 0, 4096);
            servoPwm[ch] = SERVO_STOP;
            continue;
        }

        // Signed speed -1..1: zero at changeover, ±top at the far end.
        const ServoConfig &c = channels[ch];
        float span  = max(c.changeover, 1.0f - c.changeover);
        float speed = c.dir * c.top * constrain((approachPos - c.changeover) / span, -1.0f, 1.0f);
        float mag   = fabs(speed) * currentSpeedScale;   // 0..MAX_SPEED

        int pwmVal = SERVO_STOP;
        if (mag >= MIN_SPEED)
        {
            int offset = SERVO_DEADBAND + (int)(mag * (SERVO_MAX - SERVO_STOP - SERVO_DEADBAND));
            pwmVal = constrain(SERVO_STOP + (speed > 0 ? offset : -offset), SERVO_MIN, SERVO_MAX);
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

    // Jump to top-left and overwrite in place (no full clear = no flicker).
    // Each line ends with \033[K (clear to end of line); \033[J at the end
    // clears anything left below from a previous, longer frame.
    Serial.print("\033[H");

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

        Serial.println("\033[K");
    }

    // --- Distance bar ---
    Serial.println("\033[K");
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
        Serial.print("  approach:");
        Serial.print((int)(approachPos * 100));
        Serial.print('%');
        Serial.println("\033[K");

        Serial.println("\033[K");
        if (!systemActive && activity == 0.0f)
        {
            Serial.print("  [STILL  motion:");
            Serial.print((int)motionLevel);
            Serial.print(" / wake at ");
            Serial.print((int)wakeThreshold);
            Serial.print("  noise:");
            Serial.print((int)noiseLevel);
            Serial.println("]\033[K");
        }
        else if (!systemActive)
        {
            Serial.print("  [STOPPING  speed:");
            Serial.print((int)(currentSpeedScale * 100));
            Serial.println("%]\033[K");
        }
        else
        {
            unsigned long sinceMotion = millis() - lastMotionMs;
            Serial.print("  [ACTIVE  speed:");
            Serial.print((int)(currentSpeedScale * 100));
            Serial.print("%  motion:");
            Serial.print((int)motionLevel);
            Serial.print("  idle in ");
            Serial.print((IDLE_TIMEOUT_MS - min(sinceMotion, (unsigned long)IDLE_TIMEOUT_MS)) / 1000);
            Serial.println("s]\033[K");
        }
    }
    else
    {
        Serial.println("  [SWEEP TEST — no sensor connected]\033[K");
    }
    if (sensorFound) printLog();
    Serial.print("\033[J");
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup()
{
    statusLed.begin();
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    statusLed.setPixelColor(0, statusLed.Color(0, 0, 8));
    statusLed.show();

    Serial.begin(115200);
    // Wait up to 3s for a USB serial host; continue anyway for battery/charger use.
    unsigned long serialWait = millis();
    while (!Serial && (millis() - serialWait < 3000)) { ; }
    Serial.println("----------------------------------------");
    Serial.println("gears.ino v5.9");
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

        if (!emaReady)
        {
            emaVal = emaSlow = (float)presenceVal;
            emaReady = true;
        }
        else
        {
            emaVal  = EMA_ALPHA      * (float)presenceVal + (1.0f - EMA_ALPHA)      * emaVal;
            emaSlow = EMA_SLOW_ALPHA * (float)presenceVal + (1.0f - EMA_SLOW_ALPHA) * emaSlow;
        }
        motionLevel = abs(emaVal - emaSlow);

        updateServos();

        if (millis() - lastPrintMs >= 500)
        {
            lastPrintMs = millis();
            printDisplay();
        }
    }
}
