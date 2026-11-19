/* ESP32 MASTER: ADXL345 -> pitch (float) + fast flag (byte) -> two Feelix motors
 *
 * ADXL345 on plain Wire (GPIO25 + GPIO26, order auto-detected).
 * Link.h claims Wire1 for the motors on GPIO21/22.
 *
 * Sent to each motor:
 *   float  pitch in degrees   -> callback index 0
 *   byte   1 = moving fast    -> callback index 2   (held for FAST_HOLD_MS once detected)
 * Acknowledgements
 * These sketches are based on the example sketches of the feelix Minidriver effect lib made by Anke van Oosterhout
 * Claude Sonnet 5.5 (Anthropic) was used in the development of this code for the purpose of rapid bug fixing. Functionallety and requirements where created, tested, and adjusted by Jasper Scheffer.
 */

/* ---------- libraries ----------
 * Link.h: the I2C master library from your example (it uses Wire1 internally).
 * Wire.h: the standard I2C library, used here for the ADXL345 only. */
#include "src/Link.h"
#include <Wire.h>

/* false = the library does not print every message it sends (keeps Serial readable) */
#define DEBUG           false

/* ---------- motor bus (handled by Link.h) ---------- */
#define ESP32_SCL       22
#define ESP32_SDA       21
#define SLAVE_0_ADDRESS 0x70   // motor 1
#define SLAVE_1_ADDRESS 0x71   // motor 2

/* callback indexes: must match the position in the callbackArr on the motor sketches */
#define CB_PITCH        0      // slave receiveFloat
#define CB_FAST         2      // slave receiveByte

/* ---------- ADXL345 wiring ----------
 * The two pins it is wired to. adxlConnect() tries both orders, so it does not
 * matter which one is SDA and which is SCL. */
#define ADXL_PIN_A      25
#define ADXL_PIN_B      26
#define ADXL_ADDR       0x53   // SDO to GND = 0x53, SDO to 3V3 = 0x1D

/* ---------- tuning values ---------- */
const float    TILT_SIGN      = 1.5;    // multiplies the pitch angle: negative flips the direction,
                                        // above 1 exaggerates it (the old comment said it was only a sign)
const float    FAST_THRESHOLD = 0.9;    // "motion" above this = moving fast
const uint16_t FAST_HOLD_MS   = 4000;   // once detected, the fast flag stays on this long
const float    PITCH_STEP     = 2.0;    // only send the pitch when it changed by this many degrees

/* list of motor addresses, padded to MAX_NR_OF_SLAVES (6) because the library
 * constructor reads that many entries; unused entries stay 0 */
uint8_t slaves[MAX_NR_OF_SLAVES] = { SLAVE_0_ADDRESS, SLAVE_1_ADDRESS };
MasterDevice I2C_Master(slaves);

/* ---------- ADXL345 ---------- */

/* write one value into one register of the sensor */
void adxlWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/* checks that the chip answers, then switches it on. Returns false if it is not found */
bool adxlInit() {
    Wire.beginTransmission(ADXL_ADDR);
    if (Wire.endTransmission() != 0) return false;   // no answer = not connected
    adxlWrite(0x2D, 0x08);   // POWER_CTL: start measuring
    adxlWrite(0x31, 0x08);   // DATA_FORMAT: full resolution, +-2 g
    adxlWrite(0x2C, 0x0A);   // BW_RATE: 100 Hz
    return true;
}

/* starts the sensor bus on pins A/B, and on B/A if that fails; returns true when found */
bool adxlConnect() {
    for (int attempt = 0; attempt < 2; attempt++) {
        int sda = (attempt == 0) ? ADXL_PIN_A : ADXL_PIN_B;
        int scl = (attempt == 0) ? ADXL_PIN_B : ADXL_PIN_A;
        Wire.end();                    // reset the bus before trying the other pin order
        Wire.begin(sda, scl);
        Wire.setClock(100000);         // 100 kHz, the same speed as the motor bus
        if (adxlInit()) {
            Serial.printf("ADXL345 ready (SDA=GPIO%d, SCL=GPIO%d)\n", sda, scl);
            return true;
        }
    }
    return false;
}

/* reads acceleration in g for the three axes. Returns false if the read fails */
bool adxlRead(float &x, float &y, float &z) {
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(0x32);                                              // first data register (X low byte)
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)6) != 6) return false;   // 2 bytes per axis
    int16_t rx = Wire.read() | (Wire.read() << 8);                // low byte first, then high byte
    int16_t ry = Wire.read() | (Wire.read() << 8);
    int16_t rz = Wire.read() | (Wire.read() << 8);
    x = rx * 0.0039; y = ry * 0.0039; z = rz * 0.0039;            // 3.9 mg per step = g
    return true;
}

/* ---------- sensor values ---------- */
bool  adxlReady = false;                          // true once the sensor has been found
float fx = 0, fy = 0, fz = 1;                     // smoothed acceleration (used for the tilt)
float px = 0, py = 0, pz = 1;                     // previous raw sample (used for the motion)
float motion = 0.0, pitch = 0.0;                  // motion = how much it is moving, pitch = tilt in degrees
unsigned long fastUntil = 0;                      // the fast flag stays on until this time (ms)

/* reads the sensor and updates motion, pitch and the fast flag */
void updateSensor() {
    float x, y, z;
    if (!adxlRead(x, y, z)) return;               // skip this round if the read failed

    /* motion: summed change since the previous sample, lightly smoothed.
     * Still is near 0, handled is a bit, shaken is a lot. */
    motion = 0.4 * motion + 0.6 * (fabs(x - px) + fabs(y - py) + fabs(z - pz));
    px = x; py = y; pz = z;

    /* smoothed acceleration gives a stable tilt angle */
    fx = 0.6 * fx + 0.4 * x;
    fy = 0.6 * fy + 0.4 * y;
    fz = 0.6 * fz + 0.4 * z;
    pitch = TILT_SIGN * atan2(fx, sqrt(fy * fy + fz * fz)) * 180.0 / PI;   // radians to degrees

    /* start the fast flag when the motion is big enough. It is only started when
     * pitch < 30, so fast movement while the device is tilted far over is ignored. */
    unsigned long now = millis();
    if (now >= fastUntil && motion > FAST_THRESHOLD && pitch < 30) fastUntil = now + FAST_HOLD_MS;
}

/* ---------- main ---------- */
/* timers for the periodic tasks (ms) */
unsigned long prevSensor = 0, prevSend = 0, prevPrint = 0, prevRetry = 0;

void setup() {
    delay(1000);
    Serial.begin(115200);

    /* sensor bus first, then the motor library */
    adxlReady = adxlConnect();
    if (!adxlReady) Serial.println("ADXL345 NOT found - check wiring");

    I2C_Master.init(ESP32_SDA, ESP32_SCL, CLOCK_SPEED::SLOW_MODE, DEBUG);
}

void loop() {
    unsigned long now = millis();

    /* sensor: read every 20 ms; if it was not found, retry the connection once per second */
    if (adxlReady) {
        if (now - prevSensor >= 20) { prevSensor = now; updateSensor(); }
    } else if (now - prevRetry >= 1000) {
        prevRetry = now;
        adxlReady = adxlConnect();
    }

    /* sending: check every 100 ms, but only transmit on a change (plus a refresh),
     * so the motors are interrupted as little as possible */
    if (now - prevSend >= 100) {
        prevSend = now;

        static float lastPitch = 999.0;              // last values that were sent
        static int8_t lastFast = -1;
        static unsigned long lastPitchTime = 0, lastFastTime = 0;

        /* pitch: send when it changed by PITCH_STEP, or every 1 s as a refresh */
        if (fabs(pitch - lastPitch) >= PITCH_STEP || now - lastPitchTime >= 1000) {
            lastPitch = pitch;
            lastPitchTime = now;
            I2C_Master.sendFloat(pitch, SLAVE_0_ADDRESS, CB_PITCH);
            delay(5);                                // let motor 1 finish before talking to motor 2
            I2C_Master.sendFloat(pitch, SLAVE_1_ADDRESS, CB_PITCH);
            delay(5);
        }

        /* fast flag: send when it switches on or off, or every 4 s as a refresh */
        int8_t fast = (now < fastUntil) ? 1 : 0;
        if (fast != lastFast || now - lastFastTime >= 4000) {
            lastFast = fast;
            lastFastTime = now;
            I2C_Master.sendByte(fast, SLAVE_0_ADDRESS, CB_FAST);
            delay(5);
            I2C_Master.sendByte(fast, SLAVE_1_ADDRESS, CB_FAST);
        }
    }

    /* status line every 200 ms: use it to tune FAST_THRESHOLD and TILT_SIGN */
    if (now - prevPrint >= 200) {
        prevPrint = now;
        Serial.printf("pitch: %6.1f  motion: %.3f  fast: %d\n", pitch, motion, now < fastUntil);
    }
}
