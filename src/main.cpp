#include <Arduino.h>
#include <Servo.h>

// ================= CONFIG =================
// true  = move to base position and hold (for alignment)
// false = normal random eye movement
#define CALIBRATE false

// In calibration mode, also sweep slowly: base -> min -> max -> base, repeating.
// Lets you check the full range clears your support structure.
#define SHOW_LIMITS true
#define RELEASE_AT_POINTS true
#define LIMIT_PAUSE_MS 1500 // pause at each point during the sweep

#define TILTSERVOPIN 1
#define ROTATESERVOPIN 0

// Base (rest) positions, in degrees
#define TILT_BASE 150
#define ROTATE_BASE 90

// Movement ranges used in normal mode (and shown in SHOW_LIMITS)
#define TILT_MIN 130
#define TILT_MAX 220
#define ROTATE_MIN 50
#define ROTATE_MAX 150

// Speed: milliseconds per 1° step. Higher = slower.
#define TILT_STEP_MS 3
#define ROTATE_STEP_MS 5
// ==========================================

struct SlowServo
{
  Servo servo;
  uint8_t pin;
  int current;
  int target;
  unsigned long stepMs;
  unsigned long lastStep;

  void begin(uint8_t p, int start, unsigned long ms)
  {
    pin = p;
    current = target = start;
    stepMs = ms;
    lastStep = 0;
    servo.attach(pin);
    servo.write(current);
  }

  void moveTo(int t)
  {
    target = constrain(t, 40, 220);
    if (!servo.attached())
      servo.attach(pin);
  }

  bool moving() { return current != target; }

  void update()
  {
    if (!moving())
      return;
    unsigned long now = millis();
    if (now - lastStep < stepMs)
      return;
    lastStep = now;
    current += (target > current) ? 1 : -1;
    servo.write(current);
  }

  void releaseIfIdle()
  {
    if (!moving() && servo.attached())
      servo.detach();
  }
};

SlowServo Tilt, Rotate;
unsigned long lastDecision = 0;

// ---------- Calibration ----------
void resetToBase()
{
  Tilt.moveTo(TILT_BASE);
  Rotate.moveTo(ROTATE_BASE);
}

void calibrationLoop()
{
  Tilt.update();
  Rotate.update();

#if SHOW_LIMITS
  // Sweep through: base -> min -> max -> base, pausing at each
  static uint8_t stage = 0;
  static unsigned long arrivedAt = 0;

  if (Tilt.moving() || Rotate.moving())
  {
    arrivedAt = 0;
    return;
  }
  if (arrivedAt == 0)
  {
    arrivedAt = millis();
    #if RELEASE_AT_POINTS
      Tilt.releaseIfIdle();
      Rotate.releaseIfIdle();
    #endif
    return;
    }
  if (millis() - arrivedAt < LIMIT_PAUSE_MS)
    return;

  arrivedAt = 0;
  stage = (stage + 1) % 3;
  switch (stage)
  {
  case 0:
    Tilt.moveTo(TILT_BASE);
    Rotate.moveTo(ROTATE_BASE);
    break;
  case 1:
    Tilt.moveTo(TILT_MIN);
    Rotate.moveTo(ROTATE_MIN);
    break;
  case 2:
    Tilt.moveTo(TILT_MAX);
    Rotate.moveTo(ROTATE_MAX);
    break;
  }
#endif
  // Otherwise: just hold at base. Servos stay attached so they resist being moved.
}

// ---------- Normal eye movement ----------
void eyeLoop()
{
  Tilt.update();
  Rotate.update();

  if (millis() - lastDecision >= 100)
  {
    lastDecision = millis();

    Tilt.releaseIfIdle();
    Rotate.releaseIfIdle();

    if (!Tilt.moving() && random(100) > 80)
      Tilt.moveTo(random(TILT_MIN, TILT_MAX + 1));
    if (!Rotate.moving() && random(100) > 90)
      Rotate.moveTo(random(ROTATE_MIN, ROTATE_MAX + 1));
  }
}

// ---------- Main ----------
void setup()
{
  randomSeed(analogRead(A3));

  // Start at base and settle there
  Tilt.begin(TILTSERVOPIN, TILT_BASE, TILT_STEP_MS);
  Rotate.begin(ROTATESERVOPIN, ROTATE_BASE, ROTATE_STEP_MS);
  delay(1000);
}

void loop()
{
#if CALIBRATE
  calibrationLoop();
#else
  eyeLoop();
#endif
}