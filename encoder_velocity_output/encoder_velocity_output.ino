/*
 * Quadrature Encoder -> Velocity -> 0-5V Analog Output
 * Board: Arduino Uno / Nano
 *
 * Encoder wiring (unchanged):
 *   A -> D2, B -> D3, plus power/GND
 *
 * Analog output:
 *   D9 -> [4.7k resistor] -> OUT node -> [1uF-10uF cap] -> GND
 *   Take your 0-5V signal from the OUT node (resistor/cap junction).
 *
 *   D9 runs PWM at ~31 kHz (Timer1, prescaler = 1), so a single RC
 *   stage filters it to clean DC. Larger cap = smoother but slower:
 *     4.7k + 1uF  -> ~34 Hz cutoff, fast response, tiny ripple
 *     4.7k + 10uF -> ~3.4 Hz cutoff, very smooth, ~50ms settling
 *
 *   NOTE: this output is high-impedance. It's fine feeding an ADC,
 *   PLC analog input, or DMM, but buffer it with a rail-to-rail
 *   op-amp follower (e.g. MCP6002) if it must drive any real load.
 *
 * Velocity mapping (signed -> 0-5V):
 *   -MAX_VELOCITY -> 0.0V
 *    0            -> 2.5V
 *   +MAX_VELOCITY -> 5.0V
 */

const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;
const uint8_t PIN_OUT = 9;          // Timer1 PWM pin (D9 or D10 only)

const int32_t COUNTS_PER_REV = 4096;

// Full-scale velocity in counts/sec. 7000 counts/s ~= 102 RPM at 4x
// decoding of a 1024 PPR encoder. Set this to your real max so you
// use the full output range.
const float MAX_VELOCITY = 8000.0;

// How often velocity is computed and the output updated (ms)
const uint16_t SAMPLE_INTERVAL_MS = 1;

// Exponential smoothing factor, 0-1. Higher = snappier, lower =
// smoother. 1.0 disables smoothing.
const float SMOOTHING_ALPHA = 1.0;

volatile int32_t encoderCount = 0;
volatile uint8_t prevState = 0;

const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

void encoderISR() {
  uint8_t state = (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);
  encoderCount += QDEC_TABLE[(prevState << 2) | state];
  prevState = state;
}

int32_t readCountAtomic() {
  noInterrupts();
  int32_t c = encoderCount;
  interrupts();
  return c;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PIN_OUT, OUTPUT);

  prevState = (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);
  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);

  // Raise Timer1 PWM (pins 9/10) to ~31.4 kHz so the RC filter can be
  // small and ripple negligible. Timer1 doesn't affect millis()/delay(),
  // which run on Timer0.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  analogWrite(PIN_OUT, 127);  // start at ~2.5V (zero velocity)
}

void loop() {
  static uint32_t lastSample = 0;
  static int32_t lastCount = 0;
  static float velocitySmoothed = 0.0;

  uint32_t now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;

  // Use the actual elapsed time, not the nominal interval, for accuracy
  float dt = (now - lastSample) / 1000.0;
  lastSample = now;

  int32_t count = readCountAtomic();
  float velocity = (count - lastCount) / dt;   // counts/sec, signed
  lastCount = count;

  velocitySmoothed += SMOOTHING_ALPHA * (velocity - velocitySmoothed);

  // flip sign (so that clockwise is positive)
  velocitySmoothed = -velocitySmoothed;

  // Map [-MAX_VELOCITY, +MAX_VELOCITY] -> [0, 255], centered at 127.5
  float norm = velocitySmoothed / MAX_VELOCITY;     // -1..+1
  norm = constrain(norm, -1.0, 1.0);
  uint8_t pwm = (uint8_t)((norm + 1.0) * 127.5);

  analogWrite(PIN_OUT, pwm);

  // Serial.print("Vel: ");
  // Serial.print(velocitySmoothed, 1);
  // Serial.print(" counts/s  RPM: ");
  // Serial.print(velocitySmoothed * 60.0 / COUNTS_PER_REV, 2);
  // Serial.print("  PWM: ");
  // Serial.print(pwm);
  // Serial.print("  Vout: ");
  // Serial.print(pwm * 5.0 / 255.0, 3);
  // Serial.println(" V");
}
