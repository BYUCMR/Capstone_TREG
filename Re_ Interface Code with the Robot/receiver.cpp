#include <SPI.h>
#include <RF24.h>

// --- Pins / constants ---
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define DIRECTION_PIN 10
#define DRIVE_PIN 5
#define LED_PIN_GREEN 18
#define LED_PIN_RED 19
#define SLEEP_PIN 4

#define RF_CE 7
#define RF_CSN 8

RF24 radio(RF_CE, RF_CSN);
const byte address[6] = "14";

// Use reciever new for 3 and 2

volatile long encoderPosition = 0;

long targetPos = 0;

bool isVelocityControl = true; // true: velocity control; false: position control

float targetTPS = 0.0f; // signed target counts/sec
float currentTPS = 0.0f;

unsigned long lastTime = 0;
long lastEncoderPosition = 0;

unsigned long moveStartTime = 0;
unsigned long moveEndTime = 0;
unsigned long now = 0;
bool moveActive = false;

// ---- Radio payloads ----
struct RadioPayload
{
  bool isVelocityControl;
  float velocity;   // counts per second (signed)
  long position;    // target position (counts)
  float duration_s; // duration for velocity control (seconds)
  bool reset;       // reset position to zero
};

struct AckPayload
{
  long currentPosition;
  float currentTPS;
};

RadioPayload payload;
AckPayload ack;

// --- Encoder ISR ---
void updateEncoder()
{
  if (digitalRead(ENCODER_PIN_B) == LOW)
    encoderPosition++;
  else
    encoderPosition--;
}

// --- Stop motor ---
void stopMotor()
{
  analogWrite(DRIVE_PIN, 0);
  moveActive = false;
  Serial.print("Stopped at pos: ");
  Serial.println(encoderPosition);
}

// --- State ---
float integralError = 0.0f;
float tps_f = 0.0f, lastTPS = 0.0f;
float dTPS_f = 0.0f;
float targetTPS_goal = 0.0f;    // what commands request
const float TPS_SLEW = 4000.0f; // counts/s^2 (tune)

void updateSpeedControl()
{

  const unsigned long sample_ms = 20;
  const uint8_t PWM_MIN = 15; // motor deadzone
  const float Kaw = 0.10f;    // anti-windup back-calculation

  // --- Gains ---
  float Kp = 0.750f;  // start a bit lower; re-tune
  float Ki = 0.002f;  // keep small
  float Kd = 0.0002f; // derivative on measurement (after filters)
  float Kff = 0.15f;

  float tau_v = 0.05f; // velocity LPF time constant [s] (30–70 ms)
  float tau_d = 0.15f; // derivative LPF time constant [s] (100–200 ms)

  unsigned long now = millis();
  unsigned long elapsed = now - lastTime;
  if (elapsed < sample_ms)
    return;

  // if (elapsed > 10 * sample_ms) {
  //   // Missed too many cycles, reset filter and integrator
  //   tps_f = currentTPS;
  //   lastTPS = currentTPS;
  //   lastEncoderPosition = encoderPosition;
  //   lastTime = now;
  //   return;
  // }

  float dt = elapsed / 1000.0f;
  long currentPos = encoderPosition;
  long dx = currentPos - lastEncoderPosition;

  // Velocity estimate (raw)
  float tps_raw = (dx / dt);

  // Low-pass filter the TPS
  float alpha_v = dt / (tau_v + dt);
  tps_f += alpha_v * (tps_raw - tps_f);

  // Derivative of (filtered) TPS, then low-pass it, too
  float dTPS_raw = (tps_f - lastTPS) / dt;
  float alpha_d = dt / (tau_d + dt);
  dTPS_f += alpha_d * (dTPS_raw - dTPS_f);

  // Error on filtered measurement
  float error = targetTPS - tps_f;

  // PID terms
  float up = Kp * error;
  float ud = -Kd * dTPS_f; // derivative on measurement
  float ui = integralError + Ki * error * dt;

  const float I_MAX = 20.0f; // anti-windup clamping
  ui = constrain(ui, -I_MAX, I_MAX);

  // Feedforward (open-loop compensation)
  float uff = Kff * targetTPS;

  float u_offset = 20.0f * (targetTPS / 1800.0f);

  // Combine (unsaturated)
  float u_unsat = u_offset + up + ud + ui + uff;

  // Saturate and anti-windup back-calculation
  float u = constrain(u_unsat, -255.0f, 255.0f);
  integralError = ui + Kaw * (u - u_unsat); // bleed when clamped

  // Deadzone lift for small nonzero commands
  if (fabs(u) > 0.0f && fabs(u) < PWM_MIN)
  {
    u = copysign((float)PWM_MIN, u);
  }

  // Zero-target hygiene: bleed integrator when stopping
  if (fabs(targetTPS) < 1e-3f)
  {
    integralError *= 0.5f; // or set to 0.0f
    if (fabs(tps_f) < 1.0f)
      u = 0.0f; // fully stop when really near zero
  }

  // Drive
  bool forward = (u >= 0.0f);
  digitalWrite(DIRECTION_PIN, forward ? LOW : HIGH);
  uint8_t pwm = (uint8_t)constrain(fabs(u), 0.0f, 255.0f);
  analogWrite(DRIVE_PIN, pwm);

  // State update
  lastTPS = tps_f;
  lastEncoderPosition = currentPos;
  lastTime = now;

  // // Debug
  // Serial.print("dt: ");  Serial.print(dt, 3);
  // Serial.print("  err: ");  Serial.print(error);
  // Serial.print("  TPS: "); Serial.print(tps_f);
  // Serial.print("  dTPS: "); Serial.print(dTPS_f);
  // Serial.print("  tgt: "); Serial.print(targetTPS);
  // Serial.print("  uP: ");  Serial.print(up);
  // Serial.print("  uD: ");  Serial.print(ud);
  // Serial.print("  uI: ");  Serial.print(integralError);
  // Serial.print("  uFF: "); Serial.print(uff);
  // Serial.print("  u: ");   Serial.print(u);
  // Serial.print("  pwm: "); Serial.println(pwm);
}

long lastPos = 0;

void updatePositionControl()
{

  // Serial.println("Position control active");
  // --- Gains ---
  float Kp_pos = 0.5f; // position proportional gain
  float Ki_pos = 0.0f; // position integral gain
  float Kd_pos = 0.0f; // position derivative gain

  const unsigned long sample_ms = 20;
  const uint8_t PWM_MIN = 15; // motor deadzone

  unsigned long now = millis();
  unsigned long elapsed = now - lastTime;
  if (elapsed < sample_ms)
    return;

  long currentPos = encoderPosition;
  long posError = targetPos - currentPos;

  if (abs(posError) < 5)
  {
    // Near enough
    stopMotor();
    return;
  }

  float dt = elapsed / 1000.0f;

  long dPos = currentPos - lastPos;
  float velocity = dPos / dt; // counts per second

  integralError += posError * dt;
  const float I_MAX_POS = 1000.0f; // anti-wind
  integralError = constrain(integralError, -I_MAX_POS, I_MAX_POS);

  float up = Kp_pos * posError;
  float ui = Ki_pos * integralError;
  float ud = -Kd_pos * velocity; // derivative on measurement
  float u = up + ui + ud;
  u = constrain(u, -255.0f, 255.0f);
  // Deadzone lift for small nonzero commands
  if (fabs(u) > 0.0f && fabs(u) < PWM_MIN)
  {
    u = copysign((float)PWM_MIN, u);
  }
  // Drive
  bool forward = (u >= 0.0f);
  digitalWrite(DIRECTION_PIN, forward ? LOW : HIGH);
  uint8_t pwm = (uint8_t)constrain(fabs(u), 0.0f, 255.0f);
  analogWrite(DRIVE_PIN, pwm);
  // State update
  lastPos = currentPos;
  lastTime = now;
}

void receiveRadioCommand()
{
  // Serial command
  if (Serial.available())
  {
    isVelocityControl = true;
    String input = Serial.readStringUntil('\n');
    input.trim();
    int comma = input.indexOf(',');
    if (comma > 0)
    {
      float v = input.substring(0, comma).toFloat();
      float dur_s = input.substring(comma + 1).toFloat();
      if (dur_s <= 0)
      {
        stopMotor();
        return;
      }

      targetTPS = v;

      moveStartTime = millis();
      moveEndTime = moveStartTime + (unsigned long)(dur_s * 1000.0f);
      moveActive = true;

      Serial.print("Serial cmd → v: ");
      Serial.print(v);
      Serial.print(" cps, t: ");
      Serial.print(dur_s);
      Serial.println(" s");
    }
  }

  // RF24 command
  if (radio.available())
  {

    // Prepare ACK
    ack.currentPosition = encoderPosition;
    ack.currentTPS = currentTPS;
    // unsigned long now = millis();
    // Sends acknowledgement to transmitter comprising current position and speed of roller:
    radio.writeAckPayload(0, &ack, sizeof(ack));

    RadioPayload cmd;
    // Read command from radio into cmd object:
    radio.read(&cmd, sizeof(cmd));

    if (cmd.reset)
    {
      encoderPosition = 0;
      lastEncoderPosition = 0;
      Serial.println("Position reset to zero.");
    }

    if (cmd.isVelocityControl)
    {
      isVelocityControl = true;
      if (cmd.duration_s > 0.0f)
      {
        noInterrupts();
        targetTPS = cmd.velocity; // signed
        interrupts();

        // // --- Reset State ---
        // integralError = 0.0f;
        // tps_f = 0.0f, lastTPS = 0.0f;
        // dTPS_f = 0.0f;

        moveStartTime = millis();
        moveEndTime = moveStartTime + (unsigned long)(cmd.duration_s * 1000.0f);
        moveActive = true;

        Serial.print("RF cmd → v: ");
        Serial.print(cmd.velocity);
        Serial.print(" tps, t: ");
        Serial.print(cmd.duration_s);
        Serial.println(" s");
      }
      else
      {
        stopMotor();
      }
    }
    else
    {
      isVelocityControl = false;
      noInterrupts();
      targetPos = cmd.position;
      interrupts();

      moveEndTime = millis() + cmd.duration_s * 1000;

      // --- Reset State ---
      integralError = 0.0f;
      lastPos = encoderPosition;
      lastTime = millis();
      moveActive = true;

      Serial.print("RF cmd → pos: ");
      Serial.print(cmd.position);
    }
  }
}

void setup()
{
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_RED, OUTPUT);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, RISING);
  digitalWrite(SLEEP_PIN, HIGH);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.openReadingPipe(0, address);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.startListening();
  radio.flush_rx();
  radio.flush_tx();

  digitalWrite(DIRECTION_PIN, LOW);
}

void loop()
{
  receiveRadioCommand();

  if (moveActive)
  {
    if (isVelocityControl)
      updateSpeedControl();
    else
      updatePositionControl();

    if (millis() >= moveEndTime)
    {
      stopMotor();
    }
  }
}
