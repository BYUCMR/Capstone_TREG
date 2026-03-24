#include <SPI.h>
#include <RF24.h>

// -------------------- RF config --------------------
RF24 radio(7, 8); // CE, CSN

const byte nodeAddresses[][6] = {"01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12"};
const int NUMBER_OF_NODES = 12;

// -------------------- Motion config --------------------
const int TICKS_PER_INCH = 1125; // if you need it elsewhere
const int MAX_VELOCITY = 1800;   // ticks per second (abs clamp)

const float VEL_DURATION = 1.50f; // seconds for VEL command
const float POS_DURATION = 5.00f; // seconds for POS command (often unused in pos mode)

// -------------------- Payloads --------------------
struct RadioPayload
{
  bool isVelocityControl;
  float velocity;   // counts per second (signed)
  long position;    // target position (counts)
  float duration_s; // duration for velocity control (seconds)
  bool reset;       // reset position to zero
};

struct AcknowledgePayload
{
  long currentPosition;
  float currentTPS;
};

RadioPayload txPayload;
AcknowledgePayload ack;

long lastKnownPositions[NUMBER_OF_NODES] = {0};

// -------------------- Helpers --------------------
static inline float clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}

bool sendToNode(uint8_t nodeIndex, const RadioPayload &p, uint8_t maxRetries = 3)
{
  // Validate node index
  if (nodeIndex >= NUMBER_OF_NODES)
  {
    Serial.print(F("Invalid node index: "));
    Serial.println(nodeIndex);
    return false;
  }

  radio.openWritingPipe(nodeAddresses[nodeIndex]);

  // Retry loop
  for (uint8_t attempt = 0; attempt < maxRetries; attempt++)
  {
    // send RadioPayload over radio
    bool ok = radio.write(&p, sizeof(p));

    if (ok) // if it succeeds
    {
      // Check for ACK payload with timeout
      unsigned long ackStart = millis();
      while (!radio.isAckPayloadAvailable()) // try to get ack for timed period
      {
        if (millis() - ackStart > 100) // 100ms timeout
        {
          break;
        }
      }

      if (radio.isAckPayloadAvailable()) // Radio has confirmed it received message
      {
        radio.read(&ack, sizeof(ack));                       // read payload acknowledgment
        lastKnownPositions[nodeIndex] = ack.currentPosition; // save positions from ack

        return true; // Success!
      }
      else // if it fails, Radio has not confirmed it received message, but was sent successfully
      {
        // Transmission succeeded but no ACK payload
        // This might still be acceptable depending on your setup
        return true;
      }
    }

    // Failed - wait before retry
    if (attempt < maxRetries - 1)
    {
      delayMicroseconds(500); // Short delay between retries
    }
  }

  // All retries failed
  Serial.print(F("FAILED to send to node "));
  Serial.print(nodeIndex + 1);
  Serial.print(F(" after "));
  Serial.print(maxRetries);
  Serial.println(F(" attempts"));

  return false;
}

void parseNodePositions(const String &csv, long out[NUMBER_OF_NODES])
{
  for (int i = 0; i < NUMBER_OF_NODES; ++i)
    out[i] = 0;
  int start = 0;
  for (int idx = 0; idx < NUMBER_OF_NODES; ++idx)
  {
    int comma = csv.indexOf(',', start);
    String token = (comma == -1) ? csv.substring(start) : csv.substring(start, comma);
    token.trim();
    if (token.length() > 0)
      out[idx] = token.toInt();
    if (comma == -1)
      break;
    start = comma + 1;
  }
}

void parseNodeVelocities(const String &csv, float out[NUMBER_OF_NODES])
{
  for (int i = 0; i < NUMBER_OF_NODES; ++i)
    out[i] = 0.0f;
  int start = 0;
  for (int idx = 0; idx < NUMBER_OF_NODES; ++idx)
  {
    int comma = csv.indexOf(',', start);
    String token = (comma == -1) ? csv.substring(start) : csv.substring(start, comma);
    token.trim();
    if (token.length() > 0)
      out[idx] = token.toFloat();
    if (comma == -1)
      break;
    start = comma + 1;
  }
}

void printPositionsArray()
{
  Serial.print('[');
  for (int i = 0; i < NUMBER_OF_NODES; i++)
  {
    Serial.print(lastKnownPositions[i]);
    if (i < NUMBER_OF_NODES - 1)
      Serial.print(',');
  }
  Serial.println(']');
  Serial.flush();
}

float readFloat()
{
  float num;
  if (Serial.available() >= 4)
  {

    for (int j = 0; j < 4; j++)
    {
      inSer[j] = Serial.read();
    }
    memcpy(&num, inSer, 4);
    return num
  }
  return 0.0f
}

int readInt()
{
  int num;
  if (Serial.available() >= 4)
  {

    for (int j = 0; j < 4; j++)
    {
      inSer[j] = Serial.read();
    }
    memcpy(&num, inSer, 4);
    return num
  }
  return 0
}

// -------------------- Setup & Loop --------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.stopListening();

  Serial.println(F("Transmitter ready. Commands: RESET | STOP | VEL:v1,v2,... | POS:p1,p2,..."));
}

void loop()
{
  {
    if (!Serial.available())
    {
      return;
    }
    int type = readInt();
    switch (type)
    {
    default:
      Serial.println(F("Unrecognized command."));

      break;

    case 0: // STOP
      for (int i = 0; i < NUMBER_OF_NODES; ++i)
      {
        RadioPayload p = {};
        p.isVelocityControl = true;
        p.velocity = 0.0f;
        p.position = 0;
        p.duration_s = 0.0f; // explicit immediate stop
        p.reset = false;
        sendToNode(i, p);
      }
      break;

    case 1: // RESET
      for (int i = 0; i < NUMBER_OF_NODES; ++i)
      {
        RadioPayload p = {};
        p.isVelocityControl = true; // doesn't matter for reset; set true by convention
        p.velocity = 0.0f;
        p.position = 0;
        p.duration_s = 0.0f;
        p.reset = true;
        sendToNode(i, p);
      }
      break;
    case 2: // VEL
      // Read 4 bytes from Serial buffer
      for (int i = 0; i < NUMBER_OF_NODES; i++)
      {
        float vel = readFloat();
        memcpy(&vel, inSer, 4);
        RadioPayload p = {};
        p.isVelocityControl = true;
        p.velocity = clampf(vel, -MAX_VELOCITY, MAX_VELOCITY);
        p.position = 0;              // ignored by receiver in velocity mode
        p.duration_s = VEL_DURATION; // from your requirement
        p.reset = false;
        sendToNode(i, p);
      }
      // Copy the byte array memory into the float variable memory
      break;
    case 3: // VEL_DUR
      float time = readFloat();
      for (int i = 0; i < NUMBER_OF_NODES; i++)
      {
        float vel = readFloat();
        memcpy(&vel, inSer, 4);
        RadioPayload p = {};
        p.isVelocityControl = true;
        p.velocity = clampf(vel, -MAX_VELOCITY, MAX_VELOCITY);
        p.position = 0; // ignored by receiver in velocity mode
        p.duration_s = time;
        sendToNode(i, p);
      }
      break;
    case 4://POS
      for (int i = 0; i < NUMBER_OF_NODES; i++)
      {
        int pos = readInt();
        memcpy(&pos, inSer, 4);
        RadioPayload p = {};
        p.isVelocityControl = false; // position mode
        p.velocity = 0.0f;           // not used in pos mode
        p.position = pos;            // counts
        p.duration_s = POS_DURATION; // provided per your spec
        p.reset = false;
        sendToNode(i, p);
      }
      break;
    }
  }

  // Print positions array back to the computer
  printPositionsArray();
}
