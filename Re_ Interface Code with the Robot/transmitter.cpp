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

    if (ok)// if it succeeds
    {
      // Check for ACK payload with timeout
      unsigned long ackStart = millis();
      while (!radio.isAckPayloadAvailable())// try to get ack for timed period
      {
        if (millis() - ackStart > 100) // 100ms timeout
        {
          break;
        }
      }

      if (radio.isAckPayloadAvailable()) // Radio has confirmed it received message
      {
        radio.read(&ack, sizeof(ack)); // read payload acknowledgment
        lastKnownPositions[nodeIndex] = ack.currentPosition; //save positions from ack

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
  if (!Serial.available())
    return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0)
    return;

  String canon = input;
  canon.replace(" ", "");

  // -------- RESET --------
  if (canon.equalsIgnoreCase("RESET"))
  {
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
  }
  // -------- STOP --------
  else if (canon.equalsIgnoreCase("STOP"))
  {
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
  }
  // -------- VEL:... --------
  else if (canon.startsWith("VEL:") || canon.startsWith("vel:") || canon.startsWith("Vel:"))
  {
    String args = input.substring(input.indexOf(':') + 1); // keep original to allow signs, spaces
    float vels[NUMBER_OF_NODES];
    parseNodeVelocities(args, vels);

    for (int i = 0; i < NUMBER_OF_NODES; ++i)
    {
      RadioPayload p = {};
      p.isVelocityControl = true;
      p.velocity = clampf(vels[i], -MAX_VELOCITY, MAX_VELOCITY);
      p.position = 0;              // ignored by receiver in velocity mode
      p.duration_s = VEL_DURATION; // from your requirement
      p.reset = false;
      sendToNode(i, p);
    }
  }
  else if (canon.startsWith("VEL_DUR:") || canon.startsWith("vel_dur:") || canon.startsWith("Vel_Dur:"))
  {
    unsigned long t0 = millis(); // Start timing

    String args = input.substring(input.indexOf(':') + 1); // get everything after "VEL_DUR:"
    // Find the last colon which separates velocities from duration
    int lastColon = args.lastIndexOf(':');
    if (lastColon == -1)
    {
      Serial.println(F("Invalid VEL_DUR format. Use: VEL_DUR:v1,v2,v3,v4,v5,v6:duration"));
      return;
    }

    // Split into velocity CSV and duration
    String velCSV = args.substring(0, lastColon);
    String durStr = args.substring(lastColon + 1);
    durStr.trim();

    float vels[NUMBER_OF_NODES];
    parseNodeVelocities(velCSV, vels);

    float duration = durStr.toFloat();
    if (duration < 0.0f)
      duration = 0.0f; // safety check

    for (int i = 0; i < NUMBER_OF_NODES; ++i)
    {
      RadioPayload p = {};
      p.isVelocityControl = true;
      p.velocity = clampf(vels[i], -MAX_VELOCITY, MAX_VELOCITY);
      p.position = 0;          // ignored by receiver in velocity mode
      p.duration_s = duration; // use the provided duration
      p.reset = false;
      sendToNode(i, p);
    }

    unsigned long dt = millis() - t0; // Calculate elapsed time
    Serial.print(F("VEL_DUR command completed in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
  }
  // -------- POS:... --------
  else if (canon.startsWith("POS:") || canon.startsWith("pos:") || canon.startsWith("Pos:"))
  {
    String args = input.substring(input.indexOf(':') + 1);
    long positions[NUMBER_OF_NODES];
    parseNodePositions(args, positions);

    for (int i = 0; i < NUMBER_OF_NODES; ++i)
    {
      RadioPayload p = {};
      p.isVelocityControl = false; // position mode
      p.velocity = 0.0f;           // not used in pos mode
      p.position = positions[i];   // counts
      p.duration_s = POS_DURATION; // provided per your spec
      p.reset = false;
      sendToNode(i, p);
    }
  }
  // -------- Unknown --------
  else
  {
    Serial.println(F("Unrecognized command. Use: RESET | STOP | VEL:v1,v2,... | POS:p1,p2,..."));
  }

  // Print positions array back to the computer
  printPositionsArray();
}
