# IOT-DEVICE-SMART-BAT
#define BLYNK_TEMPLATE_ID "TMPL3yfx8TIqU"
#define BLYNK_TEMPLATE_NAME "smart bat"
#define BLYNK_AUTH_TOKEN "PZWBnKxazhQYWJYU00Cvao6Qp4zPejMt"

#define BLYNK_PRINT Serial

/*************** LIBRARIES ***************/
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

/*************** WIFI ***************/
char ssid[] = "Airtel_sugu_0550";
char pass[] = "Air@23860";

/*************** MPU6050 ***************/
MPU6050 mpu;

/*************** VIRTUAL PINS ***************/
const int RESULT_PIN   = V3; // existing result
const int MIC_V_TOP    = V6; // mic top -> V6
const int MIC_V_CENTER = V7; // mic centre -> V7
const int MIC_V_BOT    = V8; // mic bottom -> V8
const int MIC_V_STRING = V9; // human readable event -> V9
const int FEEDBACK_VPIN = V10; // new: show random feedback when bad shot

/*************** PARAMETERS ***************/
const int CAL_SAMPLES_REST = 200;
const int SAMPLE_DELAY_MS = 20;
const int MOVING_BUF = 10;
const int WINDOW_SIZE = 12;

const int MOVEMENT_MIN_GX = 350;
const unsigned long TRIGGER_COOLDOWN = 1500;

/*************** ZONE TUNING (NARROWER / NON-OVERLAPPING) ***************/
const int ZONE_THRESHOLD = 450; // reduced radius for left/right (narrower zones)
const int ZONE_DEADBAND = 150; // central dead zone to avoid ambiguous results

/*************** NEW VERTICAL THRESHOLDS ***************/
const int Y_LIFT_MIN = 800; // right shots → bat up
const int Z_LIFT_MIN = 600;
const int Y_DROP_MIN = 800; // left sweep → bat down
const int Z_DROP_MIN = 500;

/*************** MICROPHONE INPUTS (hardware pins) ***************/
const int MIC_TOP_PIN = D5; // GPIO14
const int MIC_CENTER_PIN = D6; // GPIO12
const int MIC_BOT_PIN = D7; // GPIO13

const unsigned long MIC_DEBOUNCE_MS = 50; // debounce for mic taps
const int buzzer = D8;

/*************** FEEDBACK MESSAGES ***************/
// 10 different feedback strings — edit these as you like
const char* feedbacks[] = {
  "Correct the angle",
  "Play in another phase",
  "Lower the bat downward",
  "Step forward more",
  "Keep elbow bent",
  "Open the face less",
  "Shift weight earlier",
  "Follow through more",
  "Bring bat down quicker",
  "Watch ball earlier"
};
const int FEEDBACK_COUNT = sizeof(feedbacks)/sizeof(feedbacks[0]);

/*************** GLOBALS ***************/
float rest_mean_x = 0;
float rest_mean_y = 0;
float rest_mean_z = 0;

int16_t bufX[MOVING_BUF] = {0}, bufY[MOVING_BUF] = {0}, bufZ[MOVING_BUF] = {0};
int bufIndex = 0;
long sumX=0,sumY=0,sumZ=0;
bool bufFilled=false;

unsigned long lastTrigger=0;

bool sendEnabled=false;
bool rightMode=false;
bool leftMode=false;

/* mic state tracking */
int lastMicState[3] = {LOW, LOW, LOW}; // top, center, bottom
unsigned long lastMicTime[3] = {0,0,0};

/* feedback state */
int lastFeedbackIndex = -1; // -1 means no feedback shown currently
bool lastShotWasGood = true; // track whether previous recognized shot was "good"

/*************** BLYNK HANDLERS ***************/
BLYNK_WRITE(V1){
  if(param.asInt()==1){
    calibrateRest();
    Blynk.virtualWrite(V1,0);
  }
}
BLYNK_WRITE(V2){ sendEnabled = param.asInt(); }
BLYNK_WRITE(V4){ rightMode = param.asInt(); }
BLYNK_WRITE(V5){ leftMode = param.asInt(); }

/*************** SETUP ***************/
void setup(){
  Serial.begin(115200);
  delay(100);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  // seed PRNG from A0 (floating/noise) for random picks
  randomSeed(analogRead(A0));

  // I2C for MPU (SDA, SCL)
  Wire.begin(D2, D1); // D2 = SDA, D1 = SCL
  delay(10);

  // Initialize MPU and check connection
  mpu.initialize();
  delay(50);
  Serial.println("Checking MPU6050 connection...");
  unsigned long start = millis();
  while (!mpu.testConnection()) {
    Serial.println("MPU6050 not detected - retrying...");
    delay(500);
    if (millis() - start > 5000) {
      Serial.println("Still waiting for MPU6050... make sure wiring/power/I2C are correct.");
      start = millis();
    }
  }
  Serial.println("MPU6050 detected and ready.");
  delay(50); // give MPU a moment

  // mic pins
  pinMode(MIC_TOP_PIN, INPUT);
  pinMode(MIC_CENTER_PIN, INPUT);
  pinMode(MIC_BOT_PIN, INPUT);

  // ensure smoothing buffers cleared
  for (int i=0;i<MOVING_BUF;i++){
    bufX[i]=bufY[i]=bufZ[i]=0;
  }
  sumX = sumY = sumZ = 0;
  bufIndex = 0;
  bufFilled = false;

  // connect Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // initial read of mic states
  lastMicState[0] = digitalRead(MIC_TOP_PIN);
  lastMicState[1] = digitalRead(MIC_CENTER_PIN);
  lastMicState[2] = digitalRead(MIC_BOT_PIN);
  for(int i=0;i<3;i++) lastMicTime[i] = millis();

  // initial calibration of resting gyro means
  calibrateRest();

  // ensure feedback starts blank
  Blynk.virtualWrite(FEEDBACK_VPIN, "");
  lastFeedbackIndex = -1;
  lastShotWasGood = true;
}

/*************** ZONE CLASSIFIER (exclusive, narrow zones + deadband) ***************/
int classifyZone(float dX){
  if (fabs(dX) <= ZONE_DEADBAND) return 0; // center / too small -> middle
  if (dX <= -ZONE_THRESHOLD) return 1; // LEFT
  if (dX >= ZONE_THRESHOLD) return 2; // RIGHT
  return 0;
}

/*************** MAIN LOOP ***************/
void loop(){
  Blynk.run();

  // check microphones first (fast)
  checkMic(0, MIC_TOP_PIN, MIC_V_TOP); // top
  checkMic(1, MIC_CENTER_PIN, MIC_V_CENTER); // centre
  checkMic(2, MIC_BOT_PIN, MIC_V_BOT); // bottom

  // read gyro (rotation) values
  int16_t gx = mpu.getRotationX();
  int16_t gy = mpu.getRotationY();
  int16_t gz = mpu.getRotationZ();

  updateBuffer(gx, gy, gz);
  float avgX = getAvgX();
  float avgY = getAvgY();
  float avgZ = getAvgZ();

  float deltaX = avgX - rest_mean_x;
  float deltaY = avgY - rest_mean_y;
  float deltaZ = avgZ - rest_mean_z;

  bool modeError = (leftMode && rightMode);
  if(modeError){
    send("MODE ERROR");
    return;
  }

  // Quick swing check to avoid expensive window sampling for small movements
  bool swing = fabs(deltaX) > (MOVEMENT_MIN_GX * 0.5);
  if(!swing) return;
  if(millis() - lastTrigger < TRIGGER_COOLDOWN) return;

  long sx=0, sy=0, sz=0;
  for(int i=0;i<WINDOW_SIZE;i++){
    int16_t x = mpu.getRotationX();
    int16_t y = mpu.getRotationY();
    int16_t z = mpu.getRotationZ();
    sx += x; sy += y; sz += z;
    delay(SAMPLE_DELAY_MS);
  }

  float wx = (float)sx / WINDOW_SIZE;
  float wy = (float)sy / WINDOW_SIZE;
  float wz = (float)sz / WINDOW_SIZE;

  float dX = wx - rest_mean_x;
  float dY = wy - rest_mean_y;
  float dZ = wz - rest_mean_z;

  int zone = classifyZone(dX);
  String result;

  /********************* PERFECTED LOGIC *********************/
  if(!leftMode && !rightMode){ // STRAIGHT MODE
    if(zone == 0) result = "Front foot defense";
    else result = "bad shot";
  }
  else if(rightMode){ // RIGHT MODE
    bool lifted = (abs(dY) > Y_LIFT_MIN) || (abs(dZ) > Z_LIFT_MIN);
    if(zone == 2 && lifted) result = "Cover drive";
    else result = "bad shot";
  }
  else if(leftMode){ // LEFT MODE
    bool downward = (abs(dY) > Y_DROP_MIN) || (abs(dZ) > Z_DROP_MIN);
    if(zone == 1 && downward) result = "Left drive";
    else result = "bad shot";
  }

  // FEEDBACK HANDLING — only change feedback on transitions:
  if(result == "bad shot"){
    if(lastShotWasGood){
      // only when previous shot was good, show a NEW feedback
      showRandomFeedbackIfBad();
      lastShotWasGood = false;
    }
    // if lastShotWasGood == false, do nothing (keep existing feedback)
  } else {
    // correct shot: clear feedback and mark lastShotWasGood
    if(lastFeedbackIndex != -1){
      Blynk.virtualWrite(FEEDBACK_VPIN, ""); // clear feedback
      lastFeedbackIndex = -1;
    }
    lastShotWasGood = true;
  }

  send(result);
  lastTrigger = millis();
}

/*************** showRandomFeedbackIfBad ***************/
// Show a new random feedback only when called for a bad shot.
// Ensures it's different from the last shown feedback.
void showRandomFeedbackIfBad(){
  int newIdx = random(0, FEEDBACK_COUNT);
  // avoid repeating same feedback twice in a row
  if(FEEDBACK_COUNT > 1){
    int attempts = 0;
    while(newIdx == lastFeedbackIndex && attempts < 10){
      newIdx = random(0, FEEDBACK_COUNT);
      attempts++;
    }
  }
  lastFeedbackIndex = newIdx;
  Blynk.virtualWrite(FEEDBACK_VPIN, feedbacks[newIdx]);
  Serial.println(String("FEEDBACK: ") + feedbacks[newIdx]);
}

/*************** checkMic helper ***************/
void checkMic(int idx, int hwPin, int vPin){
  int state = digitalRead(hwPin);
  unsigned long now = millis();
  if(state != lastMicState[idx]){
    if(now - lastMicTime[idx] > MIC_DEBOUNCE_MS){
      lastMicTime[idx] = now;
      lastMicState[idx] = state;
      Blynk.virtualWrite(vPin, state ? 1 : 0);
      if(state == HIGH){
        String msg;
        if(idx==0) msg = "Top Edge tapped";
        else if(idx==1)
        {
           msg = "Centre Shot tapped";
           // short buzzer pattern for centre
           digitalWrite(buzzer,HIGH);
            delay(50);
            digitalWrite(buzzer,LOW);
            delay(50);
            digitalWrite(buzzer,HIGH);
            delay(50);
            digitalWrite(buzzer,LOW);
            delay(50);
        }
        else msg = "Bottom Edge tapped";
        Blynk.virtualWrite(MIC_V_STRING, msg);
        Serial.println("MIC EVENT: " + msg);
      }
    }
  }
}

/*************** SEND ***************/
void send(String s){
  Serial.println("RESULT: " + s);
  if(sendEnabled){
    Blynk.virtualWrite(RESULT_PIN, s);
  }
}

/*************** CALIBRATION ***************/
void calibrateRest(){
  long sx=0, sy=0, sz=0;
  for(int i=0;i<CAL_SAMPLES_REST;i++){
    sx += mpu.getRotationX();
    sy += mpu.getRotationY();
    sz += mpu.getRotationZ();
    delay(10);
  }
  rest_mean_x = (float)sx / CAL_SAMPLES_REST;
  rest_mean_y = (float)sy / CAL_SAMPLES_REST;
  rest_mean_z = (float)sz / CAL_SAMPLES_REST;

  // reset smoothing buffers so averages align to the new rest baseline
  sumX = sumY = sumZ = 0;
  bufIndex = 0;
  bufFilled = false;
  for (int i=0;i<MOVING_BUF;i++){
    bufX[i]=bufY[i]=bufZ[i]=0;
  }

  // reset mic last states/time so we don't get a burst of events after recalibration
  lastMicState[0] = digitalRead(MIC_TOP_PIN);
  lastMicState[1] = digitalRead(MIC_CENTER_PIN);
  lastMicState[2] = digitalRead(MIC_BOT_PIN);
  unsigned long now = millis();
  for(int i=0;i<3;i++) lastMicTime[i] = now;

  Serial.println("Calibration complete. rest_mean_x=" + String(rest_mean_x) +
                 " rest_mean_y=" + String(rest_mean_y) + " rest_mean_z=" + String(rest_mean_z));
}

/*************** BUFFER SMOOTHING ***************/
void updateBuffer(int16_t x,int16_t y,int16_t z){
  sumX -= bufX[bufIndex];
  sumY -= bufY[bufIndex];
  sumZ -= bufZ[bufIndex];

  bufX[bufIndex] = x;
  bufY[bufIndex] = y;
  bufZ[bufIndex] = z;

  sumX += x;
  sumY += y;
  sumZ += z;

  bufIndex++;
  if(bufIndex >= MOVING_BUF){
    bufIndex = 0;
    bufFilled = true;
  }
}

float getAvgX(){
  int denom = bufFilled ? MOVING_BUF : (bufIndex==0 ? 1 : bufIndex);
  return (float)sumX / denom;
}
float getAvgY(){
  int denom = bufFilled ? MOVING_BUF : (bufIndex==0 ? 1 : bufIndex);
  return (float)sumY / denom;
}
float getAvgZ(){
  int denom = bufFilled ? MOVING_BUF : (bufIndex==0 ? 1 : bufIndex);
  return (float)sumZ / denom;
}
