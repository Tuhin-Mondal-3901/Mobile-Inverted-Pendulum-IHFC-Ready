#include "ICM_20948.h"
#include "MadgwickAHRS.h"

// ==========================================
// OBJECTS & PARAMETERS
// ==========================================
ICM_20948_I2C myICM;
Madgwick filter;

const float WHEEL_RADIUS = 0.045f; // 4.5 cm
const float TRACK_WIDTH  = 0.265f; // 26.5 cm
const float PPR          = 800.0f; // 1/4th Microstepping (Hardware switches OFF/OFF)
const float MAX_STEP_FREQ = 6000.0f; 

float startup_pitch_offset = 0.0f; 


#define STEP_L 27
#define DIR_L  26
#define STEP_R 17
#define DIR_R  16

// ==========================================
// TIMING & MOTOR VARIABLES
// ==========================================
const unsigned long LOOP_US = 10000; // 100 Hz (dt = 0.01)
const float dt = 0.01f;
unsigned long lastLoop = 0;

float stepFreq_L = 0;
float stepFreq_R = 0;
unsigned long lastStep_L = 0;
unsigned long lastStep_R = 0;
bool stepState_L = false;
bool stepState_R = false;

float gyrX_off = 0, gyrY_off = 0, gyrZ_off = 0;

// ==========================================
// PROVEN SOFTWARE PWM MOTOR DRIVER
// ==========================================
void driveMotors() {
  unsigned long now = micros();

  if (stepFreq_L > 1.0f) {
    unsigned long period = (unsigned long)(1000000.0f / stepFreq_L);
    if (now - lastStep_L >= period / 2) {
      stepState_L = !stepState_L;
      digitalWrite(STEP_L, stepState_L);
      lastStep_L = now;
    }
  } else {
    digitalWrite(STEP_L, LOW);
  }

  if (stepFreq_R > 1.0f) {
    unsigned long period = (unsigned long)(1000000.0f / stepFreq_R);
    if (now - lastStep_R >= period / 2) {
      stepState_R = !stepState_R;
      digitalWrite(STEP_R, stepState_R);
      lastStep_R = now;
    }
  } else {
    digitalWrite(STEP_R, LOW);
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L,  OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R,  OUTPUT);
  
  // Enable TMC2225 drivers
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);

  Wire.begin();
  Wire.setClock(400000);
  myICM.begin(Wire, 1);
  myICM.initializeDMP();
  filter.begin(100.0f);

  // --- 1. GYRO BIAS CALIBRATION ---
  Serial.println("Keep still for Gyro calibration...");
  const int N = 200;
  double sgx=0, sgy=0, sgz=0;
  for (int i = 0; i < N; i++) {
    while (!myICM.dataReady()) {}
    myICM.getAGMT();
    sgx += myICM.gyrX();
    sgy += myICM.gyrY();
    sgz += myICM.gyrZ();
    delay(5);
  }
  gyrX_off = sgx / N;
  gyrY_off = sgy / N;
  gyrZ_off = sgz / N;

  // --- 2. FILTER CONVERGENCE ---
  Serial.println("Converging Filter...");
  unsigned long converge = millis();
  while (millis() - converge < 2000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      filter.updateIMU(myICM.gyrX() - gyrX_off, myICM.gyrY() - gyrY_off, myICM.gyrZ() - gyrZ_off, 
                       myICM.accX(), myICM.accY(), myICM.accZ());
      delay(10);
    }
  }

  // --- 3. DYNAMIC PITCH CAPTURE ---
  Serial.println("HOLD UPRIGHT. Capturing 200 samples for Mechanical Datum...");
  float pitch_sum = 0;
  int samples = 0;
  while(samples < 200) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      filter.updateIMU(myICM.gyrX() - gyrX_off, myICM.gyrY() - gyrY_off, myICM.gyrZ() - gyrZ_off, 
                       myICM.accX(), myICM.accY(), myICM.accZ());
      pitch_sum += filter.getPitch();
      samples++;
      delay(10); 
    }
  }
  startup_pitch_offset = pitch_sum / 200.0f;

  Serial.print("Datum Set To: ");
  Serial.println(startup_pitch_offset);
  Serial.println("READY. Armed for balancing.");
  lastLoop = micros();
}

// ==========================================
// TUNABLE GAINS & OFFSETS
// ==========================================
float Kp = 250.0f;  
float Kd = 35.0f;   

//  Tune this to fix the creeping (start with +/- 0.5)
float CG_TRIM = 0.15f; 

// Tune this to fix the yawing/twisting
float K_yaw = 5.0f; 
//  Velocity Damping (Electronic Braking)
// Start extremely small because velocity is measured in Hz (0 to 6000)
float K_vel = 0.001f;
// ==========================================
// THRESHOLD ZONES
// ==========================================
const float DEAD_ZONE = 0.2f;       
const float PANIC_ZONE = 25.0f;     
const float FALL_ZONE = 35.0f;      

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  driveMotors();

  if (micros() - lastLoop < LOOP_US) return;
  lastLoop += LOOP_US;

  if (!myICM.dataReady()) return;
  myICM.getAGMT();

  float gx = myICM.gyrX() - gyrX_off;
  float gy = myICM.gyrY() - gyrY_off;
  float gz = myICM.gyrZ() - gyrZ_off;

  filter.updateIMU(gx, gy, gz, myICM.accX(), myICM.accY(), myICM.accZ());

  // 1. Get raw states
  float pitch_deg = filter.getPitch(); 
  float pitch_rate = gy;               
  float yaw_rate = gz; // Raw twisting speed (ignoring absolute angle)
  
  // 2. Calculate Static Target (Datum + Manual Mechanical Trim)
  float target_pitch = startup_pitch_offset + CG_TRIM;
  
  float error = target_pitch - pitch_deg; 
  float abs_error = abs(error);

  float speed_L = 0;
  float speed_R = 0;

  // 3. THE THRESHOLD STATE MACHINE
  if (abs_error > FALL_ZONE) {
    // SYSTEM FALLEN
    stepFreq_L = 0;
    stepFreq_R = 0;
    Serial.println("FALLEN - EMERGENCY STOP");
    return;
  } 
  
  else if (abs_error > PANIC_ZONE) {
    // PANIC RECOVERY
    float panic_speed = (error > 0) ? MAX_STEP_FREQ : -MAX_STEP_FREQ;
    speed_L = panic_speed;
    speed_R = panic_speed;
  } 
  
  else if (abs_error > DEAD_ZONE) {
    // STANDARD PITCH PD
    float pitch_output = (Kp * error) + (Kd * -pitch_rate); 

    // YAW RATE DAMPING
    float yaw_damp = K_yaw * yaw_rate;

    // DECOUPLED DIFFERENTIAL DISPATCH
    speed_L = pitch_output + yaw_damp;
    speed_R = pitch_output - yaw_damp;
  } 
  
  else {
    // DEAD ZONE
    speed_L = 0;
    speed_R = 0;
  }

  // 4. ACTUATOR DISPATCH
  speed_L = constrain(speed_L, -MAX_STEP_FREQ, MAX_STEP_FREQ);
  speed_R = constrain(speed_R, -MAX_STEP_FREQ, MAX_STEP_FREQ);

  digitalWrite(DIR_L, speed_L >= 0 ? HIGH : LOW);
  digitalWrite(DIR_R, speed_R >= 0 ? LOW : HIGH); 

  stepFreq_L = abs(speed_L);
  stepFreq_R = abs(speed_R);
  
}