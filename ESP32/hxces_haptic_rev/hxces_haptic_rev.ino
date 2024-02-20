//#include <Wire.h>
//#include <L3G4200D.h>
#include <ESP32Servo.h>
// usable pin 4-8, 9-14, 1-35, 48-21 (28pins)

// [Servo]
Servo servo[5];
ESP32PWM pwm;
int minUs = 400;
int maxUs = 2500;

// [IMU L3G4200D]
//#define I2C_SDA 47
//#define I2C_SCL 48

//TwoWire I2C = TwoWire(0);

//L3G4200D gyroscope;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

//serial over USB
#define SERIAL_BAUD_RATE 115200
//serial over Bluetooth
#define BTSERIAL_DEVICE_NAME "hxces-dodo-right"
unsigned long timeout = 20;

#define MAXENCODE "A,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255\n"
#define MAXDECODE "A255B255C255D255E255\n"
// [Potentiometer]
//ANALOG INPUT CONFIG
#define FLIP_POTS false  //Flip values from potentiometers (for fingers!) if they are backwards

// int finger_value[15];
#define ANALOG_MAX 4096
#define ANALOG_MIN 0
#define ANGLE_MAX 180
#define ANGLE_MIN 0

// You must install RunningMedian library to use this feature
// https://www.arduino.cc/reference/en/libraries/runningmedian/
#define ENABLE_MEDIAN_FILTER false  //use the median of the previous values, helps reduce noise
#define MEDIAN_SAMPLES 20

#define USING_FORCE_FEEDBACK true
int hapticLimits[5] = {0,0,0,0,0};
int servo_fixed[5] = {30,30,30,30,30};
int servo_released[5] = {0,0,0,0,0};

// [PIN]
const int PIN_SERVO[5] = {36,38,40,42,1};
const int PIN_FINGER[12] = {13,12,10,9};
const int PIN_ENC_SERVO[5] = {35,37};
const int PIN_LED[3] = {6,5,4};

// [Serial2]
#define rx2 48
#define tx2 47

// [EBIMU]
// parameter for EMimu
float euler[3];

//function forward config
char* encode(int* flexion);
void decodeData(char* stringToDecode, int* hapticLimits);
int getArgument(char* stringToDecode, char command);
void setupInputs();
void setupPWM();
void setupIMU();
void setServos(int degrees);
int* getFingerPositions(bool calibrating, bool reset);
int* getWirePositions(bool calibrating, bool reset);
void writeServoHaptics(int* hapticLimits);
void scaleLimits(int* hapticLimits, float* scaledLimits);
bool readData(char* input);
char* rx_from_serial();
void rx_from_imu();


// [Multitask]
TaskHandle_t Task1;

void setup() {

  xTaskCreatePinnedToCore(
    SERVO,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(timeout);

  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial2.setTimeout(timeout);
//    I2C.begin(I2C_SDA, I2C_SCL, 100000);
//    Wire.begin(I2C_SDA, I2C_SCL);

  setupInputs();
  setupPWM();
//  setupIMU();
}

void loop() {
  // put your main code here, to run repeatedly:
  int* finger_value = getFingerPositions(0, 0);
  int* wire_value = getWirePositions(0, 0);
  Serial.print(encode(finger_value));

  // for (int i = 0; i < 5; i++) {
  //   Serial.print(wire_value[i]);
  //   Serial.print("//");
  // }
  // Serial.println("");

  vTaskDelay(pdMS_TO_TICKS(5));

  #if USING_FORCE_FEEDBACK
    char received[sizeof(MAXDECODE)];
    if (readData(received)) {
      // Serial.println(received);
      decodeData(received, hapticLimits);
    }
  #endif
  vTaskDelay(pdMS_TO_TICKS(5));

  rx_from_imu();
//  String imu_msg = "*," + String(euler[0]) + "," + String(euler[1]) + "," + String(euler[2]) + ",\n";
//  Serial.print(imu_msg);
  
}

void SERVO(void * parameter) {
  for(;;) {
    #if USING_FORCE_FEEDBACK
      writeServoHaptics(hapticLimits);
      vTaskDelay(pdMS_TO_TICKS(10));
    #endif
    
//    // IMU
//    timer = millis();
//
//    // Read normalized values
//    Vector norm = gyroscope.readNormalize();
//  
//    // Calculate Pitch, Roll and Yaw
//    pitch = pitch + norm.YAxis * timeStep;
//    roll = roll + norm.XAxis * timeStep;
//    yaw = yaw + norm.ZAxis * timeStep;
//  
//    // Output raw
//    Serial.print(" Pitch = ");
//    Serial.print(pitch);
//    Serial.print(" Roll = ");
//    Serial.print(roll);  
//    Serial.print(" Yaw = ");
//    Serial.println(yaw);
//    String imu_msg = "*," + String(pitch) + "," + String(roll) + "," + String(yaw) + ",\n";
//    Serial.print(imu_msg);
//  
//    // Wait to full timeStep period
//    delay((timeStep*1000) - (millis() - timer));
  }
}

// functions
char* encode(int* flexion) {
  static char stringToEncode[sizeof(MAXENCODE)];
  flexion[14] = 0;
  sprintf(stringToEncode, "A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f\n",
          flexion[14], flexion[14], flexion[14], flexion[14], -flexion[0], -flexion[1], 
          flexion[14], -flexion[2], -flexion[3], flexion[14], flexion[14], flexion[14], 
          euler[0], euler[1], euler[2]);
  return stringToEncode;
}

void decodeData(char* stringToDecode, int* hapticLimits) {
  hapticLimits[0] = getArgument(stringToDecode, 'A');  //thumb
  hapticLimits[1] = getArgument(stringToDecode, 'B');  //index
  hapticLimits[2] = getArgument(stringToDecode, 'C');  //middle
  hapticLimits[3] = getArgument(stringToDecode, 'D');  //ring
  hapticLimits[4] = getArgument(stringToDecode, 'E');  //pinky
  
  // Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
}

int getArgument(char* stringToDecode, char command) {
  char* start = strchr(stringToDecode, command);
  if (start == NULL)
    return -1;
  else
    return atoi(start + 1);
}

void setupInputs() {
  for (int i = 0; i < 4; i++) {
    pinMode(PIN_FINGER[i], INPUT);
  }
  for (int i = 0; i < 2; i++) {
    pinMode(PIN_ENC_SERVO[i], INPUT);
  } 
  for (int i =0; i < 3; i++) {
    pinMode(PIN_LED[i], OUTPUT);
  }
}

void setupPWM() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  //servo 주파수가 같은게 여러개면 작동을 안함 원인은 ㅁㄹ
  servo[0].setPeriodHertz(50);     // Standard 50hz servo
	servo[1].setPeriodHertz(50);     // Standard 50hz servo
//	servo[2].setPeriodHertz(200);     // Standard 50hz servo
//	servo[3].setPeriodHertz(200);     // Standard 50hz servo
//	servo[4].setPeriodHertz(333);     // Standard 50hz servo

  // Serial.println("servo channel: ");
  servo[0].attach(PIN_SERVO[0], minUs, maxUs);
	servo[1].attach(PIN_SERVO[1], minUs, maxUs);
//  servo[2].attach(PIN_SERVO[2], minUs, maxUs);
//	servo[3].attach(PIN_SERVO[3], minUs, maxUs);
//	servo[4].attach(PIN_SERVO[4], minUs, maxUs);

  setServos(0);
}

//void setupIMU(){
//  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
//  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
//  {
//    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
//    delay(500);
//  }
// 
//  // Calibrate gyroscope. The calibration must be at rest.
//  // If you don't want calibrate, comment this line.
//  gyroscope.calibrate(100);
//}

void setServos(int degrees) {
    for(int i = 0; i < 2; ++i) {
        servo[i].write(degrees);
        // Serial.println(servo[i].read());
    }
}

int* getFingerPositions(bool calibrating, bool reset) {
  static int raw_finger_value[15];
  for (int i = 0; i < 4; i++) {
    raw_finger_value[i] = map(analogRead(PIN_FINGER[i]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
    // raw_finger_value[i] = analogRead(PIN_FINGER[i]);
    // Serial.print(raw_finger_value[i]);
    // Serial.print(" ");
  }
  // Serial.println("");

  return raw_finger_value;
}

int* getWirePositions(bool calibrating, bool reset) {
  static int raw_wire_value[5];
  for (int i = 0; i < 2; i++) {
    // raw_wire_value[i] = map(analogRead(PIN_ENC_SERVO[i]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
    raw_wire_value[i] = analogRead(PIN_ENC_SERVO[i]);
    // Serial.print(raw_finger_value[i]);
    // Serial.print(" ");
  }
  // raw_wire_value[0] = map(analogRead(PIN_ENC_SERVO[0]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
  // raw_wire_value[1] = map(analogRead(PIN_ENC_SERVO[1]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
  // raw_wire_value[2] = map(analogRead(PIN_ENC_SERVO[2]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
  // raw_wire_value[3] = map(analogRead(PIN_ENC_SERVO[3]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
  // raw_wire_value[4] = map(analogRead(PIN_ENC_SERVO[4]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);

  // Serial.print(raw_wire_value[0]);
  // Serial.print(raw_wire_value[1]);
  // Serial.print(raw_wire_value[2]);
  // Serial.print(raw_wire_value[3]);
  // Serial.println(raw_wire_value[4]);

  // int raw_w_v1 = map(analogRead(PIN_ENC_SERVO[0]), 0, ANALOG_MAX, ANGLE_MIN, ANGLE_MAX);
  // Serial.println(raw_w_v1);
  // Serial.println("");

  return raw_wire_value;
}

void writeServoHaptics(int* hapticLimits) {
  
  if (hapticLimits[1] > 0) {
    servo[0].write(servo_fixed[0]);
    digitalWrite(PIN_LED[0],1);
  }
  else {
    servo[0].write(servo_released[0]);
    digitalWrite(PIN_LED[0],0);
  }
  if (hapticLimits[2] > 0) {
    servo[1].write(servo_fixed[1]);
    digitalWrite(PIN_LED[1],1);
  }
  else {
    servo[1].write(servo_released[1]);
    digitalWrite(PIN_LED[1],0);
  }
//  if (hapticLimits[2] > 0) {
//    servo[2].write(servo_fixed[2]);
//    digitalWrite(PIN_LED[2],1);
//  }
//  else {
//    servo[2].write(servo_released[2]);
//    digitalWrite(PIN_LED[2],0);
//  }
//  if (hapticLimits[3] > 0) {
//    servo[3].write(servo_fixed[3]);
//  }
//  else {
//    servo[3].write(servo_released[3]);
//  }
//  if (hapticLimits[4] > 0) {
//    servo[4].write(servo_fixed[4]);
//  }
//  else {
//    servo[4].write(servo_released[4]);
//  }
}

//static scaling, maps to entire range of servo
void scaleLimits(int* hapticLimits, float* scaledLimits) {
  for (int i = 0; i < 2; i++) {
    // scaledLimits[i] = 180.0f - hapticLimits[i] / 1000.0f * 180.0f;
    scaledLimits[i] = hapticLimits[i];
  }
}

bool readData(char* input) {
  byte size = Serial.readBytesUntil('\n', input, sizeof(MAXENCODE));
  input[size] = NULL;
  return input != NULL && strlen(input) > 0;
}

char* rx_from_serial() {
  char ch[sizeof(MAXDECODE)];
  String msg = Serial.readStringUntil('\n');
  // Serial.println(msg);
  if (msg.startsWith("A")) {
    int term1 = msg.indexOf("A");               //A
    int term2 = msg.indexOf("B", term1 + 1);    //thumb
    int term3 = msg.indexOf("C", term2 + 1);    //index
    int term4 = msg.indexOf("D", term3 + 1);    //middle
    int term5 = msg.indexOf("E", term4 + 1);    //ring
    int term6 = msg.indexOf("\n", term5 + 1);   //pinky
    // Serial.println(term1);
    // Serial.println(term2);
    // Serial.println(term3);
    // Serial.println(term4);
    // Serial.println(term5);
    // Serial.println(term6);
        
    hapticLimits[0] = (msg.substring(term1 + 1, term2)).toInt();
    hapticLimits[1] = (msg.substring(term2 + 1, term3)).toInt();
    //hapticLimits[2] = (msg.substring(term3 + 1, term4)).toInt();
    //hapticLimits[3] = (msg.substring(term4 + 1, term5)).toInt();
    //hapticLimits[4] = (msg.substring(term5 + 1, term6)).toInt();
    
    // Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
  }
  strcpy(ch, msg.c_str());
  // Serial.println(String(ch));
  return ch;
}

void rx_from_imu() {
  if (Serial2.available()>0) {
      String msg_from_imu = Serial2.readStringUntil(0x0a);
    if (msg_from_imu.startsWith("*")) {
      int term1 = msg_from_imu.indexOf("*");               // *
      int term2 = msg_from_imu.indexOf(",", term1 + 1);    // pitch
      int term3 = msg_from_imu.indexOf(",", term2 + 1);    // roll
      int term4 = msg_from_imu.indexOf(",", term3 + 1);    // yaw
  
      String pitch_string = msg_from_imu.substring(term1 + 1, term2);
      String roll_string = msg_from_imu.substring(term2 + 1, term3);
      String yaw_string = msg_from_imu.substring(term3 + 1, term4);
  
      euler[0] = pitch_string.toFloat();
      euler[1] = roll_string.toFloat();
      euler[2] = yaw_string.toFloat();
  
//      Serial.print("Pitch :"); Serial.print(euler[0]);   Serial.print(" ");
//      Serial.print("Roll :"); Serial.print(euler[1]);   Serial.print(" ");
//      Serial.print("Yaw :"); Serial.print(euler[2]);   Serial.println(" ");
    }
    else {
      // Serial.println("====== not imu data ======");
    }
  }
  else {
    // Serial.println("==== No data Received from imu ====");
  }

}
