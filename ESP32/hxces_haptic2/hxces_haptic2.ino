#include <ESP32Servo.h>
// usable pin 4-8, 9-14, 1-35, 48-21 (28pins)

// [Servo]
Servo servo[5];
ESP32PWM pwm;
int minUs = 400;
int maxUs = 2500;

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

// [PIN]
const int PIN_SERVO[5] = {1,42,40,38,36}; //1 2 5 번 됨 1 42 36
const int PIN_FINGER[15] = {4,5,6,7,15,16,17,18,8,9,10,11,12,13,14};
const int PIN_ENC_SERVO[5] = {2,41,39,37,35};

//function forward config
char* encode(int* flexion);
void decodeData(char* stringToDecode, int* hapticLimits);
int getArgument(char* stringToDecode, char command);
void setupInputs();
void setupPWM();
void setServos(int degrees);
int* getFingerPositions(bool calibrating, bool reset);
int* getWirePositions(bool calibrating, bool reset);
void writeServoHaptics(int* hapticLimits);
void scaleLimits(int* hapticLimits, float* scaledLimits);
bool readData(char* input);
char* rx_from_serial();


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

  setupInputs();
  setupPWM();
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

  // for(int posDegrees = 0; posDegrees < 180; posDegrees++) {
  //       setServos(posDegrees);
  //       // Serial.println(posDegrees);
  //       vTaskDelay(pdMS_TO_TICKS(50));
  //   }

  //   for(int posDegrees = 180; posDegrees > 0; posDegrees--) {
  //       setServos(posDegrees);
  //       // Serial.println(posDegrees);
  //       vTaskDelay(pdMS_TO_TICKS(50));
  //   }
  // setServos(90);
  vTaskDelay(pdMS_TO_TICKS(5));

  #if USING_FORCE_FEEDBACK
    char received[sizeof(MAXDECODE)];
    if (readData(received)) {
      // Serial.println(received);
      decodeData(received, hapticLimits);
    }
  #endif

  // #if USING_FORCE_FEEDBACK
  //   if (Serial.available()>0) {
  //     char* received = rx_from_serial();
  //   }
  // #endif

  vTaskDelay(pdMS_TO_TICKS(5));
}

void SERVO(void * parameter) {
  for(;;) {
    #if USING_FORCE_FEEDBACK
    writeServoHaptics(hapticLimits);
    vTaskDelay(pdMS_TO_TICKS(20));
    #endif
  }
}

// functions
char* encode(int* flexion) {
  static char stringToEncode[sizeof(MAXENCODE)];
  sprintf(stringToEncode, "A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          flexion[0], flexion[1], flexion[2], flexion[3], flexion[4], flexion[5], flexion[6],
          flexion[7], flexion[8], flexion[9], flexion[10], flexion[11], flexion[12], flexion[13],
          flexion[14]);
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
  for (int i = 0; i < 15; i++) {
    pinMode(PIN_FINGER[i], INPUT);
  }
  for (int i = 0; i < 5; i++) {
    pinMode(PIN_ENC_SERVO[i], INPUT);
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
	servo[2].setPeriodHertz(200);     // Standard 50hz servo
	servo[3].setPeriodHertz(200);     // Standard 50hz servo
	servo[4].setPeriodHertz(333);     // Standard 50hz servo

  // Serial.println("servo channel: ");
  servo[0].attach(PIN_SERVO[0], minUs, maxUs);
	servo[1].attach(PIN_SERVO[1], minUs, maxUs);
  servo[2].attach(PIN_SERVO[2], minUs, maxUs);
	servo[3].attach(PIN_SERVO[3], minUs, maxUs);
	servo[4].attach(PIN_SERVO[4], minUs, maxUs);

  setServos(0);
}

void setServos(int degrees) {
    for(int i = 0; i < 5; ++i) {
        servo[i].write(degrees);
        // Serial.println(servo[i].read());
    }
}

int* getFingerPositions(bool calibrating, bool reset) {
  static int raw_finger_value[15];
  for (int i = 0; i < 15; i++) {
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
  for (int i = 0; i < 5; i++) {
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
  float scaledLimits[5];
  scaleLimits(hapticLimits, scaledLimits);
  if (hapticLimits[0] >= 0) servo[0].write(scaledLimits[0]);
  if (hapticLimits[1] >= 0) servo[1].write(scaledLimits[1]);
  if (hapticLimits[2] >= 0) servo[2].write(scaledLimits[2]);
  if (hapticLimits[3] >= 0) servo[3].write(scaledLimits[3]);
  if (hapticLimits[4] >= 0) servo[4].write(scaledLimits[4]);
}

//static scaling, maps to entire range of servo
void scaleLimits(int* hapticLimits, float* scaledLimits) {
  for (int i = 0; i < 5; i++) {
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
    hapticLimits[2] = (msg.substring(term3 + 1, term4)).toInt();
    hapticLimits[3] = (msg.substring(term4 + 1, term5)).toInt();
    hapticLimits[4] = (msg.substring(term5 + 1, term6)).toInt();
    
    // Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
  }
  strcpy(ch, msg.c_str());
  // Serial.println(String(ch));
  return ch;
}

