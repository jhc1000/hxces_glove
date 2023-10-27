#include <Servo.h>
// usable pin 4-8, 9-14, 1-35, 48-21 (28pins)

// [Servo]
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

//serial over USB
#define SERIAL_BAUD_RATE 115200
//serial over Bluetooth
#define BTSERIAL_DEVICE_NAME "hxces-dodo-right"
unsigned long timeout = 20;

#define MAXENCODE "A255B255C255D255E255F255G255H255J255K255L255M255N255O255P255\n"

// [Potentiometer]
//ANALOG INPUT CONFIG
#define FLIP_POTS false  //Flip values from potentiometers (for fingers!) if they are backwards

// int finger_value[15];
#define ANALOG_MAX 4095
#define ANALOG_MIN 0
#define ANGLE_MAX 180
#define ANGLE_MIN 0

// You must install RunningMedian library to use this feature
// https://www.arduino.cc/reference/en/libraries/runningmedian/
#define ENABLE_MEDIAN_FILTER false  //use the median of the previous values, helps reduce noise
#define MEDIAN_SAMPLES 20

#define USING_FORCE_FEEDBACK false

// [PIN]
const int PIN_SERVO[5] = {1,42,40,38,36};
const int PIN_FINGER[15] = {4,5,6,7,15,16,17,18,8,9,10,11,12,13,14};
const int PIN_ENC_SERVO[5] = {2,41,39,37,35};

//function forward config
char* encode(int* flexion);
void decodeData(char* stringToDecode, int* hapticLimits);
int getArgument(char* stringToDecode, char command);
void setupInputs();
int* getFingerPositions(bool calibrating, bool reset);
int* getWirePositions(bool calibrating, bool reset);
void writeServoHaptics(int* hapticLimits);
void scaleLimits(int* hapticLimits, float* scaledLimits);
bool readData(char* input);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(timeout);

  void setupInputs();
  servo1.attach(PIN_SERVO[0]);
  servo2.attach(PIN_SERVO[1]);
  servo3.attach(PIN_SERVO[2]);
  servo4.attach(PIN_SERVO[3]);
  servo5.attach(PIN_SERVO[4]);
}

void loop() {
  // put your main code here, to run repeatedly:
  int* finger_value = getFingerPositions(0, 0);
  int* wire_value = getWirePositions(0, 0);
  Serial.print(encode(finger_value));
  for (int i = 0; i < (sizeof(PIN_ENC_SERVO) / sizeof(PIN_ENC_SERVO[0])); i++) {
    Serial.print(wire_value[i]);
  }
  Serial.println("");

  for (int i = 0; i <= 90; i++) {
    servo1.write(i);
    servo2.write(i);
    servo3.write(i);
    servo4.write(i);
    servo5.write(i);
    vTaskDelay(pdMS_TO_TICKS(5));  
  }
  for (int i = 90; i >= 0; i--) {
    servo1.write(i);
    servo2.write(i);
    servo3.write(i);
    servo4.write(i);
    servo5.write(i);
    vTaskDelay(pdMS_TO_TICKS(5));  
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // #if USING_FORCE_FEEDBACK
  //   char received[sizeof(MAXENCODE)];
  //   if (readData(received)) {
  //     int hapticLimits[5];
  //     //This check is a temporary hack to fix an issue with haptics on v0.5 of the driver, will make it more snobby code later
  //     if (String(received).length() >= 10) {
  //       decodeData(received, hapticLimits);
  //       writeServoHaptics(hapticLimits);
  //     }
  //   }
  // #endif
}

// functions
char* encode(int* flexion) {
  static char stringToEncode[sizeof(MAXENCODE)];
  sprintf(stringToEncode, "A%dB%dC%dD%dE%dF%dG%dH%dI%dJ%dK%dL%dM%dN%dP%d\n",
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
  //Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
}

int getArgument(char* stringToDecode, char command) {
  char* start = strchr(stringToDecode, command);
  if (start == NULL)
    return -1;
  else
    return atoi(start + 1);
}

void setupInputs() {
  for (int i = 0; i < (sizeof(PIN_FINGER) / sizeof(PIN_FINGER[0])); i++) {
    pinMode(PIN_FINGER[i], INPUT);
  }
  for (int i = 0; i < (sizeof(PIN_ENC_SERVO) / sizeof(PIN_ENC_SERVO[0])); i++) {
    pinMode(PIN_ENC_SERVO[i], INPUT);
  } 
}

int* getFingerPositions(bool calibrating, bool reset) {
  static int raw_finger_value[15];
  for (int i = 0; i < (sizeof(PIN_FINGER) / sizeof(PIN_FINGER[0])); i++) {
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
  for (int i = 0; i < (sizeof(PIN_ENC_SERVO) / sizeof(PIN_ENC_SERVO[0])); i++) {
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
  if (hapticLimits[0] >= 0) servo1.write(scaledLimits[0]);
  if (hapticLimits[1] >= 0) servo2.write(scaledLimits[1]);
  if (hapticLimits[2] >= 0) servo3.write(scaledLimits[2]);
  if (hapticLimits[3] >= 0) servo4.write(scaledLimits[3]);
  if (hapticLimits[4] >= 0) servo5.write(scaledLimits[4]);
}

//static scaling, maps to entire range of servo
void scaleLimits(int* hapticLimits, float* scaledLimits) {
  for (int i = 0; i < 5; i++) {
    scaledLimits[i] = 180.0f - hapticLimits[i] / 1000.0f * 180.0f;
  }
}

bool readData(char* input) {
  byte size = Serial.readBytesUntil('\n', input, sizeof(MAXENCODE));
  input[size] = NULL;
  return input != NULL && strlen(input) > 0;
}
