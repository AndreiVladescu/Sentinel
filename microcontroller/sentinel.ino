#include <Servo.h>

// Microseconds
#define VERY_LOW_SPEED_DELAY 22
#define LOW_SPEED_DELAY 16
#define MEDIUM_SPEED_DELAY 8
#define HIGH_SPEED_DELAY 4
#define BRAKING_DISTANCE 4

Servo servo_pan, servo_tilt, servo_trigger, servo_pan_panoramic, servo_tilt_panoramic;
unsigned int servo_speed = 10;

int anglePan = 0;
int angleTilt = 0;
int anglePanPanoramic = 0;
int angleTiltPanoramic = 0;

int newAngle = 0;
const int MaxChars = 5;
char strValue[MaxChars];
int index = 0;
bool laserFire = false;
bool constantLasing = true;
int laserPin = 8;


void BrakeServo(int current_angle, int target_angle, Servo *servo, int delta_sign)
{
  /* Braking sequence */
  for (int step_angle = current_angle + delta_sign;
       /*intentionally left blank*/;
       step_angle += delta_sign) {
    servo->write(step_angle);
    if (step_angle == target_angle)
      break;

    delay(VERY_LOW_SPEED_DELAY);
  }
}

void MoveServo(int target_angle, Servo *servo) {
  int current_angle = servo->read();
  if (current_angle == target_angle)
    return;
  int delta_sign = current_angle > target_angle ? -1 : 1;
  int angle_delta = abs(current_angle - target_angle);

  if (angle_delta < BRAKING_DISTANCE) {
    /* If inside the braking distance */
    BrakeServo(current_angle, target_angle, servo, delta_sign);
  }
  else {
    servo->write(target_angle  - (delta_sign * BRAKING_DISTANCE));
    delay(MEDIUM_SPEED_DELAY * angle_delta);
    /* If inside the braking distance */
    BrakeServo(target_angle  - (delta_sign * BRAKING_DISTANCE),
               target_angle, servo, delta_sign);
  }
}

void MoveServoFast(int target_angle, Servo *servo) {
  int current_angle = servo->read();
  if (current_angle == target_angle)
    return;
  int delta_sign = current_angle > target_angle ? -1 : 1;
  int angle_delta = abs(current_angle - target_angle);

  for (int step_angle = current_angle + delta_sign;
       /*intentionally left blank*/;
       step_angle += delta_sign) {
    servo->write(step_angle);
    if (step_angle == target_angle)
      break;
  }
}

void serialEvent() {
  //Serial.write("Ok\n");
  while (Serial.available()) {
    char ch = Serial.read();
    Serial.write(ch);

    if (ch == 'f') {
      if (laserFire) {
        digitalWrite(laserPin, HIGH);
        delay(200);
        digitalWrite(laserPin, LOW);
        delay(200);
        digitalWrite(laserPin, HIGH);
        delay(200);
        digitalWrite(laserPin, LOW);
      }
      else {
        MoveServo(120, &servo_trigger);
        delay(1000);
        MoveServo(85, &servo_trigger);
        delay(1000);
        MoveServo(120, &servo_trigger);
      }
      return;
    }

    if (index < MaxChars && isDigit(ch)) {
      strValue[index++] = ch;
    } else {
      strValue[index] = 0;
      newAngle = atoi(strValue);
      if (newAngle >= 20 && newAngle <= 160) {
        //MOVE_SERVOS:
        if (ch == 't') {
          if (newAngle < 40 || newAngle > 130)
            return;
          MoveServo(newAngle, &servo_tilt);
          angleTilt = newAngle;
        } else if (ch == 'p'){
          MoveServo(newAngle, &servo_pan);
          anglePan = newAngle;
        }
        // s -> sus jos inspirat din tastele w si s
        else if (ch == 's'){
          if (newAngle < 70 || newAngle > 150)
            return;
          MoveServoFast(newAngle, &servo_tilt_panoramic);
          anglePanPanoramic = newAngle;
        }
        // d -> stanga dreapta inspirat din tastele a si d
        else if (ch == 'd'){
          MoveServoFast(newAngle, &servo_pan_panoramic);
          angleTiltPanoramic = newAngle;
        }
      }
      index = 0;
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(laserPin, OUTPUT);
  servo_tilt_panoramic.attach(5);
  servo_pan_panoramic.attach(6);
  servo_tilt.attach(9);
  servo_pan.attach(10);
  servo_trigger.attach(11);

  anglePan = servo_pan.read();
  angleTilt = servo_tilt.read();
  anglePanPanoramic = servo_pan_panoramic.read();
  angleTiltPanoramic = servo_tilt_panoramic.read();
  
  //Serial.write("Ok\n");
  
  MoveServo(90, &servo_tilt);
  MoveServo(90, &servo_pan);
  MoveServoFast(90, &servo_pan_panoramic);
  MoveServoFast(115, &servo_tilt_panoramic);
  
  if (constantLasing)
    digitalWrite(laserPin, HIGH);
}

void loop() {
}
