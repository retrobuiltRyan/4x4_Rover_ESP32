/*
Barebones example for L293D skid steer ESP32-Wroom rover.
this code is not compatible with the ESP32_4x4_Rover project.
This was a proof of concept with off the shelf parts; uses different motor driver.
*/

#include <PS4Controller.h>
//#include "esp_bt_device.h"
//ESP32 wroom32 with two L293D motor drivers.
#define DEADZONE 15  // ← change this to widen or shrink the deadzone
/*
       FRONT
 ┌──────────────┐
 │  ◄ L1   R1 ► │
 |              | 
 |              | 
 │  ◄ L2   R2 ► │
 └──────────────┘
       BACK
*/

// Skid Steer Drive output assignments
#define L_MOTOR1_IN1 13   //(Front Left) M1
#define L_MOTOR1_IN2 27   //(Front Left) 
#define L_MOTOR2_IN1 26   //(Rear Left) M2
#define L_MOTOR2_IN2 25   //(Rear Left) 
#define L_ENABLE_A 32
#define L_ENABLE_B 33

#define R_MOTOR1_IN1 19    //(Front Right) M3
#define R_MOTOR1_IN2 18   //(Front Right) 
#define R_MOTOR2_IN1 17   //(Rear Right) M4
#define R_MOTOR2_IN2 23   //(Rear Right)
#define R_ENABLE_A 2
#define R_ENABLE_B 15

unsigned long lastTimeStamp = 0;
int lastLX = 0, lastLY = 0, lastRX = 0, lastRY = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  //=========Get Bluetooth mac================
  
  Serial.print("Bluetooth MAC: ");
  Serial.println(ESP.getEfuseMac(), HEX);
  
  PS4.begin("8C:28:F6:BF:71:3C"); //change this for your setup!
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  pinMode(L_MOTOR1_IN1, OUTPUT);
  pinMode(L_MOTOR1_IN2, OUTPUT);
  pinMode(L_MOTOR2_IN1, OUTPUT);
  pinMode(L_MOTOR2_IN2, OUTPUT);
  pinMode(L_ENABLE_A, OUTPUT);
  pinMode(L_ENABLE_B, OUTPUT);

  pinMode(R_MOTOR1_IN1, OUTPUT);
  pinMode(R_MOTOR1_IN2, OUTPUT);
  pinMode(R_MOTOR2_IN1, OUTPUT);
  pinMode(R_MOTOR2_IN2, OUTPUT);
  pinMode(R_ENABLE_A, OUTPUT);
  pinMode(R_ENABLE_B, OUTPUT);
}

void loop() {
  delay(100);
}

// 🟡 Apply deadzone to joystick input
int applyDeadzone(int value) {
  return (abs(value) < DEADZONE) ? 0 : value;
}

void notify() {
  if (millis() - lastTimeStamp > 50) {
    int lxRaw = PS4.LStickX();
    int lyRaw = PS4.LStickY();
    int rxRaw = PS4.RStickX();
    int ryRaw = PS4.RStickY();

    int lx = applyDeadzone(lxRaw);
    int ly = applyDeadzone(lyRaw);
    int rx = applyDeadzone(rxRaw);
    int ry = applyDeadzone(ryRaw);

    if (lx != lastLX || ly != lastLY || rx != lastRX || ry != lastRY) {
      Serial.println("Joystick Movement:");
      Serial.printf("  LStick: X=%d, Y=%d\n", lx, ly);
      Serial.printf("  RStick: X=%d, Y=%d\n", rx, ry);
      lastLX = lx;
      lastLY = ly;
      lastRX = rx;
      lastRY = ry;
    }

    // Button presses
    if (PS4.Square())    Serial.println("Button Pressed: SQUARE");
    if (PS4.Cross())     Serial.println("Button Pressed: CROSS");
    if (PS4.Circle())    Serial.println("Button Pressed: CIRCLE");
    if (PS4.Triangle())  Serial.println("Button Pressed: TRIANGLE");
    if (PS4.Up())        Serial.println("Button Pressed: UP");
    if (PS4.Down())      Serial.println("Button Pressed: DOWN");
    if (PS4.Left())      Serial.println("Button Pressed: LEFT");
    if (PS4.Right())     Serial.println("Button Pressed: RIGHT");
    if (PS4.L1())        Serial.println("Button Pressed: L1");
    if (PS4.R1())        Serial.println("Button Pressed: R1");
    if (PS4.L2())        Serial.println("Button Pressed: L2");
    if (PS4.R2())        Serial.println("Button Pressed: R2");
    if (PS4.Share())     Serial.println("Button Pressed: SHARE");
    if (PS4.Options())   Serial.println("Button Pressed: OPTIONS");
    if (PS4.PSButton())  Serial.println("Button Pressed: PS");
    if (PS4.Touchpad())  Serial.println("Button Pressed: TOUCHPAD");

    // 🟢 Control motors
    int forward = ly;  // ← Inversion removed here (was: -ly)
    int turn = rx;
    int leftSpeed = constrain(forward + turn, -255, 255);
    int rightSpeed = constrain(forward - turn, -255, 255);

    leftSpeed *= 2;
    rightSpeed *= 2;

    setMotor(L_MOTOR1_IN1, L_MOTOR1_IN2, L_ENABLE_A, leftSpeed);
    setMotor(L_MOTOR2_IN1, L_MOTOR2_IN2, L_ENABLE_B, leftSpeed);
    setMotor(R_MOTOR1_IN1, R_MOTOR1_IN2, R_ENABLE_A, rightSpeed);
    setMotor(R_MOTOR2_IN1, R_MOTOR2_IN2, R_ENABLE_B, rightSpeed);

    lastTimeStamp = millis();
  }
}

void setMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisConnect() {
  Serial.println("Disconnected!");
}
