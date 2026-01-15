/*

//DEMO motors, OLED, INA260, 

  Sensor & Output Function
  - Reads battery voltage/current from INA260, prints to OLED
  - Beeps if battery voltage below threshold (16.5V)
  - Reads IMU XYZ acceleration and XYZ Gyroscope, and temperature; prints to Serial
  - Beeps if tilt exceeds 45° for at least 1 second
  - Pairs to PS4 controller, moves motors based on analog Joystick values

ES32-Wroom-32UE (Digikey P/N: 1965-ESP32-DEVKITC-32UE-ND) with classic Bluetooth needed for PS4 controller
   ESP32 Bluetooth MAC 5C:01:3B:63:67:56   //yours will be different!!!!!!!!
   ESP32 wifi: 5C:01:3B:63:67:54           //yours will be different!!!!!!!!



Motor Drivers: DRV8833

Sensor i2c addresses: 
OLED (32x128) 
PCA9685
INA260 


uses the LEDC PWM for ESP32
//you must downgrade the esp32 board package to pre3.0 version for the espPWM to compile
//compiles on v 2.0.17 esp32 by Espressif

 Table 1. H-Bridge Logic (From TI datasheet) https://www.ti.com/lit/ds/symlink/drv8833.pdf

 xIN1 | xIN2 | xOUT1 | xOUT2 |   FUNCTION
 -----------------------------------------
  0   |  0   |   Z   |   Z   | Coast = Fast Decay
  0   |  1   |   L   |   H   | Reverse
  1   |  0   |   H   |   L   | Forward
  1   |  1   |   L   |   L   | Brake = Slow Decay (i cant get this to work irl)
  motors just act like coast/fast decay when all set to H
   
Motor/chassis Layout:

       FRONT
    ___________
  /            \
 │  ◄ L1   R1 ► │
 |              | 
 |              | 
 │  ◄ L2   R2 ► │
 └──────────────┘
       BACK
*/

#include <PS4Controller.h> 
#include <WiFi.h>  // make sure this is included for WiFi.macAddress()
//aside from printing  wifi mac to the oled (for convienence only) initializing this is pointless and 
#include <esp_bt.h> // Include for BT MAC address

int driveDirection = 1;   // 1 = normal, -1 = reversed (drive direction/orientation mode)
#define BUZZER_PIN 23     // piezo buzzer
#define BUZZER_CHANNEL 8   // free LEDC channel (0–15 available, motors already use 0–7)

// Deadzone for joystick
#define DEADZONE 15  

//===IMU setup===
#include <Adafruit_LSM6DS3TRC.h>
#include <math.h>
// ===== Accelerometer/Gyro =====
Adafruit_LSM6DS3TRC lsm6ds3trc;
// ===== Tilt Detection =====
const float TILT_THRESHOLD = 45.0;      // Degrees
const unsigned long TILT_HOLD = 1000;   // 1 second
unsigned long tiltStartTime = 0;


// ==== NeoPixel Strip ====
#include <Adafruit_NeoPixel.h>
#define LED_PIN 13
#define LED_COUNT 4
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ==== Store speeds for LED update ====
int motorSpeeds[4] = {0, 0, 0, 0};     // Current speed for each motor, used for RGB LEDs

// ===== OLED DISPLAY SETTINGS =====
#include <GyverOLED.h>
GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled; // Using 128x32 without buffer

// ===== INA260 SENSOR SETTINGS =====
#include <Adafruit_INA260.h>
#define INA260_ADDRESS 0x45 // set by both solder jumpers
Adafruit_INA260 ina260;

//special constrain (map) for floating numbers. Needed for battery voltage %
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// //Speed toggle with R1
// float speedMultiplier = 1.0;   // default high speed
// bool lowSpeedMode = false;     // toggle state

// Motor driver pin assignments (DRV8833)
#define L_MOTOR1_IN1 13   //(Front Left) //was GPIO14, which was a strapping pin :(
#define L_MOTOR1_IN2 27   //(Front Left) 
#define L_MOTOR2_IN1 26   //(Rear Left)  
#define L_MOTOR2_IN2 25   //(Rear Left)  

#define R_MOTOR1_IN1 19   //(Front Right) 
#define R_MOTOR1_IN2 18   //(Front Right) 
#define R_MOTOR2_IN1 17    //(Rear Right)  //was GPIO5, which was a strapping pin :(
#define R_MOTOR2_IN2 4    //(Rear Right)

#define SLEEP_PIN 33      // DRV8833 nSLEEP pin

// PWM settings
#define PWM_FREQ 2000     // 2kHz → still kinda squealy and noisy but performance seems ok
#define PWM_RES  12        // 12-bit resolution → values 0–4095

// Assign LEDC channels
#define CH_L1 0
#define CH_L2 1
#define CH_L3 2
#define CH_L4 3
#define CH_R1 4
#define CH_R2 5
#define CH_R3 6
#define CH_R4 7

unsigned long lastTimeStamp = 0;
int lastLX = 0, lastLY = 0, lastRX = 0, lastRY = 0;

void setup() {

  pinMode(SLEEP_PIN, OUTPUT); // DRV8833 Sleep pin
  digitalWrite(SLEEP_PIN, LOW);  // Start with motor drive disabled

ledcSetup(BUZZER_CHANNEL, 2000, 8);   // 2 kHz, 8-bit resolution
ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  Serial.begin(115200);
  delay(500);
    // Initialize NeoPixel strip
  strip.begin();
  strip.show();  // LEDs off at start

  // Initialize INA260
  if (!ina260.begin(INA260_ADDRESS)) {
    Serial.println(F("Couldn't find INA260 chip"));
    while (1) delay(10);
  }
  Serial.println(F("Found INA260 chip"));

  // ----- LSM6DS3TR-C init -----
  if (!lsm6ds3trc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) delay(10);
  }
  Serial.println("LSM6DS3TR-C Found");
  lsm6ds3trc.configInt1(false, false, true);  // Accel DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false);  // Gyro DRDY on INT2

  // Initialize OLED
  oled.init(); oled.clear();
  oled.flipH(true);  // true = flip horizontally, false = normal
  oled.flipV(true);  // true = flip horizontally, false = normal
  oled.setScale(2);       // 1–4 (4 = huge text)
  oled.invertText(false);
  oled.setCursorXY(0, 0);
  oled.print(F("Ready"));
  oled.update();
  delay(1000); // wait for reading the screen

  oled.setScale(1);       // 1–4 (4 = huge text)
  oled.clear();

 // Serial Print Wifi and Bluetooth MAC
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_BT); // Get the Bluetooth MAC address
  Serial.print("Bluetooth MAC: ");
  for (int i = 0; i < 5; i++) {
    Serial.printf("%02X:", baseMac[i]);
  }
  Serial.printf("%02X\n", baseMac[5]);
/////Wifi mac
  Serial.print("Wifi MAC: ");
  Serial.println(WiFi.macAddress());

  // OLED Print Bluetooth MAC
  oled.setCursorXY(0, 0);
  oled.print(F("BT MAC:"));
  oled.setCursorXY(0, 8); // move down 8 pixels
    // Convert to string
  char BTmacStr[18];
  sprintf(BTmacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          baseMac[0], baseMac[1], baseMac[2],
          baseMac[3], baseMac[4], baseMac[5]);
  oled.print(BTmacStr);

  // OLED Print WiFi MAC
  oled.setCursorXY(0, 16); // next line
  oled.print(F("WiFi MAC:"));
  oled.setCursorXY(0, 24); // next line
  oled.print(WiFi.macAddress());
  oled.update();
  delay(2000); // wait for reading the screen
  oled.clear(); //final clear before main loop

  PS4.begin("5C:01:3B:63:67:54");
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  // Setup PWM channels
  ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L3, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L4, PWM_FREQ, PWM_RES);

  ledcSetup(CH_R1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R3, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R4, PWM_FREQ, PWM_RES);

  // Attach channels to pins
  ledcAttachPin(L_MOTOR1_IN1, CH_L1);
  ledcAttachPin(L_MOTOR1_IN2, CH_L2);
  ledcAttachPin(L_MOTOR2_IN1, CH_L3);
  ledcAttachPin(L_MOTOR2_IN2, CH_L4);

  ledcAttachPin(R_MOTOR1_IN1, CH_R1);
  ledcAttachPin(R_MOTOR1_IN2, CH_R2);
  ledcAttachPin(R_MOTOR2_IN1, CH_R3);
  ledcAttachPin(R_MOTOR2_IN2, CH_R4);

  // Sleep pin
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);  // Start in low-power mode
}
//==LOOP START=========================================================================================
void loop() {
  delay(100);
  displayDashboard();
  IMU_read();
  
}

// 🟡 Apply deadzone to joystick input
int applyDeadzone(int value) {
  return (abs(value) < DEADZONE) ? 0 : value;
}

//==============Buzzer square wave generation--------------
void buzzerTone(int freq, int duty = 128) {
  ledcWriteTone(BUZZER_CHANNEL, freq);  // set frequency
  ledcWrite(BUZZER_CHANNEL, duty);      // 50% duty cycle by default
}

void buzzerOff() {
  ledcWrite(BUZZER_CHANNEL, 0);         // turn off 
}

void notify() { //do stuff when controller input is detected 
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

// Toggle speed mode when R1 is pressed
// if (PS4.R1()) {
//   lowSpeedMode = !lowSpeedMode;
//   speedMultiplier = lowSpeedMode ? 0.5 : 1.0;  // half speed in low mode
//   Serial.printf("Speed Mode: %s\n", lowSpeedMode ? "LOW" : "HIGH");
//   delay(200); // simple debounce so it doesn’t toggle too fast
//                }
// 🟡 Toggle horn
if (PS4.Square()) {
    buzzerTone(740);   // 740Hz beep (this is F#)
} else {
    buzzerOff();
}

// 🟡 Toggle drive orientation with Cross button
if (PS4.Triangle()) {
    driveDirection *= -1;  // flip between 1 and -1
    Serial.printf("Drive Direction: %s\n", (driveDirection == 1) ? "Normal" : "Reversed");
    delay(200); // debounce
}


// 🟢 Control motors
// Scale joystick -127..127 to motor PWM -255..255
int forward = map(ly, -127, 127, -255, 255);  
int turn    = map(rx, -127, 127, -255, 255);

// forward *= speedMultiplier; //speed controller with R1 toggle
// turn    *= speedMultiplier;

// apply drive direction toggle 
forward *= driveDirection;
turn    *= driveDirection;


int leftSpeed  = constrain(forward + turn, -255, 255);
int rightSpeed = constrain(forward - turn, -255, 255);


    // Scale 8-bit joystick range (-255..255) to 12-bit PWM (0..4095)
    setMotor(CH_L1, CH_L2, leftSpeed);
    setMotor(CH_L3, CH_L4, leftSpeed);
    setMotor(CH_R1, CH_R2, rightSpeed);
    setMotor(CH_R3, CH_R4, rightSpeed);

  // Assign speeds for LED updates
    motorSpeeds[0] = leftSpeed;   // L1
    motorSpeeds[1] = leftSpeed;   // L2
    motorSpeeds[2] = rightSpeed;  // R1
    motorSpeeds[3] = rightSpeed;  // R2
updateMotorLEDs();
   
    lastTimeStamp = millis();
  }


}

// 🟢 DRV8833 motor control function using LEDC PWM
void setMotor(int ch_in1, int ch_in2, int speed) {
  int pwmVal = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);

  if (speed > 0) {
    ledcWrite(ch_in1, pwmVal);
    ledcWrite(ch_in2, 0);
  } else if (speed < 0) {
    ledcWrite(ch_in1, 0);
    ledcWrite(ch_in2, pwmVal);
  } else {
    ledcWrite(ch_in1, 0);
    ledcWrite(ch_in2, 0);
  }
}

// PS4 events
void onConnect() {
  Serial.println("Connected!");
  digitalWrite(SLEEP_PIN, HIGH);  // Wake DRV8833
}

void onDisConnect() {
  Serial.println("Disconnected!");
  digitalWrite(SLEEP_PIN, LOW);   // Sleep DRV8833
}

//======================subroutines=================================================================
// Update all LEDs based on motorSpeeds array, then show the strip
void updateMotorLEDs() {
  for (int i = 0; i < LED_COUNT; i++) {
    int absSpeed = abs(motorSpeeds[i]);
    int brightness = map(absSpeed, 0, 255, 0, 50); // scale brightness for full PWM range

    if (motorSpeeds[i] > 0) {
      strip.setPixelColor(i, strip.Color(0, brightness, 0));   // Green forward
    } 
    else if (motorSpeeds[i] < 0) {
      strip.setPixelColor(i, strip.Color(0, brightness / 5, brightness)); // Blue reverse
    } 
    else {
      strip.setPixelColor(i, strip.Color(20, 0, 0));   // Red stopped/coast
    }
  }
  strip.show();
}
// get data and print to the OLED
void displayDashboard() {
  // 1️⃣ PS4 controller battery %
  int batteryPercent = PS4.Battery(); //this reading +/-10% accruate ?
  oled.setCursorXY(0, 0);
  oled.setScale(1); 
  oled.print(F("PS4: "));
  oled.print(batteryPercent);
  oled.print(F("%   ")); // extra spaces to erase old digits

  // 2️⃣ Bus voltage
  float voltage_V = ina260.readBusVoltage() / 1000.0; //convert from mV to V
  //float batteryVoltage = abs((voltage_V-16.5) / (20.4 - 16.5) *100); //M18 charge level. smart but unsigned values lets the % wrap around
  float batteryVoltage = constrain(mapFloat(voltage_V, 16.5, 20.4, 0.0, 100.0), 0.0f, 100.0f); //battery charge level

  oled.setCursorXY(0, 12);
  oled.print(F("Bat: "));
  oled.print(voltage_V, 1); 
  oled.print(F("V   ")); 
  oled.setCursorXY(72, 12);
   oled.print(F("(")); 
  oled.print(batteryVoltage, 0);
  oled.print(F("%) "));

  // 3️⃣ Current
  float current_A = abs(ina260.readCurrent() / 1000.0); //sometimes its -0.0 which is dumb
  oled.setCursorXY(0, 24);
  oled.print(F("Cur: "));
  oled.print(current_A, 1);
  oled.print(F("A  "));

//  //Speed: High or Low
//  oled.setCursorXY(110, 0);
//  oled.print(lowSpeedMode ? "Lo " : "Hi");

// 4️⃣ Drive Direction indicator
  oled.setCursorXY(110, 12);  // bottom-right corner
  oled.print(driveDirection == 1 ? "Fwd " : "Rev ");

  oled.update();
}

//=== IMU math and sensor get====
void IMU_read(){
  // ----- Read LSM6DS3TR-C -----
  sensors_event_t accel, gyro, temp;
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float tempC = temp.temperature; 
  float tempF = ((tempC * 9/5) + 32) ;
  // Print accelerometer values to Serial
  //Serial.printf("Accel X: %.3f m/s², Y: %.3f m/s², Z: %.3f m/s²\n", ax, ay, az);
  //Calculate the vector magnitue in 3 dimensions (X,Y,Z). so regardless of direction, we can calculate 
//if the 4x4 rover is at a tilt
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  float tilt_deg = acos(az / magnitude) * (180.0 / PI);
  //float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI; //not sure about this
  //float roll  = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI; //unsure


  //Serial.printf("Tilt: %.1f°\n", tilt_deg);
 oled.setCursorXY(66,0); 
 oled.print(F("Angle:")); //print angle in degrees ( no degee sign in oled library)
 oled.print(tilt_deg, 0);
 oled.print(F("  ")); // Clear leftover chars
 //oled.print(F("X:"));
 //oled.print(pitch, 0);

 oled.setCursorXY(75, 24);
 oled.print(F("Tf: "));
 oled.print(tempF, 1);
}


