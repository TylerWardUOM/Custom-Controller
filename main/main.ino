#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <Wire.h>

// MPU6050 I2C address (default is 0x68)
const int MPU = 0x68;

const uint8_t hidReportDescriptor[] = {
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x05,               // Usage (Game Pad)
  0xA1, 0x01,               // Collection (Application)
  0x85, 0x01,               // Report ID (1)

  // Joystick and button collection
  0xA1, 0x00,               // Collection (Physical)
  
  // Joystick X and Y axes
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x30,               // Usage (X) - Joystick X
  0x09, 0x31,               // Usage (Y) - Joystick Y
  0x15, 0x81,               // Logical Minimum (-127)
  0x25, 0x7F,               // Logical Maximum (127)
  0x75, 0x08,               // Report Size (8 bits per axis)
  0x95, 0x02,               // Report Count (2 axes)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  // Buttons (8 buttons)
  0x05, 0x09,               // Usage Page (Button)
  0x19, 0x01,               // Usage Minimum (Button 1)
  0x29, 0x08,               // Usage Maximum (Button 8)
  0x15, 0x00,               // Logical Minimum (0)
  0x25, 0x01,               // Logical Maximum (1)
  0x75, 0x01,               // Report Size (1 bit)
  0x95, 0x08,               // Report Count (8 buttons)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  0xC0,                     // End Collection (Physical)

  // Second Joystick collection for Rx and Ry
  0xA1, 0x00,               // Collection (Physical)
  0x85, 0x02,               // Report ID (2)

  // Joystick Rx and Ry axes
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x33,               // Usage (Rx) - Joystick Rotation X
  0x09, 0x34,               // Usage (Ry) - Joystick Rotation Y
  0x15, 0x81,               // Logical Minimum (-127)
  0x25, 0x7F,               // Logical Maximum (127)
  0x75, 0x08,               // Report Size (8 bits per axis)
  0x95, 0x02,               // Report Count (2 axes)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  0xC0,                     // End Collection (Physical)

  // Quaternion collection for orientation
  0xA1, 0x00,               // Collection (Physical)
  0x85, 0x03,               // Report ID (3)

  // Quaternion (Qx, Qy, Qz, Qw)
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x2A,               // Usage (Vector)
  0x15, 0x00,               // Logical Minimum (0)
  0x26, 0xFF, 0x7F,         // Logical Maximum (32767)
  0x75, 0x10,               // Report Size (16 bits per component)
  0x95, 0x04,               // Report Count (4 components)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  0xC0,                      // End Collection (Physical)
  
  0xC0                      // End Collection (Application)
};




BLEHIDDevice* hidDevice;
BLEServer* server;
BLECharacteristic* inputCharacteristic;
BLECharacteristic* inputCharacteristic2;
BLECharacteristic* inputCharacteristic3;


const int buttonA = 0;
const int buttonB = 2;
const int buttonC = 4;
const int joystickButton = 5;

bool isConnected = false;
const int DEADZONE = 30;

// Quaternion variables
float q0 = 1.0; // Scalar component
float q1 = 0.0; // x-component
float q2 = 0.0; // y-component
float q3 = 0.0; // z-component


int16_t ax, ay, az, gx, gy, gz;

// Time management for integration
unsigned long lastTime;
float deltaTime;

// Complementary filter constant (adjust as necessary)
const float alpha = 0.98;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Client connected!");
    isConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Client disconnected!");
    isConnected = false;
  }
};

void setupMPU6050() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up the MPU
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  // Debug prints for accelerometer data
  Serial.print("AX: "); Serial.print(ax); Serial.print("\t");
  Serial.print("AY: "); Serial.print(ay); Serial.print("\t");
  Serial.print("AZ: "); Serial.println(az);

  Wire.read(); Wire.read(); // Skip temperature data

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  // Debug prints for gyroscope data
  Serial.print("GX: "); Serial.print(gx); Serial.print("\t");
  Serial.print("GY: "); Serial.print(gy); Serial.print("\t");
  Serial.print("GZ: "); Serial.println(gz);
}


void computeQuaternions() {
  // Convert to radians
  float ax_f = ax / 16384.0; // Scale accelerometer data to g
  float ay_f = ay / 16384.0;
  float az_f = az / 16384.0;
  float gx_rad = gx * (M_PI / 180.0) / 131.0; // Convert gyroscope data to rad/s
  float gy_rad = gy * (M_PI / 180.0) / 131.0;
  float gz_rad = gz * (M_PI / 180.0) / 131.0;

  // Normalize the accelerometer data
  float norm = sqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
  ax_f /= norm;
  ay_f /= norm;
  az_f /= norm;

  // Calculate roll and pitch from accelerometer data
  float roll = atan2(ay_f, az_f);
  float pitch = atan(-ax_f / sqrt(ay_f * ay_f + az_f * az_f));

  // Integrate gyroscope data to get angular displacement
  float dq0 = -0.5 * (q1 * gx_rad + q2 * gy_rad + q3 * gz_rad);
  float dq1 = 0.5 * (q0 * gx_rad - q3 * gy_rad + q2 * gz_rad);
  float dq2 = 0.5 * (q0 * gy_rad + q3 * gx_rad - q1 * gz_rad);
  float dq3 = 0.5 * (q0 * gz_rad - q1 * gy_rad + q2 * gx_rad);

  // Integrate to get quaternion
  q0 += dq0 * deltaTime;
  q1 += dq1 * deltaTime;
  q2 += dq2 * deltaTime;
  q3 += dq3 * deltaTime;

  // Normalize the quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  // Apply complementary filter to fuse accelerometer and gyroscope data
  float accRoll = atan2(ay_f, az_f);
  float accPitch = atan(-ax_f / sqrt(ay_f * ay_f + az_f * az_f));

  float rollComp = alpha * (roll + gx_rad * deltaTime) + (1 - alpha) * accRoll;
  float pitchComp = alpha * (pitch + gy_rad * deltaTime) + (1 - alpha) * accPitch;

  // Update quaternion components with fused orientation
  float halfRoll = rollComp / 2.0;
  float halfPitch = pitchComp / 2.0;
  float halfYaw = 0.0; // Assuming no yaw component for simplicity

  float q0_new = cos(halfRoll) * cos(halfPitch);
  float q1_new = sin(halfRoll) * cos(halfPitch);
  float q2_new = cos(halfRoll) * sin(halfPitch);
  float q3_new = sin(halfRoll) * sin(halfPitch);

  // Update quaternion with new values
  q0 = q0_new;
  q1 = q1_new;
  q2 = q2_new;
  q3 = q3_new;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing BLE device...");

  BLEDevice::init("My Controller");
  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  hidDevice = new BLEHIDDevice(server);

  hidDevice->manufacturer()->setValue("ESP32 Manufacturer");
  hidDevice->pnp(0x02, 0x090E, 0x2050, 0x0100);
  hidDevice->hidInfo(0x00, 0x01);

  inputCharacteristic = hidDevice->inputReport(0x01);
  inputCharacteristic2 = hidDevice->inputReport(0x02);
  inputCharacteristic3 = hidDevice->inputReport(0x03);
  hidDevice->reportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));
  hidDevice->startServices();

  BLEAdvertising* advertising = server->getAdvertising();
  advertising->setAppearance(HID_GAMEPAD);
  advertising->addServiceUUID(hidDevice->hidService()->getUUID());
  advertising->start();
  Serial.println("BLE advertising started.");

  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  pinMode(buttonC, INPUT_PULLUP);
  pinMode(joystickButton, INPUT_PULLUP);

  hidDevice->setBatteryLevel(100);
  Serial.println("Switch Controller Emulator Ready");

  setupMPU6050();
}

void loop() {
  bool buttonAState = digitalRead(buttonA) == LOW;
  bool buttonBState = digitalRead(buttonB) == LOW;
  bool buttonCState = digitalRead(buttonC) == LOW;
  bool joystickButtonState = digitalRead(joystickButton) == LOW;

  int xAxis = analogRead(39);
  int yAxis = analogRead(36);

  xAxis = map(xAxis, 0, 4095, -127, 127);
  yAxis = map(yAxis, 0, 4095, -127, 127);

  if (abs(xAxis - 0) < DEADZONE) xAxis = 0;
  if (abs(yAxis - 0) < DEADZONE) yAxis = 0;

  readMPU6050();
  computeQuaternions();

  //temp manual axis set
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace or newline characters

    // Parse the command
    if (input.startsWith("ax = ")) {
      ax = input.substring(5).toInt();
      Serial.print("Set AX to: "); Serial.println(ax);
    } else if (input.startsWith("ay = ")) {
      ay = input.substring(5).toInt();
      Serial.print("Set AY to: "); Serial.println(ay);
    } else if (input.startsWith("az = ")) {
      az = input.substring(5).toInt();
      Serial.print("Set AZ to: "); Serial.println(az);
    } else if (input.startsWith("gx = ")) {
      gx = input.substring(5).toInt();
      Serial.print("Set GX to: "); Serial.println(gx);
    } else if (input.startsWith("gy = ")) {
      gy = input.substring(5).toInt();
      Serial.print("Set GY to: "); Serial.println(gy);
    } else if (input.startsWith("gz = ")) {
      gz = input.substring(5).toInt();
      Serial.print("Set GZ to: "); Serial.println(gz);
    } else {
      Serial.println("Unknown command. Use 'ax = <value>' to set axis values.");
    }
  }

  int RxAxis = 0;
  int RyAxis = 0;


  // Create a combined report buffer
  uint8_t report1[3] = {
    (uint8_t)(xAxis & 0xFF),         // Joystick X (low byte)
    (uint8_t)(yAxis & 0xFF),         // Joystick Y (low byte)
    (uint8_t)(
      (buttonAState ? (1 << 0) : 0) |
      (buttonBState ? (1 << 1) : 0) |
      (buttonCState ? (1 << 2) : 0) |
      (joystickButtonState ? (1 << 3) : 0)
    )
  };

  //Report ID 2: 
  uint8_t report2[2] = {
    (uint8_t)(RxAxis & 0xFF),         // Joystick X (low byte)
    (uint8_t)(RyAxis & 0xFF),         // Joystick Y (low byte)
  };

  // Report ID 3: 
  uint8_t report3[8] = {
      (uint8_t)(0 & 0xFF),           // Accelerometer X (low byte)
      (uint8_t)((0 >> 8) & 0xFF),    // Accelerometer X (high byte)
      (uint8_t)(0 & 0xFF),           // Accelerometer Y (low byte)
      (uint8_t)((0 >> 8) & 0xFF),    // Accelerometer Y (high byte)
      (uint8_t)(0 & 0xFF),           // Accelerometer Z (low byte)
      (uint8_t)((0 >> 8) & 0xFF),    // Accelerometer Z (high byte)
      (uint8_t)(0 & 0xFF),           // Accelerometer Z (low byte)
      (uint8_t)((0 >> 8) & 0xFF),    // Accelerometer Z (high byte)
  };

  // Debugging: Print the report to Serial
  Serial.print("Report: ");
  for (int i = 0; i < sizeof(report1); i++) {
    Serial.print(report1[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

    // Debugging: Print the report to Serial
  Serial.print("Report2: ");
  for (int i = 0; i < sizeof(report2); i++) {
    Serial.print(report2[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  inputCharacteristic->setValue(report1, sizeof(report1));
  inputCharacteristic->notify();
  Serial.println("HID report sent");


  inputCharacteristic2->setValue(report2, sizeof(report2));
  inputCharacteristic2->notify();

  inputCharacteristic3->setValue(report3, sizeof(report3));
  inputCharacteristic3->notify();

  delay(10);
}
