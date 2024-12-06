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
  0x85, 0x01,

  // Joystick X and Y axes
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x30,               // Usage (X)
  0x09, 0x31,               // Usage (Y)
  0x15, 0x81,               // Logical Minimum (-127)
  0x25, 0x7F,               // Logical Maximum (127)
  0x75, 0x08,               // Report Size (8 bits per axis)
  0x95, 0x02,               // Report Count (2 axes)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  // Buttons (8 buttons)
  0x05, 0x09,               // Usage Page (Button)
  0x19, 0x01,               // Usage Minimum (Button 1)
  0x29, 0x08,               // Usage Maximum (8 buttons)
  0x15, 0x00,               // Logical Minimum (0)
  0x25, 0x01,               // Logical Maximum (1)
  0x75, 0x01,               // Report Size (1 bit)
  0x95, 0x08,               // Report Count (8 buttons)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  // Accelerometer X, Y, Z axes (16 bits each)
  0x05, 0x01,               // Usage Page (Generic Desktop Ctrls)
  0x09, 0x30,               // Usage (X)
  0x09, 0x31,               // Usage (Y)
  0x09, 0x32,               // Usage (Z)
  0x16, 0x00, 0x80,         // Logical Minimum (-32768)
  0x26, 0xFF, 0x7F,         // Logical Maximum (32767)
  0x75, 0x10,               // Report Size (16 bits per axis)
  0x95, 0x03,               // Report Count (3 axes)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  // Gyroscope Z axis (16 bits)
  0x09, 0x35,               // Usage (Rz)
  0x16, 0x00, 0x80,         // Logical Minimum (-32768)
  0x26, 0xFF, 0x7F,         // Logical Maximum (32767)
  0x75, 0x10,               // Report Size (16 bits)
  0x95, 0x01,               // Report Count (1 axis)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  0xC0                      // End Collection
};



BLEHIDDevice* hidDevice;
BLEServer* server;
BLECharacteristic* inputCharacteristic;

const int buttonA = 0;
const int buttonB = 2;
const int buttonC = 4;
const int joystickButton = 5;

bool isConnected = false;
const int DEADZONE = 30;

int16_t ax, ay, az, gx, gy, gz;

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

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing BLE device...");

  BLEDevice::init("My Controller");
  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  hidDevice = new BLEHIDDevice(server);

  hidDevice->manufacturer()->setValue("ESP32 Manufacturer");
  hidDevice->pnp(0x02, 0x070E, 0x2030, 0x0100);
  hidDevice->hidInfo(0x00, 0x01);

  inputCharacteristic = hidDevice->inputReport(0x01);
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

  //readMPU6050();

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


  // Create a combined report buffer
  uint8_t combined_report[11] = {
    (uint8_t)(xAxis & 0xFF),         // Joystick X (low byte)
    (uint8_t)(yAxis & 0xFF),         // Joystick Y (low byte)
    (uint8_t)(
      (buttonAState ? (1 << 0) : 0) |
      (buttonBState ? (1 << 1) : 0) |
      (buttonCState ? (1 << 2) : 0) |
      (joystickButtonState ? (1 << 3) : 0)
    ),
    (uint8_t)(ax & 0xFF),            // Accelerometer X (low byte)
    (uint8_t)((ax >> 8) & 0xFF),     // Accelerometer X (high byte)
    (uint8_t)(ay & 0xFF),            // Accelerometer Y (low byte)
    (uint8_t)((ay >> 8) & 0xFF),     // Accelerometer Y (high byte)
    (uint8_t)(az & 0xFF),            // Accelerometer Z (low byte)
    (uint8_t)((az >> 8) & 0xFF),     // Accelerometer Z (high byte)
    (uint8_t)(gz & 0xFF),            // Gyroscope Z (low byte)
    (uint8_t)((gz >> 8) & 0xFF)      // Gyroscope Z (high byte)
  };

  // Debugging: Print the report to Serial
  Serial.print("Report: ");
  for (int i = 0; i < sizeof(combined_report); i++) {
    Serial.print(combined_report[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  inputCharacteristic->setValue(combined_report, sizeof(combined_report));
  inputCharacteristic->notify();
  Serial.println("HID report sent");

  delay(10);
}
