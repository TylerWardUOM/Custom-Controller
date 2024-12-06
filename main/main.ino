#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>

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
  0x09, 0x32,               // Usage (Z) - Joystick Rotation X
  0x09, 0x33,               // Usage (Rx) - Joystick Rotation Y
  0x15, 0x81,               // Logical Minimum (-127)
  0x25, 0x7F,               // Logical Maximum (127)
  0x75, 0x08,               // Report Size (8 bits per axis)
  0x95, 0x02,               // Report Count (2 axes)
  0x81, 0x02,               // Input (Data, Variable, Absolute)

  0xC0,                     // End Collection (Physical)
  0xC0                      // End Collection (Application)
};


BLEHIDDevice* hidDevice;
BLEServer* server;
BLECharacteristic* inputCharacteristic;
BLECharacteristic* inputCharacteristic2;


const int buttonA = 0;
const int buttonB = 2;
const int buttonC = 4;
const int joystickButton = 5;

bool isConnected = false;
const int DEADZONE = 30;

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

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing BLE device...");
    // Print the chip ID
  uint64_t chipId = ESP.getEfuseMac(); // Returns the MAC address, which is unique
  Serial.print("Chip ID (MAC address): ");
  Serial.println(chipId, HEX);

  BLEDevice::init("My Controler");
  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  hidDevice = new BLEHIDDevice(server);

  hidDevice->manufacturer()->setValue("ESP32 Manufacturer");
  hidDevice->pnp(0x02, 0x092E, 0x2052, 0x0100);
  hidDevice->hidInfo(0x00, 0x01);

  inputCharacteristic = hidDevice->inputReport(0x01);
  inputCharacteristic2 = hidDevice->inputReport(0x02);
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
}

void loop() {
  bool buttonAState = digitalRead(buttonA) == LOW;
  bool buttonBState = digitalRead(buttonB) == LOW;
  bool buttonCState = digitalRead(buttonC) == LOW;
  bool joystickButtonState = digitalRead(joystickButton) == LOW;

  int xAxis = analogRead(39);
  int yAxis = analogRead(36);

  Serial.print("Raw X-axis value: ");
  Serial.println(xAxis);
  Serial.print("Raw Y-axis value: ");
  Serial.println(yAxis);

  xAxis = map(xAxis, 0, 4095, -127, 127);
  yAxis = map(yAxis, 0, 4095, -127, 127);

  if (abs(xAxis - 0) < DEADZONE) {
    xAxis = 0;
  }
  if (abs(yAxis - 0) < DEADZONE) {
    yAxis = 0;
  }

  int RxAxis = 0;
  int RyAxis = 0;




  Serial.print("Normalized X-axis value (with deadzone): ");
  Serial.println(xAxis);
  Serial.print("Normalized Y-axis value (with deadzone): ");
  Serial.println(yAxis);

  // Create a combined report buffer
  uint8_t report1[3] = {
    (uint8_t)(xAxis & 0xFF), // X-axis value
    (uint8_t)(yAxis & 0xFF), // Y-axis value
    (uint8_t)(
      (buttonAState ? (1 << 0) : 0) |
      (buttonBState ? (1 << 1) : 0) |
      (buttonCState ? (1 << 2) : 0) |
      (joystickButtonState ? (1 << 3) : 0)
    ),
  };

  // Create a combined report buffer
  uint8_t report2[2] = {
    (uint8_t)(RxAxis & 0xFF), // X-axis value
    (uint8_t)(RyAxis & 0xFF), // Y-axis value
  };

  inputCharacteristic->setValue(report1, sizeof(report1));
  inputCharacteristic->notify();

  inputCharacteristic2->setValue(report2, sizeof(report2));
  inputCharacteristic2->notify();

  Serial.println("HID report sent.");

  delay(10);
}
