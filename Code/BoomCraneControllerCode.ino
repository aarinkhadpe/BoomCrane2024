#include "ESP32_NOW.h"
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include <esp_mac.h>

#define ESPNOW_WIFI_CHANNEL 6

// USABLE PINS FOR BUTTONS INCLUDE /13, /12, /14, /27, /26, /25, /33, /32, /15, /4, 5, 18, /19, /21, /22, /23

const int shaft3ExtendButtonPin = 13;
const int shaft3RetractButtonPin = 12;
const int shaft2ExtendButtonPin = 14;
const int shaft2RetractButtonPin = 27;
const int shaft1ExtendButtonPin = 26;
const int shaft1RetractButtonPin = 25;

const int pointShaftUpButtonPin = 22;
const int pointShaftDownButtonPin = 23;

const int hookUpButtonPin = 19;
const int hookDownButtonPin = 21;

const int clockwiseButtonPin = 15;
const int counterClockwiseButtonPin = 4;

const int unlockShaftButtonPin = 18;

const int driveJoystickXAxisPin = 32;
const int driveJoystickYAxisPin = 33;


int driveJoystickXAxisValue;
int driveJoystickYAxisValue;
int mappedXValue;
int mappedYValue;


class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *controllerData, size_t len) {
    if (!send(controllerData, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

/* Global Variables */

uint32_t msg_count = 0;

// Create a broadcast peer object
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);


struct buttonData {
  byte shaft3ExtendButtonStatus;
  byte shaft3RetractButtonStatus;
  byte shaft2ExtendButtonStatus;
  byte shaft2RetractButtonStatus;
  byte shaft1ExtendButtonStatus;
  byte shaft1RetractButtonStatus;

  byte pointShaftUpButtonStatus;
  byte pointShaftDownButtonStatus;

  byte hookUpButtonStatus;
  byte hookDownButtonStatus;

  byte clockwiseButtonStatus;
  byte counterClockwiseButtonStatus;

  byte driveJoystickMappedXAxisValue;
  byte driveJoystickMappedYAxisValue;

  byte unlockShaftButtonStatus;
};

buttonData controllerData;



bool readShaft3ExtendButton() {
  if (digitalRead(shaft3ExtendButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readShaft3RetractButton() {
  if (digitalRead(shaft3RetractButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readShaft2ExtendButton() {
  if (digitalRead(shaft2ExtendButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readShaft2RetractButton() {
  if (digitalRead(shaft2RetractButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readShaft1ExtendButton() {
  if (digitalRead(shaft1ExtendButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readShaft1RetractButton() {
  if (digitalRead(shaft1RetractButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readPointShaftUpButton() {
  if (digitalRead(pointShaftUpButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readPointShaftDownButton() {
  if (digitalRead(pointShaftDownButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readHookUpButton() {
  if (digitalRead(hookUpButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readHookDownButton() {
  if (digitalRead(hookDownButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readClockwiseButton() {
  if (digitalRead(clockwiseButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readCounterClockwiseButton() {
  if (digitalRead(counterClockwiseButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}

bool readUnlockShaftButton() {
  if (digitalRead(unlockShaftButtonPin) == LOW)
    return true;
  else
    ;
  return false;
}



void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Example - Broadcast Master");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    Serial.println("Reebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Setup complete. Broadcasting messages every 5 seconds.");

  pinMode(shaft3ExtendButtonPin, INPUT_PULLUP);
  pinMode(shaft3RetractButtonPin, INPUT_PULLUP);
  pinMode(shaft2ExtendButtonPin, INPUT_PULLUP);
  pinMode(shaft2RetractButtonPin, INPUT_PULLUP);
  pinMode(shaft1ExtendButtonPin, INPUT_PULLUP);
  pinMode(shaft1RetractButtonPin, INPUT_PULLUP);

  pinMode(pointShaftUpButtonPin, INPUT_PULLUP);
  pinMode(pointShaftDownButtonPin, INPUT_PULLUP);

  pinMode(hookUpButtonPin, INPUT_PULLUP);
  pinMode(hookDownButtonPin, INPUT_PULLUP);

  pinMode(clockwiseButtonPin, INPUT_PULLUP);
  pinMode(counterClockwiseButtonPin, INPUT_PULLUP);

  pinMode(unlockShaftButtonPin, INPUT_PULLUP);
}

void loop() {

  // snprintf(controllerData, sizeof(controllerData), "Hello, World! #%lu", msg_count++);



  if (!broadcast_peer.send_message((uint8_t *)&controllerData, sizeof(controllerData))) {
    Serial.println("Failed to broadcast message");
  }

  //SENDING SHAFT 3 EXTEND AND RETRACT BUTTON DATA

  if (readShaft3ExtendButton() == true) {
    controllerData.shaft3ExtendButtonStatus = true;
    Serial.println("Shaft 3 Extend Button Pressed");
  } else {
    controllerData.shaft3ExtendButtonStatus = false;
    Serial.println(" Shaft 3 Extend Button Released");
  }

  //RETRACT

  if (readShaft3RetractButton() == true) {
    controllerData.shaft3RetractButtonStatus = true;
    Serial.println("Shaft 3 Retract Button Pressed");
  } else {
    controllerData.shaft3RetractButtonStatus = false;
    Serial.println("Shaft 3 Retract Button Released");
  }

  //SENDING SHAFT 2 EXTEND AND RETRACT BUTTON DATA

  if (readShaft2ExtendButton() == true) {
    controllerData.shaft2ExtendButtonStatus = true;
    Serial.println("Shaft 2 Extend Button Pressed");
  } else {
    controllerData.shaft2ExtendButtonStatus = false;
    Serial.println("Shaft 2 Extend Button Released");
  }

  //RETRACT

  if (readShaft2RetractButton() == true) {
    controllerData.shaft2RetractButtonStatus = true;
    Serial.println("Shaft 2 Retract Button Pressed");
  } else {
    controllerData.shaft2RetractButtonStatus = false;
    Serial.println("Shaft 2 Retract Button Released");
  }

  //SENDING SHAFT 1 EXTEND AND RETRACT BUTTON DATA

  if (readShaft1ExtendButton() == true) {
    controllerData.shaft1ExtendButtonStatus = true;
    Serial.println("Shaft 1 Extend Button Pressed");
  } else {
    controllerData.shaft1ExtendButtonStatus = false;
    Serial.println("Shaft 1 Extend Button Released");
  }

  //RETRACT

  if (readShaft1RetractButton() == true) {
    controllerData.shaft1RetractButtonStatus = true;
    Serial.println("Shaft 1 Retract Button Pressed");
  } else {
    controllerData.shaft1RetractButtonStatus = false;
    Serial.println("Shaft 1 Retract Button Released");
  }

  // SENDING POINT SHAFT UP BUTTON DATA

  if (readPointShaftUpButton() == true) {
    controllerData.pointShaftUpButtonStatus = true;
    Serial.println("Point Shaft Up Button Has Been Pressed");
  } else {
    controllerData.pointShaftUpButtonStatus = false;
    Serial.println("Point Shaft Up Button Has Been Released");
  }

  // SENDING POINT SHAFT DOWN BUTTON DATA

  if (readPointShaftDownButton() == true) {
    controllerData.pointShaftDownButtonStatus = true;
    Serial.println("Point Shaft Down Button Has Been Pressed");
  } else {
    controllerData.pointShaftDownButtonStatus = false;
    Serial.println("Point Shaft Down Button Has Been Released");
  }

  // SENDING HOOK UP BUTTON DATA

  if (readHookUpButton() == true) {
    controllerData.hookUpButtonStatus = true;
    Serial.println("Hook Up Button Has Been Pressed");
  } else {
    controllerData.hookUpButtonStatus = false;
    Serial.println("Hook Up Button Has Been Released");
  }

  // SENDING HOOK DOWN BUTTON DATA

  if (readHookDownButton() == true) {
    controllerData.hookDownButtonStatus = true;
    Serial.println("Hook Down Button Has Been Pressed");
  } else {
    controllerData.hookDownButtonStatus = false;
    Serial.println("Hook Down Button Has Been Released");
  }

  // SENDING CLOCKWISE BUTTON DATA

  if (readClockwiseButton() == true) {
    controllerData.clockwiseButtonStatus = true;
    Serial.println("Clockwise Button Has Been Pressed");
  } else {
    controllerData.clockwiseButtonStatus = false;
    Serial.println("Clockwise Button Has Been Released");
  }

  //SENDING COUNTERCLOCKWISE BUTTON DATA

  if (readCounterClockwiseButton() == true) {
    controllerData.counterClockwiseButtonStatus = true;
    Serial.println("Counter Clockwise Button Has Been Pressed");
  } else {
    controllerData.counterClockwiseButtonStatus = false;
    Serial.println("Counter Clockwise Button Has Been Released");
  }

  //SENDING OVERRIDE BUTTON DATA

  if (readUnlockShaftButton() == true) {
    controllerData.unlockShaftButtonStatus = true;
    Serial.println("Unlock Shaft Button Has Been Pressed");
  } else {
    controllerData.unlockShaftButtonStatus = false;
    Serial.println("Unlcok Shaft Button Has Been Released");
  }

  // SENDING DRIVE X AXIS VALUES;

  driveJoystickXAxisValue = analogRead(driveJoystickXAxisPin);

  if (driveJoystickXAxisValue > 1900) {
    controllerData.driveJoystickMappedXAxisValue = map(driveJoystickXAxisValue, 1900, 4095, 128, 255);
  } else if (driveJoystickXAxisValue < 1800) {
    controllerData.driveJoystickMappedXAxisValue = map(driveJoystickXAxisValue, 1799, 0, 126, 0);
  } else {
    controllerData.driveJoystickMappedXAxisValue = 127;
  }

  Serial.print("DRIVE X AXIS MAPPED VALUE:");
  Serial.println(controllerData.driveJoystickMappedXAxisValue);

  // SENDING DRIVE Y AXIS VALUES

  driveJoystickYAxisValue = analogRead(driveJoystickYAxisPin);

  if (driveJoystickYAxisValue > 1900) {
    controllerData.driveJoystickMappedYAxisValue = map(driveJoystickYAxisValue, 1900, 4095, 126, 0);
  } else if (driveJoystickYAxisValue < 1800) {
    controllerData.driveJoystickMappedYAxisValue = map(driveJoystickYAxisValue, 1799, 0, 128, 255);
  } else {
    controllerData.driveJoystickMappedYAxisValue = 127;
  }

  Serial.print("DRIVE Y AXIS MAPPED VALUE:");
  Serial.println(controllerData.driveJoystickMappedYAxisValue);
}
