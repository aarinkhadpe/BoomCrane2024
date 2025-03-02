#include <ESP32Servo.h>

#include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include <vector>

/* Definitions */

#define ESPNOW_WIFI_CHANNEL 6



// enums
enum shaft {
  NONE,
  shaft3,
  shaft2,
  shaft1,
};


// ES is short for "Extending Step"

enum extendingStep {
  ES_NONE,
  ES_LOCKING_SERVO_127_1,
  ES_HOME_1,
  ES_EXTEND_15_1,
  ES_EXTEND_15_2,
  ES_EXTEND_15_3,
  ES_LOCKING_SERVO_142,
  ES_EXTEND_150,
  ES_LOCKING_SERVO_162,
  ES_RETRACT_4,
  ES_LOCKING_SERVO_145_2,
  ES_RETRACT_10,
  ES_LOCKING_SERVO_127_2,
  ES_HOME_2
};

// RS is short for "Retracting Step"

enum retractingStep {
  RS_NONE,
  RS_LOCKING_SERVO_127_1,
  RS_HOME_1,
  RS_EXTEND_15_1,
  RS_EXTEND_15_2,
  RS_EXTEND_15_3,
  RS_EXTEND_10,
  RS_LOCKING_SERVO_145,
  RS_EXTEND_135,
  RS_LOCKING_SERVO_132,
  RS_EXTEND_4,
  RS_LOCKING_SERVO_108,
  RS_RETRACT_150,
  RS_LOCKING_SERVO_127_2,
  RS_HOME_2
};

shaft currentShaft = NONE;

extendingStep shaftExtendingStep = ES_NONE;

retractingStep shaftRetractingStep = RS_NONE;


Servo lockingServo;
//Servo rServo;
Servo hookServo;

const int pinRotateMotor1 = 12;
const int pinRotateMotor2 = 18;
const int pinRotateMotorPWM = 21;

// pin numbers
const int pinStep = 26;
const int pinDirection = 27;

const int pinHomingLimitSwitch = 13;

const int pinLockingServo = 23;
const int pinHookServo = 22;

const int pinLinearActuator1 = 14;
const int pinLinearActuator2 = 15;

const int pinLeftDriveMotor1 = 4;
const int pinLeftDriveMotor2 = 5;


const int pinRightDriveMotor1 = 25;
const int pinRightDriveMotor2 = 33;


// flags

bool shaft3Extending = false;
bool shaft3Retracting = false;

bool shaft2Extending = false;
bool shaft2Retracting = false;

bool shaft1Extending = false;
bool shaft1Retracting = false;


bool shaft3Extended = false;
bool shaft3Retracted = false;

bool shaft2Extended = false;
bool shaft2Retracted = false;

bool shaft1Extended = false;
bool shaft1Retracted = false;

int numberOfSteps = 0;

bool lockHookServo = true;

int pos = 0;

int counter = 0;
int direction = LOW;

// VALUES

int driveLeftMotorSpeed = 0;
int driveRightMotorSpeed = 0;
int driveXAxisPWMValue = 0;
int driveYAxisPWMValue = 0;

// Struct for the Status of Buttons

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

buttonData storedData;

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *controllerData, size_t len, bool broadcast) {
    Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    //Serial.printf("  Message: %s\n", (char *)data);
    memcpy(&storedData, controllerData, sizeof(storedData));
  }
};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *controllerData, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

// CHECKS IF ANY SHAFT IS MOVING

bool isShaftInMotion() {

  if (shaft3Extending == true || shaft2Extending == true || shaft1Extending == true || shaft3Retracting == true || shaft2Retracting == true || shaft1Retracting == true)
    return true;
  else
    return false;
}


// Function for reading the Homing Limit Switch

bool readLimitSwitch() {
  if (digitalRead(pinHomingLimitSwitch) == LOW)
    return true;
  else
    return false;
}

// MOVEMENT

void rotateTopBody() {
  if (storedData.clockwiseButtonStatus == true && storedData.counterClockwiseButtonStatus == false && isShaftInMotion() == false) {
    digitalWrite(pinRotateMotor1, LOW);
    digitalWrite(pinRotateMotor2, HIGH);
    analogWrite(pinRotateMotorPWM, 100);

    Serial.println("CLOCKWISE");
  }

  if (storedData.counterClockwiseButtonStatus == true && storedData.clockwiseButtonStatus == false && isShaftInMotion() == false) {
    digitalWrite(pinRotateMotor1, HIGH);
    digitalWrite(pinRotateMotor2, LOW);
    analogWrite(pinRotateMotorPWM, 100);
    Serial.println("COUNTERCLOCKWISE");
  }

  if (storedData.clockwiseButtonStatus == false && storedData.counterClockwiseButtonStatus == false && isShaftInMotion() == false) {
    digitalWrite(pinRotateMotor1, LOW);
    digitalWrite(pinRotateMotor2, LOW);
  }
}

void moveHookUpOrDown() {
  if (storedData.hookUpButtonStatus == true && storedData.hookDownButtonStatus == false && isShaftInMotion() == false) {
    hookServo.write(160);
    Serial.println("MOVING HOOK UP");
  }

  if (storedData.hookDownButtonStatus == true && storedData.hookUpButtonStatus == false && isShaftInMotion() == false) {
    hookServo.write(20);
    Serial.println("MOVING HOOK DOWN");
  }

  if (storedData.hookUpButtonStatus == false && storedData.hookDownButtonStatus == false && isShaftInMotion() == false) {
    hookServo.write(90);
  }
}


// FUCNTION TO DRIVE THE WHOLE CRANE

void driveCrane() {
  // RE-MAPPING X AXIS VALUES FROM 0 THROUGH 255 TO -255 THROUGH 255. THE NEW CENTER IS NOW AT 0 INSTEAD OF 127

  if (storedData.driveJoystickMappedXAxisValue > 134) {
    driveXAxisPWMValue = map(storedData.driveJoystickMappedXAxisValue, 128, 255, 0, 255);
  } else if (storedData.driveJoystickMappedXAxisValue < 120) {
    driveXAxisPWMValue = map(storedData.driveJoystickMappedXAxisValue, 126, 0, -1, -255);
  } else {
    driveXAxisPWMValue = 0;
  }

  // RE-MAPPING Y AXIS VALUES FROM 0 THROUGH 255 TO -255 THROUGH 255. THE NEW CENTER IS NOW AT 0
  if (storedData.driveJoystickMappedYAxisValue > 134) {
    driveYAxisPWMValue = map(storedData.driveJoystickMappedYAxisValue, 128, 255, 0, 255);
  } else if (storedData.driveJoystickMappedYAxisValue < 120) {
    driveYAxisPWMValue = map(storedData.driveJoystickMappedYAxisValue, 126, 0, -1, -255);
  } else {
    driveYAxisPWMValue = 0;
  }

  driveLeftMotorSpeed = driveYAxisPWMValue + driveXAxisPWMValue;
  driveRightMotorSpeed = driveYAxisPWMValue - driveXAxisPWMValue;

  driveLeftMotorSpeed = constrain(driveLeftMotorSpeed, -255, 255);
  driveRightMotorSpeed = constrain(driveRightMotorSpeed, -255, 255);

  // DIRECTION FOR LEFT DRIVE MOTOR
  if (driveLeftMotorSpeed == 0) {
    digitalWrite(pinLeftDriveMotor1, LOW);
    digitalWrite(pinLeftDriveMotor2, LOW);
  } else if (driveLeftMotorSpeed > 0) {
    digitalWrite(pinLeftDriveMotor1, HIGH);
    digitalWrite(pinLeftDriveMotor2, LOW);
  } else {
    digitalWrite(pinLeftDriveMotor1, LOW);
    digitalWrite(pinLeftDriveMotor2, HIGH);
  }

  //DIRECTION FOR RIGHT DRIVE MOTOR
  if (driveRightMotorSpeed == 0) {
    digitalWrite(pinRightDriveMotor1, LOW);
    digitalWrite(pinRightDriveMotor2, LOW);
  } else if (driveRightMotorSpeed > 0) {
    digitalWrite(pinRightDriveMotor1, HIGH);
    digitalWrite(pinRightDriveMotor2, LOW);
  } else {
    digitalWrite(pinRightDriveMotor1, LOW);
    digitalWrite(pinRightDriveMotor2, HIGH);
  }
}

// FUNCTION FOR POINTING SHAFT UP OR DOWN

void pointShaftUpOrDown() {

  if (storedData.pointShaftUpButtonStatus == true && storedData.pointShaftDownButtonStatus == false && isShaftInMotion() == false) {
    digitalWrite(pinLinearActuator1, HIGH);
    digitalWrite(pinLinearActuator2, LOW);
    Serial.println("POINTING SHAFT UP");
  }

  if (storedData.pointShaftDownButtonStatus == true && storedData.pointShaftUpButtonStatus == false && isShaftInMotion() == false) {
    digitalWrite(pinLinearActuator1, LOW);
    digitalWrite(pinLinearActuator2, HIGH);
    Serial.println("POINTING SHAFT DOWN");
  }

  if (storedData.pointShaftDownButtonStatus == false && storedData.pointShaftUpButtonStatus == false) {
    digitalWrite(pinLinearActuator1, LOW);
    digitalWrite(pinLinearActuator2, LOW);
  }
}

// FUNTION FOR LOCKING SERVO POSITION

void lockingServoPosition() {
  lockingServo.write(pos);
}

//FUNCTION TO MOVE THE KEY

void move() {
  if (counter <= numberOfSteps) {
    digitalWrite(pinDirection, direction);

    digitalWrite(pinStep, HIGH);
    delayMicroseconds(1400);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(1400);

    if (lockHookServo == false) {
      if (direction == LOW) {
        hookServo.write(100);
      }
      if (direction == HIGH) {
        hookServo.write(86);
      }
    }
    if (lockHookServo == true) {
      hookServo.write(90);
    }
  }
}

// FUNCTION FOR HOMING THE KEY

void goToHome() {
  digitalWrite(pinDirection, HIGH);

  digitalWrite(pinStep, HIGH);
  delayMicroseconds(1500);
  digitalWrite(pinStep, LOW);
  delayMicroseconds(1500);
}

// FUCNTION TO EXTEND SHAFTS 1, 2, and 3

void extendShaft() {

  switch (shaftExtendingStep) {
    case ES_LOCKING_SERVO_127_1:
      pos = 127;
      lockingServoPosition();
      delay(2000);
      lockHookServo = true;
      shaftExtendingStep = ES_HOME_1;
      break;

    case ES_HOME_1:

      if (readLimitSwitch() == true) {
        goToHome();
      }
      if (readLimitSwitch() == false) {
        shaftExtendingStep = ES_EXTEND_15_1;
        delay(2000);
      }
      break;

    case ES_EXTEND_15_1:
      numberOfSteps = 375;
      direction = LOW;
      move();
      counter++;

      if (counter == 375) {
        counter = 0;

        if (currentShaft == shaft3) {
          shaftExtendingStep = ES_LOCKING_SERVO_142;
        }

        if (currentShaft == shaft2 || currentShaft == shaft1) {
          shaftExtendingStep = ES_EXTEND_15_2;
        }
      }
      break;

    case ES_EXTEND_15_2:
      numberOfSteps = 375;
      direction = LOW;
      move();
      counter++;

      if (counter == 375) {
        counter = 0;

        if (currentShaft == shaft2) {
          shaftExtendingStep = ES_LOCKING_SERVO_142;
        }

        if (currentShaft == shaft1) {
          shaftExtendingStep = ES_EXTEND_15_3;
        }
      }
      break;

    case ES_EXTEND_15_3:
      numberOfSteps = 350;
      direction = LOW;
      move();
      counter++;

      if (counter == 350) {
        counter = 0;

        if (currentShaft == shaft1) {
          shaftExtendingStep = ES_LOCKING_SERVO_142;
        }
      }
      break;

    case ES_LOCKING_SERVO_142:
      delay(2000);
      pos = 142;
      lockingServoPosition();
      delay(2000);
      shaftExtendingStep = ES_EXTEND_150;

      break;

    case ES_EXTEND_150:
      numberOfSteps = 3750;
      direction = LOW;
      lockHookServo = false;
      move();
      counter++;
      if (counter == 3750) {
        shaftExtendingStep = ES_LOCKING_SERVO_162;
        counter = 0;
      }
      break;

    case ES_LOCKING_SERVO_162:
      delay(2000);
      pos = 162;
      lockingServoPosition();
      delay(2000);
      shaftExtendingStep = ES_RETRACT_4;
      break;

    case ES_RETRACT_4:
      numberOfSteps = 100;
      direction = HIGH;
      move();
      counter++;
      if (counter == 100) {
        shaftExtendingStep = ES_LOCKING_SERVO_145_2;
        counter = 0;
        lockHookServo = true;
        hookServo.write(90);
      }
      break;

    case ES_LOCKING_SERVO_145_2:
      delay(2000);
      pos = 145;
      lockingServoPosition();
      delay(2000);
      shaftExtendingStep = ES_RETRACT_10;
      break;

    case ES_RETRACT_10:
      numberOfSteps = 250;
      direction = HIGH;
      move();
      counter++;
      if (counter == 250) {
        shaftExtendingStep = ES_LOCKING_SERVO_127_2;
        counter = 0;
      }
      break;

    case ES_LOCKING_SERVO_127_2:
      delay(2000);
      pos = 127;
      lockingServoPosition();
      delay(2000);
      shaftExtendingStep = ES_HOME_2;
      break;

    case ES_HOME_2:
      if (readLimitSwitch() == true) {
        goToHome();
      }
      if (readLimitSwitch() == false) {
        shaftExtendingStep = ES_NONE;
        shaft3Extending = false;
        shaft2Extending = false;
        shaft1Extending = false;

        if(currentShaft == shaft3)
          shaft3Extended = true;
          
        if(currentShaft == shaft2)
          shaft2Extended = true;
        
        if(currentShaft == shaft1)
          shaft1Extended = true;


        currentShaft = NONE;
        lockHookServo = true;
      }
      break;
  }
}

// FUNCTION TO RETRACT SHAFTS 1, 2, and 3

void retractShaft() {
  
  switch (shaftRetractingStep) {

    case RS_LOCKING_SERVO_127_1:
      pos = 127;
      lockingServoPosition();
      delay(2000);
      shaftRetractingStep = RS_HOME_1;
      lockHookServo = true;
      break;

    case RS_HOME_1:

      if (readLimitSwitch() == true) {
        goToHome();
      }
      if (readLimitSwitch() == false) {
        shaftRetractingStep = RS_EXTEND_15_1;
        delay(2000);
      }
      break;

    case RS_EXTEND_15_1:
      numberOfSteps = 375;
      direction = LOW;
      move();
      counter++;
      if (counter == 375) {
        counter = 0;

        if (currentShaft == shaft2 || currentShaft == shaft1) {
          shaftRetractingStep = RS_EXTEND_15_2;
        }

        if (currentShaft == shaft3) {
          shaftRetractingStep = RS_EXTEND_10;
        }
      }
      break;

    case RS_EXTEND_15_2:
      numberOfSteps = 375;
      direction = LOW;
      move();
      counter++;
      if (counter == 375) {
        counter = 0;

        if (currentShaft == shaft1) {
          shaftRetractingStep = RS_EXTEND_15_3;
        }

        if (currentShaft == shaft2) {
          shaftRetractingStep = RS_EXTEND_10;
        }
      }
      break;

    case RS_EXTEND_15_3:
      numberOfSteps = 350;
      direction = LOW;
      move();
      counter++;
      if (counter == 350) {
        counter = 0;

        if (currentShaft == shaft1) {
          shaftRetractingStep = RS_EXTEND_10;
        }
      }
      break;

    case RS_EXTEND_10:
      numberOfSteps = 250;
      direction = LOW;
      move();
      counter++;
      if (counter == 250) {
        counter = 0;

        shaftRetractingStep = RS_LOCKING_SERVO_145;
      }
      break;

    case RS_LOCKING_SERVO_145:
      pos = 145;
      lockingServoPosition();
      delay(2000);
      shaftRetractingStep = RS_EXTEND_135;
      break;

    case RS_EXTEND_135:
      numberOfSteps = 3375;
      direction = LOW;
      move();
      counter++;

      if (counter == 3625) {
        shaftRetractingStep = RS_LOCKING_SERVO_132;
        counter = 0;
      }
      break;

    case RS_LOCKING_SERVO_132:
      delay(2000);
      pos = 130;
      lockingServoPosition();
      delay(2000);
      shaftRetractingStep = RS_EXTEND_4;

      break;

    case RS_EXTEND_4:
      numberOfSteps = 111;
      direction = LOW;
      move();
      counter++;
      if (counter == 111) {
        counter = 0;
        shaftRetractingStep = RS_LOCKING_SERVO_108;
      }
      break;

    case RS_LOCKING_SERVO_108:
      delay(2000);
      pos = 105;
      lockingServoPosition();
      delay(2000);
      shaftRetractingStep = RS_RETRACT_150;
      break;

    case RS_RETRACT_150:
      numberOfSteps = 3750;
      direction = HIGH;
      lockHookServo = false;
      move();
      counter++;
      if (counter == 3750) {
        shaftRetractingStep = RS_LOCKING_SERVO_127_2;
        counter = 0;
        lockHookServo = true;
        hookServo.write(90);
      }
      break;

    case RS_LOCKING_SERVO_127_2:
      delay(2000);
      pos = 127;
      lockingServoPosition();
      delay(2000);
      shaftRetractingStep = RS_HOME_2;
      break;

    case RS_HOME_2:
      if (readLimitSwitch() == true) {
        goToHome();
      }
      if (readLimitSwitch() == false) {
        shaftRetractingStep = RS_NONE;
        shaft3Retracting = false;
        shaft2Retracting = false;
        shaft1Retracting = false;
        
        currentShaft = NONE;


        if(currentShaft == shaft3)
          shaft3Extended = false;
       
        if(currentShaft == shaft2)      
          shaft2Extended = false;
      
        if(currentShaft == shaft1)       
          shaft1Extended = false;
        
        lockHookServo = true;
      }
      break;
      
  }
  
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

  Serial.println("ESP-NOW Example - Broadcast Slave");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");

  pinMode(pinHomingLimitSwitch, INPUT_PULLUP);

  lockingServo.attach(pinLockingServo);
  hookServo.attach(pinHookServo);

  pinMode(pinRotateMotor1, OUTPUT);
  pinMode(pinRotateMotor2, OUTPUT);
  pinMode(pinRotateMotorPWM, OUTPUT);

  digitalWrite(pinRotateMotor1, LOW);
  digitalWrite(pinRotateMotor2, LOW);

  pinMode(pinStep, OUTPUT);
  pinMode(pinDirection, OUTPUT);

  digitalWrite(pinLeftDriveMotor1, LOW);
  digitalWrite(pinLeftDriveMotor2, LOW);

  digitalWrite(pinRightDriveMotor1, LOW);
  digitalWrite(pinRightDriveMotor2, LOW);

  digitalWrite(pinLinearActuator1, LOW);
  digitalWrite(pinLinearActuator2, LOW);

  lockingServo.write(127);


  pinMode(pinLinearActuator1, OUTPUT);
  pinMode(pinLinearActuator2, OUTPUT);

  pinMode(pinLeftDriveMotor1, OUTPUT);
  pinMode(pinLeftDriveMotor2, OUTPUT);
  pinMode(pinRightDriveMotor1, OUTPUT);
  pinMode(pinRightDriveMotor2, OUTPUT);
}


void loop() {

  //SHAFT 3 EXTENSION

  if (storedData.shaft3ExtendButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft3Extending = true;
    currentShaft = shaft3;
    shaftExtendingStep = ES_LOCKING_SERVO_127_1;
    Serial.println("EXTENDING SHAFT 3");
  }
  extendShaft();

  //SHAFT 2 EXTENSION

  if (storedData.shaft2ExtendButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft2Extending = true;
    currentShaft = shaft2;
    shaftExtendingStep = ES_LOCKING_SERVO_127_1;
    Serial.println("EXTENDING SHAFT 2");
  }
  extendShaft();

  //SHAFT 1 EXTENSION

  if (storedData.shaft1ExtendButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft1Extending = true;
    currentShaft = shaft1;
    shaftExtendingStep = ES_LOCKING_SERVO_127_1;
    Serial.println("EXTENDING SHAFT 1");
  }
  extendShaft();

  //SHAFT 3 RETRACTION

  if (storedData.shaft3RetractButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft3Retracting = true;
    currentShaft = shaft3;
    shaftRetractingStep = RS_LOCKING_SERVO_127_1;
    Serial.println("RETRACTING SHAFT 3");
  }
  retractShaft();

  //SHAFT 2 RETRACTION

  if (storedData.shaft2RetractButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft2Retracting = true;
    currentShaft = shaft2;
    shaftRetractingStep = RS_LOCKING_SERVO_127_1;
    Serial.println("RETRACTING SHAFT 2");
  }
  retractShaft();

  // SHAFT 1 RETRACTTION

  if (storedData.shaft1RetractButtonStatus == true && isShaftInMotion() == false && storedData.unlockShaftButtonStatus == true) {
    shaft1Retracting = true;
    currentShaft = shaft1;
    shaftRetractingStep = RS_LOCKING_SERVO_127_1;
    Serial.println("RETRACTING SHAFT 1");
  }
  retractShaft();

  // POINT SHAFT UP OR DOWN
  pointShaftUpOrDown();

  //MOVE HOOK UP OR DOWN
  moveHookUpOrDown();

  // ROTATE CLOCKWISE AND COUNTERCLOKWISE
  rotateTopBody();

  // DRIVING THE WHOLE CRANE
  if (isShaftInMotion() == false)
    driveCrane();
}