#include <Bluepad32.h>

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

//#define DEBUG

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn;
  int pwmSpeedChannel;
};

MOTOR_PINS motorPins[] = {
  { 23, 22, 19, 4 },  //BACK_RIGHT_MOTOR pins: IN1, IN2, EN, PWM Speed Channel(leave PWM alone)
  { 1, 3, 18, 5 },    //BACK_LEFT_MOTOR
  { 27, 26, 33, 6 },  //FRONT_RIGHT_MOTOR
  { 12, 14, 25, 7 },  //FRONT_LEFT_MOTOR
};

const int PWMFreq = 50000; //50 KHz (to prevent audible whine)
const int PWMResolution = 8;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void ProcessCarMovement(int inputValue, ControllerPtr ctl) {
  int X = ctl->axisX();  // (-511 - 512) left X Axis
  int Y = ctl->axisY();  // (-511 - 512) left Y axis
  int RX = ctl->axisRX();
  switch (inputValue) {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, abs(Y / 2));
      rotateMotor(BACK_RIGHT_MOTOR, abs(Y / 2));
      rotateMotor(FRONT_LEFT_MOTOR, abs(Y / 2));
      rotateMotor(BACK_LEFT_MOTOR, abs(Y / 2));
      break;

    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, (Y / 2));
      rotateMotor(BACK_RIGHT_MOTOR, (Y / 2));
      rotateMotor(FRONT_LEFT_MOTOR, (Y / 2));
      rotateMotor(BACK_LEFT_MOTOR, (Y / 2));
      break;

    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, (abs(X / 2)));
      rotateMotor(BACK_RIGHT_MOTOR, -(abs(X / 2)));
      rotateMotor(FRONT_LEFT_MOTOR, -(abs(X / 2)));
      rotateMotor(BACK_LEFT_MOTOR, (abs(X / 2)));
      break;

    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -(abs(X / 2)));
      rotateMotor(BACK_RIGHT_MOTOR, (abs(X / 2)));
      rotateMotor(FRONT_LEFT_MOTOR, (abs(X / 2)));
      rotateMotor(BACK_LEFT_MOTOR, -(abs(X / 2)));
      break;

    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, (max(abs(X), Y) / 2));
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, (max(abs(X), Y) / 2));
      break;

    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, (max(X, Y) / 2));
      rotateMotor(FRONT_LEFT_MOTOR, (max(X, Y) / 2));
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;

    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -(max(abs(X), abs(Y)) / 2));
      rotateMotor(FRONT_LEFT_MOTOR, -(max(abs(X), abs(Y)) / 2));
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;

    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -(max(abs(X), abs(Y)) / 2));
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -(max(abs(X), abs(Y)) / 2));
      break;

    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, abs(RX / 2));
      rotateMotor(BACK_RIGHT_MOTOR, abs(RX / 2));
      rotateMotor(FRONT_LEFT_MOTOR, -(abs(RX / 2)));
      rotateMotor(BACK_LEFT_MOTOR, -(abs(RX / 2)));
      break;

    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -(abs(RX / 2)));
      rotateMotor(BACK_RIGHT_MOTOR, -(abs(RX / 2)));
      rotateMotor(FRONT_LEFT_MOTOR, abs(RX / 2));
      rotateMotor(BACK_LEFT_MOTOR, abs(RX / 2));
      break;

    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;

    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;
  }
}

void processGamepad(ControllerPtr ctl) {
  int deadzone = 75;
  int Y = ctl->axisY();
  int X = -(ctl->axisX());
  int RX = ctl->axisRX();

  if (RX > deadzone) {
    ProcessCarMovement(TURN_RIGHT, ctl);
  } else if (RX < -deadzone) {
    ProcessCarMovement(TURN_LEFT, ctl);
  } else if (abs(Y) < deadzone && X > deadzone) {
    ProcessCarMovement(RIGHT, ctl);
  } else if (abs(Y) < deadzone && X < -deadzone) {
    ProcessCarMovement(LEFT, ctl);
  } else if (Y > deadzone && abs(X) < deadzone) {
    ProcessCarMovement(FORWARD, ctl);
  } else if (Y < -deadzone && abs(X) < deadzone) {
    ProcessCarMovement(BACKWARD, ctl);
  } else if (Y > deadzone && X > deadzone) {
    ProcessCarMovement(FORWARD_RIGHT, ctl);
  } else if (Y > deadzone && X < -deadzone) {
    ProcessCarMovement(FORWARD_LEFT, ctl);
  } else if (Y < -deadzone && X > deadzone) {
    ProcessCarMovement(BACKWARD_RIGHT, ctl);
  } else if (Y < -deadzone && X < -deadzone) {
    ProcessCarMovement(BACKWARD_LEFT, ctl);
  } else {
    ProcessCarMovement(STOP, ctl);
  }
  #ifdef DEBUG
    dumpGamepad(ctl);
  #endif

  /*
  int LeftFrontWheel = (Speed - Strafe - Turn);
  int RightFrontWheel = (Speed + Strafe + Turn);
  int LeftBackWheel = (Speed + Strafe - Turn);
  int RightBackWheel = (Speed - Strafe + Turn);

  rotateMotor(FRONT_LEFT_MOTOR, (LeftFrontWheel/2));
  rotateMotor(FRONT_RIGHT_MOTOR, (RightFrontWheel/2));
  rotateMotor(BACK_LEFT_MOTOR, (LeftBackWheel/2));
  rotateMotor(BACK_RIGHT_MOTOR, (RightBackWheel/2));
    */
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  } else if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }

  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setUpPinModes() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    //Set up PWM for motor speed
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    rotateMotor(i, 0);
  }
}
// Arduino setup function. Runs in CPU 1
void setup() {

  setUpPinModes();
  
#ifdef DEBUG
  Serial.begin(115200);
  vTaskDelay(10);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
#endif

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  vTaskDelay(1);
}
