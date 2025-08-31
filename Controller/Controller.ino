#include <Bluepad32.h>
#include <GyverMotor2.h>

#define MOTOR1_LEFT_1 32
#define MOTOR1_LEFT_2 33

#define MOTOR2_RIGHT_1 26
#define MOTOR2_RIGHT_2 25

#define SPEED_SENSOR_LEFT_PIN 35
#define SPEED_SENSOR_RIGHT_PIN 34

#define SLOTS_SPEED_DISK 20
#define MEASURE_SPEED_INTERVAL 100

volatile unsigned long pulseCountLeft = 0;
volatile unsigned long pulseCountRight = 0;

unsigned long speedLeft = 0;
unsigned long speedRight = 0;

double motorDiffCoefficient = 0;

int motorLeftSpeed = 0;
int motorRightSpeed = 0;

int controlMode = 0;

GMotor2<DRIVER2WIRE_PWM> motorLeft(MOTOR1_LEFT_2, MOTOR1_LEFT_1); 
GMotor2<DRIVER2WIRE_PWM> motorRight(MOTOR2_RIGHT_2, MOTOR2_RIGHT_1); 

ControllerPtr myControllers[BP32_MAX_GAMEPADS];


void IRAM_ATTR pulseISRLeft() {
  pulseCountLeft++;
}

void IRAM_ATTR pulseISRRight() {
  pulseCountRight++;
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

// void dumpGamepad(ControllerPtr ctl) {
//     Serial.printf(
//         "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//         "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
//         ctl->index(),        // Controller Index
//         ctl->dpad(),         // D-pad
//         ctl->buttons(),      // bitmask of pressed buttons
//         ctl->axisX(),        // (-511 - 512) left X Axis
//         ctl->axisY(),        // (-511 - 512) left Y axis
//         ctl->axisRX(),       // (-511 - 512) right X axis
//         ctl->axisRY(),       // (-511 - 512) right Y axis
//         ctl->brake(),        // (0 - 1023): brake button
//         ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
//         ctl->miscButtons(),  // bitmask of pressed "misc" buttons
//         ctl->gyroX(),        // Gyro X
//         ctl->gyroY(),        // Gyro Y
//         ctl->gyroZ(),        // Gyro Z
//         ctl->accelX(),       // Accelerometer X
//         ctl->accelY(),       // Accelerometer Y
//         ctl->accelZ()        // Accelerometer Z
//     );
// }

void processGamepad(ControllerPtr ctl) {
  processSelectControlMode(ctl);

  switch (controlMode){
    case 0:
      processRunControlMode0(ctl);
      break;
    case 1:
      processRunControlMode1(ctl);
      break;
    case 2:
    break;
  }  
}


void calcSpeedBothMotor(int direction,int baseSpeed){
  if (abs(baseSpeed)<30){
    baseSpeed = 0;
  }

  int calcMotorRightSpeed = motorRightSpeed+(int)(motorRightSpeed*motorDiffCoefficient);

  motorLeftSpeed = baseSpeed;
  // Без поправок
  // motorRightSpeed = baseSpeed;
  // Используя поправочный коэфициент разности моторов
  motorRightSpeed = baseSpeed-(int)(baseSpeed*motorDiffCoefficient);

  if (direction<0){
    float popr = (abs(direction)/(float)511 * motorLeftSpeed)/float(3);     
    motorLeftSpeed -= (int)popr;
  }else{
    float popr = (abs(direction)/(float)511 * motorRightSpeed)/float(3);      
    motorRightSpeed -= (int)popr;
  }
}

void processRunControlMode0(ControllerPtr ctl){
  int baseSpeed = map(ctl->throttle()-ctl->brake(),-1023,1023,-255,255);
  int direction = ctl->axisX();
  calcSpeedBothMotor(direction,baseSpeed);
  //Serial.printf("x: %4d, motor1: %4d, motor2:  %4d\n",direction,motorLeftSpeed,motorRightSpeed);
}


void processRunControlMode1(ControllerPtr ctl){
  int baseSpeed = map(ctl->axisRY(),512,-511,-255,255);
  int direction = ctl->axisRX();
  calcSpeedBothMotor(direction,baseSpeed);
 // Serial.printf("x: %4d, motor1: %4d, motor2:  %4d\n",direction,motorLeftSpeed,motorRightSpeed);
}


void processSelectControlMode(ControllerPtr ctl){
  if (ctl->miscSelect()){
    controlMode++;
    if (controlMode>2){
      controlMode = 0;
    }
    Serial.printf("Change control mode: %4d",controlMode);
    for (int i=0;i<controlMode+1;i++){
      ctl->playDualRumble(0,150,400,400);
      delay(500);
    }    
  }
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

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup SpeedSensor
    pinMode(SPEED_SENSOR_LEFT_PIN, INPUT);
    pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT_PIN), pulseISRLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT_PIN), pulseISRRight, RISING);


    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    //BP32.forgetBluetoothKeys();

    motorLeft.smoothMode(1); 
    motorLeft.setMinDuty(60);

    motorRight.smoothMode(1); 
    motorRight.setMinDuty(60);
    
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    motorLeft.tick();
    motorRight.tick();

    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    
    // Вычисляем скорость моторов
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    if (now - lastTime >= MEASURE_SPEED_INTERVAL) {
      //Serial.printf("Импульсов слева: %d справа: %d\n",pulseCountLeft,pulseCountRight);  
      speedLeft = pulseCountLeft;
      speedRight = pulseCountRight;

      if (motorLeftSpeed!=0){
        double calcSpeedRight = (motorRightSpeed*pulseCountLeft)/motorLeftSpeed;
        if (speedRight!=0){
          motorDiffCoefficient = (calcSpeedRight-speedRight)/speedRight;
        }else{
          motorDiffCoefficient = 0;
        }        
      }else{
        motorDiffCoefficient = 0;
      }   

      pulseCountLeft = 0;  // Сбрасываем счетчик
      pulseCountRight = 0;  // Сбрасываем счетчик    
      lastTime = now;
    } 

    Serial.printf("m1: %4d, imp1: %4d, m2:  %4d, imp2: %4d, koef: %4f\n",motorLeftSpeed,speedLeft,motorRightSpeed,speedRight,motorDiffCoefficient); 

    motorLeft.setSpeed(motorLeftSpeed);
    motorRight.setSpeed(motorRightSpeed);

    vTaskDelay(1);
}
