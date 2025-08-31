#include <Arduino.h>
#include <Bluepad32.h>
#include <GyverMotor2.h>

#define BAUDRATE 115200

// Определение количества «секторов» (равномерно делим 360° на 12 частей).
#define NUM_SECTORS 12

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN -1

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

// Структура пакета лидара: 4 байта заголовка, затем 32 байта данных
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

void IRAM_ATTR pulseISRLeft() {
  pulseCountLeft++;
}

void IRAM_ATTR pulseISRRight() {
  pulseCountRight++;
}

// Пороговые расстояния (в миллиметрах) и время залипания аварии
// ALARM_DIST  — красная зона, WARNING_DIST — жёлтая зона
#define ALARM_DIST 400     // Менее 400 мм -> сектор в красном цвете
#define WARNING_DIST 650   // Менее 650 мм (но >= 400 мм) -> жёлтый
#define ALARM_HOLD_MS 300  // Время (мс), которое сектор будет «залипать» в красном
#define SECTOR_OFFSET 1    // Cдвиг секторов (0..11)

/********************************************************
 *  ГЛОБАЛЬНЫЕ МАССИВЫ
 ********************************************************/
// Храним текущее измеренное расстояние по каждому из 12 секторов.
// При отсутствии данных в секторе значение будет NO_VALUE.
static float sectorDistances[NUM_SECTORS] = { 0.0f };

// Время последнего обновления данных сектора (в миллисекундах).
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };

// Время, до которого сектор должен находиться в состоянии «тревоги» (красный цвет).
static uint32_t sectorAlarmUntil[NUM_SECTORS] = { 0 };

// Константа, обозначающая «нет данных».
static const float NO_VALUE = 99999.0f;

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


/**
 * @brief Ожидание появления в потоке UART специфического заголовка лидара.
 *        Заголовок определён в массиве LIDAR_HEADER.
 * 
 * @param ser Ссылка на Serial (обычно Serial1)
 * @return true, если заголовок найден, иначе false (по таймауту)
 */
bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  // Пытаемся «выровнять» поток байт на заголовок (4 байта)
  while (true) {
    if (ser.available()) {
      uint8_t b = ser.read();
      Serial.println(b);
      if (b == LIDAR_HEADER[matchPos]) {
        // Совпало очередное ожидаемое значение заголовка
        matchPos++;
        if (matchPos == LIDAR_HEADER_LEN) {
          // Все байты заголовка совпали
          return true;
        }
      } else {
        // Сброс, если последовательность прервалась
        matchPos = 0;
      }
    }
    // Если долго не приходит корректный заголовок, выходим с false
    if (millis() - start > 100) {
      return false;
    }
  }
}

/**
 * @brief Преобразует «rawAngle» из пакета (двухбайтовое значение)
 *        в угол в градусах [0..360).
 * 
 * Формула для данного конкретного лидара: angle = (rawAngle - 0xA000) / 64
 * 
 * @param rawAngle Сырой угол (число 16 бит из пакета лидара)
 * @return Угол в градусах, нормализованный в диапазон [0..360)
 */
float decodeAngle(uint16_t rawAngle) {
  float angleDeg = (float)(rawAngle - 0xA000) / 64.0f;
  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;
  return angleDeg;
}

/**
 * @brief Блокирующее чтение заданного количества байт из Serial
 *        с учётом таймаута (timeout_ms).
 * 
 * @param ser       Ссылка на объект Serial (например, Serial1)
 * @param buffer    Указатель на буфер, куда записывать считанные байты
 * @param length    Количество байт, которое необходимо прочитать
 * @param timeout_ms Максимальное время ожидания в миллисекундах (по умолчанию 500 мс)
 * @return true, если все байты успешно прочитаны, иначе false
 */
bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 500) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }
    if (millis() - start > timeout_ms) {
      return false;  // Истёк таймаут, данные не успели прийти
    }
  }
  return true;
}

/**
 * @brief Определяет, к какому сектору [0..11] относится данный угол.
 *        Один сектор = 30°, но для удобства «нулевой» сектор
 *        смещён так, что его диапазон ~ 345..15 градусов.
 * 
 * @param angleDeg Угол в градусах [0..360)
 * @return Индекс сектора [0..11]
 */
int angleToSector(float angleDeg) {
  // Сдвигаем угол на +15°, чтобы 0-й сектор приходился примерно на зону 345..15
  float shifted = angleDeg + 15.0f;
  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;

  // Делим на 30°, получаем индекс сектора
  int sector = (int)(shifted / 30.0f) % NUM_SECTORS;
  sector = (sector + SECTOR_OFFSET) % NUM_SECTORS;
  return sector;
}

/********************************************************
 *  ОСНОВНАЯ ФУНКЦИЯ ПАРСИНГА И ОБРАБОТКИ ДАННЫХ ЛИДАРА
 ********************************************************/

/**
 * @brief Считывает один пакет данных из лидара, обновляет глобальные
 *        массивы расстояний, а также управляет цветом светодиодов.
 * 
 * @return true, если пакет обработан успешно, false — при ошибке чтения
 */
bool parseAndProcessPacket() {
  // 1) Дожидаемся заголовка лидара
  if (!waitForHeader(Serial1)) {
    Serial.println("NO");
    return false;  // Заголовок не найден, пропускаем
  }

  // 2) Считываем тело пакета из 32 байт
  uint8_t buffer[LIDAR_BODY_LEN];
  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 500)) {
    Serial.println("Failed to read 32 bytes");
    return false;
  }

  // 3) Извлекаем общие данные пакета
  //    (Например, скорость вращения, углы начала и конца)
  uint16_t rotationSpeedTmp = buffer[0] | (buffer[1] << 8);
  float rpm = (float)rotationSpeedTmp / 64.0f;

  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  // Данные о расстояниях и интенсивностях занимают 8 групп по 3 байта
  uint8_t offset = 4;
  uint16_t distances[8];
  uint8_t intensities[8];
  for (int i = 0; i < 8; i++) {
    distances[i] = (buffer[offset] | (buffer[offset + 1] << 8));
    intensities[i] = buffer[offset + 2];
    offset += 3;
  }

  // В самом конце пакета — «конечный угол» (2 байта)
  uint16_t endAngleTmp = buffer[offset] | (buffer[offset + 1] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  // Если конечный угол «меньше» начального, то добавим 360
  if (endAngleDeg < startAngleDeg) {
    endAngleDeg += 360.0f;
  }

  // 4) Вычисляем углы для всех 8 точек внутри пакета
  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;
  float packetAngles[8];
  for (int i = 0; i < 8; i++) {
    float angle = startAngleDeg + i * angleInc;
    // Нормализуем в [0..360)
    while (angle < 0) angle += 360.0f;
    while (angle >= 360) angle -= 360.0f;
    packetAngles[i] = angle;
  }

  // 5) Создаём в ременный массив, где соберём минимальные расстояния по секторам
  //    (для 8 точек, что пришли в пакете).
  float tempSectorMin[NUM_SECTORS];
  for (int s = 0; s < NUM_SECTORS; s++) {
    tempSectorMin[s] = NO_VALUE;  // Изначально никаких данных
  }

  // 6) Проходим по всем 8 точкам пакета, фильтруя по интенсивности
  for (int i = 0; i < 8; i++) {
    if (intensities[i] > 15) {
      int sectorIndex = angleToSector(packetAngles[i]);
      float dist = (float)distances[i];
      // Запоминаем минимальное расстояние на сектор
      if (dist < tempSectorMin[sectorIndex]) {
        tempSectorMin[sectorIndex] = dist;
      }
    }
  }

  // 7) Обновляем глобальный массив расстояний и время их обновления
  uint32_t now = millis();
  for (int s = 0; s < NUM_SECTORS; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  // 8) Если сектор не обновлялся более 500 мс, считаем, что данных по нему нет
  for (int s = 0; s < NUM_SECTORS; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  // 9) Обработка «залипания» красного: если новое расстояние < ALARM_DIST,
  //    продлеваем время «AlarmUntil».
  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];
    if (dist != NO_VALUE && dist < ALARM_DIST) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  // // 10) Раскраска светодиодов в зависимости от дистанции и/или «залипания»
  // for (int s = 0; s < NUM_SECTORS; s++) {
  //   float dist = sectorDistances[s];
  //   uint32_t color;

  //   if (dist == NO_VALUE) {
  //     // Если нет данных, сделаем (для примера) зелёный.
  //     // Если нужно выключать, то заменить на strip.Color(0, 0, 0).
  //     color = strip.Color(0, 255, 0);
  //   } else {
  //     // Смотрим, не активен ли режим «залипания» красного
  //     if (millis() < sectorAlarmUntil[s]) {
  //       // Держим красный цвет
  //       color = strip.Color(255, 0, 0);
  //     } else {
  //       // Обычная логика: <WARNING_DIST = жёлтый, иначе зелёный
  //       if (dist < WARNING_DIST) {
  //         color = strip.Color(255, 255, 0);  // Жёлтый
  //       } else {
  //         color = strip.Color(0, 255, 0);  // Зелёный
  //       }
  //     }
  //   }
  //   strip.setPixelColor(s, color);
  // }

  // // 11) Отправляем данные на ленту
  // strip.show();

  //12) (Опционально) выводим отладочную информацию через Serial
  // Serial.print("RPM=");
  // Serial.print(rpm);
  // Serial.print("  sectorDistances: ");
  // for (int s = 0; s < NUM_SECTORS; s++) {
  //   Serial.print(sectorDistances[s]);
  //   Serial.print(", ");
  // }
  // Serial.println();
  // 13) Передаем статусы секторов по UART в виде строки
  //     "SECTORS: 0 1 2 ..." (0=зелёный, 1=жёлтый, 2=красный)
  Serial.print("SECTORS: ");
  for (int s = 0; s < NUM_SECTORS; s++) {
    int sectorStatus;
    float dist = sectorDistances[s];

    if (dist == NO_VALUE) {
      // Нет данных – по логике кода светится зелёный, значит статус = 0
      sectorStatus = 0;
    } else if (millis() < sectorAlarmUntil[s]) {
      // «Залипание» в красном
      sectorStatus = 2;
    } else if (dist < WARNING_DIST) {
      // Жёлтый
      sectorStatus = 1;
    } else {
      // Зелёный
      sectorStatus = 0;
    }

    Serial.print(sectorStatus);
    if (s < (NUM_SECTORS - 1)) {
      Serial.print(" ");
    }
  }
  Serial.println();

  return true;  // Пакет успешно обработан
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(BAUDRATE);
    // Инициализация Serial1 для чтения данных лидара (указать пины RX/TX)
    Serial1.begin(BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

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
     // В основном цикле просто пытаемся считать и обработать пакет
    parseAndProcessPacket();

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

   // Serial.printf("m1: %4d, imp1: %4d, m2:  %4d, imp2: %4d, koef: %4f\n",motorLeftSpeed,speedLeft,motorRightSpeed,speedRight,motorDiffCoefficient); 

    motorLeft.setSpeed(motorLeftSpeed);
    motorRight.setSpeed(motorRightSpeed);

    //vTaskDelay(1);
}
