#include <Arduino.h>
#include <Bluepad32.h>
#include <GyverMotor2.h>
#include <GTimer.h>
#include <uPID.h>



/*
***************
Общие константы
***************
*/
#define BAUDRATE 115200
#define MEASURE_SPEED_INTERVAL 100

/*
Режим управления
*/
int controlMode = 0;

/*
*******************
Управление моторами.
*******************
*/
#define MOTOR1_LEFT_1 32
#define MOTOR1_LEFT_2 33

#define MOTOR2_RIGHT_1 26
#define MOTOR2_RIGHT_2 25

GMotor2<DRIVER2WIRE_PWM> motorLeft(MOTOR1_LEFT_2, MOTOR1_LEFT_1);
GMotor2<DRIVER2WIRE_PWM> motorRight(MOTOR2_RIGHT_2, MOTOR2_RIGHT_1);

int motorLeftSpeed = 0;
int motorRightSpeed = 0;

const int dt = 30;
uPID pid(P_INPUT | D_INPUT | I_SATURATE);


/*
****************
Датчики скорости
****************
*/
#define SPEED_SENSOR_LEFT_PIN 35
#define SPEED_SENSOR_RIGHT_PIN 34

#define SLOTS_SPEED_DISK 20

// Положение дроселя
int throttle = 0;
/*
**************
Контроллер PS4
**************
*/
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*
************************************
Считывание данных с лидара Dreame D9
************************************
*/
// Определение количества «секторов» (равномерно делим 360° на 12 частей).
#define NUM_SECTORS 12
// Получение данных по UART
#define LIDAR_RX_PIN 16

// Структура пакета лидара: 4 байта заголовка, затем 32 байта данных
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

unsigned long lastTimePulseLeft = 0;

xQueueHandle pulseQueueLeft;




// Пороговые расстояния (в миллиметрах) и время залипания аварии
// ALARM_DIST  — красная зона, WARNING_DIST — жёлтая зона
#define ALARM_DIST 400     // Менее 400 мм -> сектор в красном цвете
#define WARNING_DIST 650   // Менее 650 мм (но >= 400 мм) -> жёлтый
#define ALARM_HOLD_MS 300  // Время (мс), которое сектор будет «залипать» в красном
#define SECTOR_OFFSET 0    // Cдвиг секторов (0..11)



// ГЛОБАЛЬНЫЕ МАССИВЫ
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
  // Вычисление значений для моторов
  processMotor(ctl);
}

void processMotor(ControllerPtr ctl) {
  switch (controlMode) {
    case 0:
      processRunControlMode0(ctl);
      break;
    case 1:
      processRunControlMode1(ctl);
      break;
    case 2:
      processRunControlMode2(ctl);
      break;
      break;
  }
}

void calcSpeedBothMotor(int direction, int baseSpeed) {
  if (abs(baseSpeed) < 30) {
    baseSpeed = 0;
  }

  // Базовые скорости без обратной связи
  int baseLeftSpeed = baseSpeed;
  int baseRightSpeed = baseSpeed;

  if (direction < 0) {
    float popr = (abs(direction) / (float)511 * baseLeftSpeed) / float(3);
    baseLeftSpeed -= (int)popr;
  } else {
    float popr = (abs(direction) / (float)511 * baseRightSpeed) / float(3);
    baseRightSpeed -= (int)popr;
  }

  //motorLeftSpeed = baseLeftSpeed;
  motorRightSpeed = baseRightSpeed;
}

void processRunControlMode0(ControllerPtr ctl) {
  int baseSpeed = map(ctl->throttle() - ctl->brake(), -1023, 1023, -255, 255);
  throttle = baseSpeed;
  int direction = ctl->axisX();
  calcSpeedBothMotor(direction, baseSpeed);
  //Serial.printf("x: %4d, motor1: %4d, motor2:  %4d\n",direction,motorLeftSpeed,motorRightSpeed);
}


void processRunControlMode1(ControllerPtr ctl) {
  int baseSpeed = map(ctl->axisRY(), 512, -511, -255, 255);
  int direction = ctl->axisRX();
  calcSpeedBothMotor(direction, baseSpeed);
  // Serial.printf("x: %4d, motor1: %4d, motor2:  %4d\n",direction,motorLeftSpeed,motorRightSpeed);
}

void processRunControlMode2(ControllerPtr ctl) {
  int baseSpeed = map(ctl->throttle() - ctl->brake(), -1023, 1023, -255, 255);
  int direction = ctl->axisX();
  motorLeftSpeed = baseSpeed;
  motorRightSpeed = baseSpeed;
  //Serial.printf("x: %4d, motor1: %4d, motor2:  %4d\n",direction,motorLeftSpeed,motorRightSpeed);
}


void processSelectControlMode(ControllerPtr ctl) {
  if (ctl->miscSelect()) {
    controlMode++;
    if (controlMode > 2) {
      controlMode = 0;
    }
    Serial.printf("Change control mode: %4d", controlMode);
    for (int i = 0; i < controlMode + 1; i++) {
      ctl->playDualRumble(0, 150, 400, 400);
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
bool waitForHeader(HardwareSerial& ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  // Пытаемся «выровнять» поток байт на заголовок (4 байта)
  while (true) {
    if (ser.available()) {
      uint8_t b = ser.read();
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
bool readBytesWithTimeout(HardwareSerial& ser, uint8_t* buffer, size_t length, uint32_t timeout_ms = 500) {
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
  //Serial.print("SECTORS: ");
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
    //Serial.print(dist);
    if (s < (NUM_SECTORS - 1)) {
      Serial.print(" ");
    }
  }
  Serial.println();

  return true;  // Пакет успешно обработан
}

// // Функция для фиксирования значения скорости датчика за определенный промежуток времени и сброса счетчика
// void readSpeedSensor(long& speed, long& pullseCount) {
//   speed = map(constrain(pullseCount, 0, 15), 0, 15, 0, 255);
//   pullseCount = 0;  // Сбрасываем счетчик
// }


// Класс для фильтрации значений с датчиков по скользящему среднему
template<typename T, uint8_t N>
class AverageWindow {
public:
  float filter(T val) {
    _sum += val;
    if (++_i >= N) _i = 0;
    _sum -= _buffer[_i];
    _buffer[_i] = val;
    return (float)_sum / N;
  }

private:
  T _buffer[N] = {};
  T _sum = 0;
  uint8_t _i = 0;
};


class SSF {
public:
  void init(float val) {
    _y1 = _y2 = val;
  }

  // f_cut - частота среза
  // f_discr - частота дискретизации
  void config(float f_cut, float f_discr) {
    float x = exp(-2.0 * 3.1415 * f_cut / f_discr);
    _b0 = 1.0 - 2.0 * x + x * x;
    _a1 = 2 * x;
    _a2 = -x * x;
  }

  float filter(float val) {
    float y = _b0 * val + _a1 * _y1 + _a2 * _y2;
    _y2 = _y1;
    _y1 = y;
    return y;
  }

private:
  float _b0 = 0, _a1 = 0, _a2 = 0;
  float _y1 = 0, _y2 = 0;
};

template<typename T>
class Median3 {
public:
  T filter(T val) {
    if (++_i >= 3) _i = 0;
    _buf[_i] = val;
    return getMedian(_buf[0], _buf[1], _buf[2]);
  }

  void init(T val) {
    _buf[0] = _buf[1] = _buf[2] = val;
  }

  static inline T getMedian(T a, T b, T c) {
    return (a < b) ? ((b < c) ? b : ((c < a) ? a : c)) : ((a < c) ? a : ((c < b) ? b : c));
  }

private:
  T _buf[3] = {};
  uint8_t _i = 0;
};

// Клас для обработки данных с оптического датчика скорости и вычисления RPM вала
class SpeedSensor {
public:  
  void init() {
    _pulseQueue = xQueueCreate(10, sizeof(unsigned long));  // Очередь на 10 элементов
    // Инициализация фильтров
    _speedLeftSSF.config(4, 1 / 0.05);
    _speedLeftSSF.init(0);
  }

  // Необходимо вызывать в функции прерывания
  void pulseISR() {
    _pulseCount++;
    unsigned long now = micros();
    xQueueSendFromISR(_pulseQueue, &now, NULL);  // Отправляем время в очередь
  }

  float getSpeed() {
    return _speed;
  }

  void handlePulse() {
    // Если прошло много времени с последнего импульса сичтаем что вал не двигается
    if (micros() - _lastTimePulse > _everyZeroSpeed) {
      _speed = 0;
    }

    // Обработка очереди импульсов с датчика
    unsigned long receivedTime;
    if (xQueueReceive(_pulseQueue, &receivedTime, 0) == pdTRUE) {
      // Получили новое время импульса из ISR
      unsigned long currentDelta = receivedTime - _lastTimePulse;
      _lastTimePulse = receivedTime;
      unsigned long filtredDelta = (unsigned long)_speedLeftSSF.filter(_averSpeedLeft.filter(currentDelta));
      _speed = (float)60000000 / (float(filtredDelta) * _countSectors * 2);
    }
  }
private:
  QueueHandle_t _pulseQueue;  // Очередь для обработки прерываний датчика скорости
  long _pulseCount;           // Количество импульсов с
  unsigned long _lastTimePulse = 0; // Время последнего импульса от датчика
  unsigned long _everyZeroSpeed = 50000; // Максимальный период не поступления данных от датчика, чтобы считать что скорость = 0
  int _countSectors = 20; // Количество прорезей в задающем диске
  float _speed = 0; // Скорость вала RPM
  SSF _speedLeftSSF; // Сглаживающий ФНЧ фильтр Эхлера
  AverageWindow<int, 10> _averSpeedLeft; // Скользящее среднее
};

SpeedSensor leftSpeedSensor;
SpeedSensor rightSpeedSensor;

void IRAM_ATTR pulseISRLeft() {
  leftSpeedSensor.pulseISR();
}

void IRAM_ATTR pulseISRRight() {
  rightSpeedSensor.pulseISR();
}

void setup() {
  Serial.begin(BAUDRATE);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Инициализация Serial1 для чтения данных лидара (указать пины RX/TX)
  Serial1.begin(BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, -1);

  // Инициализация датчиков скорости
  pinMode(SPEED_SENSOR_LEFT_PIN, INPUT);
  pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT_PIN), pulseISRLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT_PIN), pulseISRRight, RISING);

  // PID управление моторами
  // Не плохо работающий вариант - 1-0,3-0,1
  // pid.setKp(0.85);
  // pid.setKi(0.05);
  // pid.setKd(0.05);

  // // pid.Kbc = 0.1;
  // pid.setDt(dt);
  // pid.outMax = 255;
  // pid.outMin = 0;

  // pid.setpoint = 0;

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Расскоментировать если будут проблемы с подключением конроллера
  //BP32.forgetBluetoothKeys();

  // Установка плавного изменения скорости для моторов
  motorLeft.smoothMode(1);
  motorRight.smoothMode(1);

  // Установка минимального порога срабатывания моторов
  motorLeft.setMinDuty(77);
  motorRight.setMinDuty(77);

  // Очередь для обработки событи от датчиков скорости
  leftSpeedSensor.init();
  rightSpeedSensor.init();
}

void loop() {
  // Обработка сигналов контроллера
  if (BP32.update())
    processControllers();

  // Необходимо для плавного режима изменения скорости моторов
  motorLeft.tick();
  motorRight.tick();

  // Обработка очереди данных с датчиков скорости
  leftSpeedSensor.handlePulse();
  rightSpeedSensor.handlePulse();

  EVERY_MS(10) {
    Serial.printf("%.4f %.4f\n", leftSpeedSensor.getSpeed(), rightSpeedSensor.getSpeed());
  }

  // Обработка пакетов от лидара
  // parseAndProcessPacket();


  //throttle = 10;

  //throttle = map(throttle,0,255,120,300)

  // if (throttle != 0) {
  //   pid.setpoint = throttle;
  //   motorLeftSpeed = (int)pid.compute(speedLeft);
  // } else {
  //   motorLeftSpeed = 0;
  // }


  // Установка скорости моторов
  motorLeft.setSpeed(throttle);
  motorRight.setSpeed(throttle);
  //motorLeft.setSpeed(motorLeftSpeed);
  //motorRight.setSpeed(motorRightSpeed);

  //Serial.printf("%4d %4d %4d\n", motorLeftSpeed, speedLeft, throttle);

  //Serial.println(delta);
  //Serial.println(pulseCountRight);

  //Serial.printf("m1: %4d, imp1: %4d, m2:  %4d, imp2: %4d\n",motorLeftSpeed,speedLeft,motorRightSpeed,speedRight);
  //Serial.printf("%4d %4d %4d %4d\n",motorLeftSpeed,speedLeft,motorRightSpeed,speedRight);
  //Serial.printf("%4d %4d\n",motorLeftSpeed,speedLeft);
  //Serial.printf("ResultSpeed:%4d CurrentSpeed:%4d Throttle:%4d\n",motorLeftSpeed,speedLeft,map(throttle,0,255,0,16));

  //delay(10);
  vTaskDelay(1);
}