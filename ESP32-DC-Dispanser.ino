#include <Arduino.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include "driver/gpio.h"
#include <EEPROM.h>

#define _sign(x) ((x) > 0 ? 1 : -1)  //Сигнум для смены приращения скорости на уменьшение
// PWM configuration
#define PWM_FREQUENCY 20000  // Frequency in Hz (1kHz)
#define PWM_RESOLUTION 10    // Resolution in bits (0-1023)
#define PWM_CHANNEL 0        // PWM channel (0-15)


#define APH1_pin 32
#define BPH1_pin 33
gpio_num_t APHASE1_PIN = GPIO_NUM_32;
gpio_num_t BPHASE1_PIN = GPIO_NUM_33;

gpio_num_t PWM1_PIN = GPIO_NUM_19;
gpio_num_t IN1_PIN = GPIO_NUM_27;
gpio_num_t IN2_PIN = GPIO_NUM_14;
#define SPEED_1 19
#define IN_1 27
#define IN_2 14


#define APH2_pin 25
#define BPH2_pin 26
gpio_num_t APHASE2_PIN = GPIO_NUM_25;
gpio_num_t BPHASE2_PIN = GPIO_NUM_26;

gpio_num_t PWM2_PIN = GPIO_NUM_21;
gpio_num_t IN3_PIN = GPIO_NUM_12;
gpio_num_t IN4_PIN = GPIO_NUM_13;
#define SPEED_2 21
#define IN_3 12
#define IN_4 13

gpio_num_t LED = GPIO_NUM_2;

volatile long enc1;

BluetoothSerial SerialBT;

// Stop button is attached to PIN 0 (IO0)
#define BTN_STOP_ALARM 0

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  static int flag;
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  flag = !flag;

  gpio_set_level(LED, flag);
  //Serial.println(flag);
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}
static const int spiClk = 1000000;  // 1 MHz
//uninitalised pointers to SPI objects
SPIClass *vspi = NULL;

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
const bool level = 1;  //направление для всех пинов

int Dduty = 0;


int _accel = 30000;    //ускорение в отсчётах энкодера в секунду
int _maxSpeed = 8344;  //максимальная скорость в отсчётах энкодера в секунду

const uint8_t _dt = 15;          //Временной шаг вызова функции регулирования
float _dts = (float)_dt / 1000;  //Временной шаг для подсчёта в единицах в секунду теор. значений скоростей V и позиций pos
uint32_t _tmr2 = 0;              //Переменная таймера comp_cur_pos для подсчёта значений V и pos
long _targetPos = 0;             //Целевое положение в отсчётах энкодера
int dir = -1;                    //Направление 1 - вперёд // -1 - назад

long controlPos = 0;     //Теоретическое положение, идущее в PID
float controlSpeed = 0;  //Теоретическая скорость на шаге

int _minDuty = 0, _maxDuty = 1023;  //Минимальное значение страгивания ДПТ: (_minDuty-255)
float _k = 1.0;
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
//////////ПЕРЕМЕННЫЕ ЭНКОДЕРОВ И РЕАЛЬНОЙ СКОРОСТИ/////////

volatile int lastEncoded1 = 0;    // Here updated value of encoder store.
volatile long encoderValue1 = 0;  // Raw encoder value
volatile long encREF1 = 0;        //сбрасываемое значениеэнкодера для скорости

volatile int lastEncoded2 = 0;    // Here updated value of encoder store.
volatile long encoderValue2 = 0;  // Raw encoder value
volatile long encREF2 = 0;        //сбрасываемое значениеэнкодера для скорости

volatile long encREF1N = 0;  // Raw encoder value
volatile long encREF2N = 0;

int32_t encREF1copy = 0;
int32_t encREF2copy = 0;
float Velocity1 = 0.0;  //Переменная мгновенной скорости 1
float Velocity2 = 0.0;  //Переменная мгновенной скорости 2

float Velocity1N = 0.0;  //Переменная мгновенной скорости 1
float Velocity2N = 0.0;  //Переменная мгновенной скорости 2
int32_t encREF1copyN = 0;
int32_t encREF2copyN = 0;
//uint8_t freq1 = 10; // частота опроса

uint8_t per1 = 50;  ///freq1; //период опроса скоростей 1000/60 = 17 мс
uint32_t t1 = 0;    //начальный момент времени unsigned long
uint32_t t2 = 0;    //unsigned long 4

int _buf[3];
byte _count = 0;
float _middle_f = 0;

float T_Theoretic = 0;

int ratio = 8344;  //8433 = 28*298
///////////////////////////////////////////////////////////////////////////
////////////////////////////////ПИД РЕГУЛЯТОР//////////////////////////////
int PWM1 = 0, PWM2 = 0;
long tpid = 0;

long _previnput = 0, _previnput2 = 0;
float integral = 0.0000;
float integral2 = 0.0000;

bool cutoff = 0;
int stopzone = 10, stopzone2 = 10;


float kp = 0.55;  // 2x: 0.35;  4x: 0.085		// (знач. по умолчанию)0.1 0.5 0.05
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
double ki = 0.00000005;  //2x: 0.000005 4x: 0.00000011 0.00000015 для высок.
// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
float kd = 0.35;  //2x: 35 - 2 прерывания 4x: 0.75 7.75 для высок скоростей

float kpVel = 0.55;  // 2x: 0.35;  4x: 0.085		// (знач. по умолчанию)0.1 0.5 0.05
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
double kiVel = 0.00000005;  //2x: 0.000005 4x: 0.00000011 0.00000015 для высок.
// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
float kdVel = 0.35;  //2x: 35 - 2 прерывания 4x: 0.75 7.75 для высок скоростей
//////////////////////////////////////////////////////////////////////

////////////////Альтернативная "АНАЛИТИЧЕСКАЯ" кривая/////////////////
float ta = 0;  //конечный момент времени
float tb = 0;  //время окончания разгона
float tc = 0;  //время окончания равномерного движения

float xb = 0;
float xc = 0;
float curTime = 0;
long controlPos2 = 0;
int N = 0;  // Число шагов дискретизации

int8_t curve = 1;  //Тип кривой. -1 - пересечение парабол 2 - нормальный случай, 1 - только ускорение и равномерное

long timer1 = 0;  //ControlPos2 таймер
long f = 0;       //счётчик приращения

/////////////////////////////////////////////////////////////////////

//////////////////////////////Сбор данных////////////////////////////
uint8_t marker = 0;  //маркер прекращения
#define arr_sizePos1 40
#define arr_sizePos2 4
long EncStat1[arr_sizePos1][arr_sizePos2];
long EncStat2[arr_sizePos1][arr_sizePos2];
unsigned long del = 0;  //Обнуление таймера микрос

long tmrexp = 0;  //таймер цикла сбора

uint16_t cnn = 0, cnn1 = 0, cnn2 = 0;  //Счётчики вызовов

int16_t du = 0, du1 = 0;
/////////////////////////////////////////////////////////////////////

bool offtrig = 0;

bool PrintDataFlag = 1;

double rkp = 0.010000;
uint32_t reduceFlag;
int velfl = 0;
float rconst;
int defaultPWM = _minDuty;
int defaultPWMFlag;
int prevPWM;

void setup() {

  Serial.begin(115200);
  SerialBT.begin("ESP Test");

  while (!Serial)
    ;

  // Set BTN_STOP_ALARM to input mode
  pinMode(BTN_STOP_ALARM, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  pinMode(APHASE1_PIN, INPUT_PULLDOWN);
  pinMode(BPHASE1_PIN, INPUT_PULLDOWN);
  pinMode(APHASE2_PIN, INPUT_PULLDOWN);
  pinMode(BPHASE2_PIN, INPUT_PULLDOWN);

  pinMode(SPEED_1, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  pinMode(SPEED_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 20Mhz
  timer = timerBegin(20000000);
  //Serial.println(timer);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 20000000, true, 0);
  attachInterrupt(APH2_pin, pin_A2_ISR, CHANGE);
  attachInterrupt(BPH2_pin, pin_B2_ISR, CHANGE);
  attachInterrupt(APH1_pin, pin_A1_ISR, CHANGE);
  attachInterrupt(BPH1_pin, pin_B1_ISR, CHANGE);

  // Attach the PWM channel to the chosen motor pin
  ledcAttach(PWM2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  delay(1000);
  Calculus(_accel, _maxSpeed, _targetPos, _dts);

  setRatio(8344, 4);
  setObor(0);
  //setMillimeters(0);
  setSpeedMMS(1);
  setMinDuty(250);
  _tmr2 = tpid = millis();
  static long controlPos_Saved;
  EEPROM.get(0, controlPos_Saved);
  controlPos = _targetPos = controlPos_Saved;
}

int8_t fl = 1;

void loop() {

  inputData();
  comp_cur_pos();
  if (digitalRead(BTN_STOP_ALARM) == LOW) {
    // If timer is still running
    if (timer) {
      // Stop and free timer
      timerEnd(timer);
      timer = NULL;
    }
  }
  prevPWM = PWM2;

  //CommunicationBT();

  static uint32_t tim0;
  if (millis() - tpid >= _dt) {

    comp_cur_pos();
    Velocities(_dt);
    velfl = 1;
    if (PrintDataFlag == 1) {
      //POSITIONS();
      VELS();
    }
    PWM2 = PIDcalc(controlPos, encoderValue2, kp, ki, kd, _dt, dir, 0, offtrig);
    tpid = millis();
  }

  static uint32_t tim2;
  bool thisDir = (controlSpeed * controlSpeed / _accel / 2.0 >= abs(_targetPos - encoderValue2));

  if (millis() - tim2 >= 7) {

    VelocitiesN();
    //Serial.println("Velo: " + String(Velocity2N));

    if (abs(Velocity2N) >= _maxSpeed - 25 && defaultPWMFlag == 0) {  // Фиксация значения нормльного PWM

      defaultPWMFlag = 1;
      defaultPWM = PWM2;
      Serial.println("DEFAULT PWM SET: " + String(defaultPWM));
      Serial.println("DEFAULT Velocity2N: " + String(Velocity2N));
    }

    static long adjDefPWMtimer;
    if (abs(Velocity2N) > _maxSpeed && abs(prevPWM) <= abs(defaultPWM)) {

      if (millis() - adjDefPWMtimer >= 100) {
        defaultPWM = constrain(abs(defaultPWM) - ((_maxDuty / 100) * (_dt * 2 / 10)), _minDuty, _maxDuty);
        if (_maxSpeed < 0) {

          defaultPWM = defaultPWM * -1;
        }
        Serial.println("DefPWM changed: " + String(defaultPWM));
        Serial.println("Prev PWm: " + String(prevPWM));
        Serial.println("Velo: " + String(Velocity2N));
        adjDefPWMtimer = millis();
      }
    }

    if (abs(Velocity2N) > _maxSpeed && thisDir == false) {  //Оперативная редукция ШИМ
      if (rconst != 0) {
        static long deltaV = abs(Velocity2N - _maxSpeed);
        //Serial.println("CORR: " + String(Velocity2N));
        if (_maxSpeed > 0) {

          Serial.println("BEF: " + String(PWM2));
          Serial.println("Vabs " + String(abs(Velocity2N)));
          Serial.println("V " + String(Velocity2N));
          Serial.println("max " + String(_maxSpeed));
          Serial.println("encREF " + String(encREF2copyN));

          PWM2 = PWM2 - (rconst * deltaV);
          if (PWM2 < defaultPWM) {
            PWM2 = defaultPWM;
            //Serial.println("default PWM:" + String(defaultPWM));
          }

          Serial.println("AFT: " + String(PWM2));
          Serial.println("Delta V " + String(deltaV));

        } else {

          Serial.println("BEF: " + String(PWM2));
          PWM2 = PWM2 + (rconst * deltaV);
          if (PWM2 > defaultPWM) {
            PWM2 = defaultPWM;
            //Serial.println("default PWM:" + String(defaultPWM));
          }
          Serial.println("AFT: " + String(PWM2));
        }
      }
    }

    tim2 = millis();
    if (abs(PWM2) >= abs(defaultPWM) * 1.2 && abs(Velocity2N) >= _maxSpeed - 25 && rconst != 0 && defaultPWM >= _minDuty) {  //Отсечка увеличенного ШИМ, если скорость выше нужной

      PWM2 = defaultPWM;
    }

    movement(PWM2, 2);
  }


  static uint32_t tim1;
  if (millis() - tim1 >= 4) {

    // Serial.println(PWM2);
    tim1 = millis();
  }
  //Serial.println("real PWM: " + String(prevPWM));
}

void CommunicationBT() {

  if (SerialBT.available() > 0) {

    String dannie = "";

    while (SerialBT.available()) {

      while (SerialBT.available()) {

        dannie = dannie + SerialBT.readString();
      }
    }

    dannie.trim();
    //Serial.println("");
    Serial.println(dannie);

    if (dannie.indexOf("sp") != -1) {

      uint8_t sppos = dannie.indexOf("sp") + 2;
      String speedrec = dannie.substring(sppos, sppos + 3);
      int receivedspeed = constrain(speedrec.toInt(), 1, 100);
      Serial.println("received speed: " + String(receivedspeed));
      SerialBT.println("received speed: " + String(receivedspeed));
      //recalculation(1, receivedspeed);
    }


    if (dannie.indexOf("1") != -1) {
      SerialBT.println("One received");
      Serial.println("One received");
    } else {
      SerialBT.println("Another command");
      Serial.println("Another command");
    }
  }
}

void pin_A2_ISR() {

  if (!(gpio_get_level(APHASE2_PIN)) == !(gpio_get_level(BPHASE2_PIN))) {  //2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    //enc1++;
    encoderValue2++;
    encREF2++;
    encREF2N++;
  } else {

    //enc1--;
    encoderValue2--;
    encREF2--;
    encREF2N--;
  }
  //enc1++;
}

void pin_B2_ISR() {
  if (!(gpio_get_level(APHASE2_PIN)) == !(gpio_get_level(BPHASE2_PIN))) {  // 2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    //enc1--;
    encoderValue2--;
    encREF2--;
    encREF2N--;
  } else {

    //enc1++;
    encoderValue2++;
    encREF2++;
    encREF2N++;
  }
  //enc1++;
}

void pin_A1_ISR() {

  if (!(gpio_get_level(APHASE1_PIN)) == !(gpio_get_level(BPHASE1_PIN))) {  //2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    //enc1++;
    encoderValue1++;
    encREF1++;
    encREF1N++;
  } else {

    //enc1--;
    encoderValue1--;
    encREF1--;
    encREF1N--;
  }
  //enc1++;
}

void pin_B1_ISR() {
  if (!(gpio_get_level(APHASE1_PIN)) == !(gpio_get_level(BPHASE1_PIN))) {  // 2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    //enc1--;
    encoderValue1--;
    encREF1--;
    encREF1N--;
  } else {

    //enc1++;
    encoderValue1++;
    encREF1++;
    encREF1N++;
  }
  //enc1++;
}

void comp_cur_pos() {  // USes timpos & _tmr2  (long encoder, uint8_t motor)

  unsigned long timpos = millis();

  if (timpos - _tmr2 >= _dt) {
    _dts = (_dt) / 1000.0;  //(timpos - _tmr2) / 1000.0;
    _tmr2 = millis();
    long err = _targetPos - controlPos;  // "ошибка" позиции
    if (err != 0) {
      if (_accel != 0) {

        bool thisDir = (controlSpeed * controlSpeed / _accel / 2.0 >= abs(err));  // пора тормозить (false до приближения к параболе торможения обр. от ускор.)
        controlSpeed += _accel * _dts * (thisDir ? -_sign(controlSpeed) : _sign(err));
        //Serial.println("Cont SPEED VAR 1: " + String(controlSpeed));

      } else {
        controlSpeed = err / _dts;  // профиль постоянной скорости
        //Serial.println("Cont SPEED VAR 2: " + String(controlSpeed));
      }
      controlSpeed = constrain(controlSpeed, -_maxSpeed, _maxSpeed);
      controlPos += controlSpeed * _dts;
      controlPos = constrain(controlPos, -150000, 150000);
    }
    //return controlPos;
    //Serial.print(encoderValue1);
    //Serial.print(' ');
    // Serial.print(Velocity1);
    // Serial.print(' ');
    // Serial.println();
  }
}



float rkpmax = 0.00;
int PIDcalc(long setPoint, long current, float kp, float ki, float kd, float DT, int dir, bool cutoff, bool off) {  //ПИД для 1 первого мотора

  //if (!off) {
  static long prevVelocity;
  long velocity = Velocity2;

  if (dir == -1) {

    setPoint = -setPoint;
  }

  float Duty = 0;
  long err1 = setPoint - current;
  long deltainput = _previnput - err1;
  _previnput = err1;
  Duty = (float)(err1 * kp);
  integral += (float)err1 * ki * DT;
  Duty += (float)deltainput * kd / DT + integral;

  if (cutoff) {  // отсечка (для режимов позиции)
    if (abs(err1) < stopzone) {
      integral = 0;
      Duty = 0;
      // Serial.println("null");
    }
  }

  Duty = constrain(Duty, -_maxDuty, _maxDuty);

  int PWMnew;
  PWMnew = Duty;
  static bool flag5;

  if (abs(velocity) > (abs(_maxSpeed)) && velfl == 1 && rkpmax != 0) {

    long errVel = abs(_maxSpeed) - abs(velocity);
    float deltaVel = velocity - prevVelocity;

    int deltaPWM = PWM2 - prevPWM;  //rkp * errVel
    //Serial.println("");
    //Serial.println("DelPWM: " + String(deltaPWM));
    //Serial.println("DeltaVel: " + String(deltaVel));



    PWMnew = PWM2;
    prevPWM = PWM2;           //Сохранение предыдущего значения ШИМ
    prevVelocity = velocity;  //Сохранение предыдущего значения скорости
    //if (abs(velocity1) > abs(controlspeed)) {
    if (rkp != 0.0) {
      flag5 = 1;
      //Serial.println("Bef: " + String(PWMnew));
      PWMnew = constrain(abs(PWMnew) + rkp * errVel, defaultPWM, PWM2);
      //Serial.println("Reducing: ");
      //Serial.print(rkp, 6);
      //Serial.println();
    }

    if (_maxSpeed < 0) {
      PWMnew = -1 * abs(PWMnew);
      defaultPWM = -1 * abs(defaultPWM);
    }

    //Serial.println("NEWPWM:" + String(PWMnew));

    reduceFlag++;
    velfl = 0;
    flag5 = 1;
    //return int(PWMnew);
  }


  prevVelocity = velocity;
  prevPWM = PWM2;
  velfl = 0;
  if (flag5 == 1) {
    //Serial.println("Aft: " + String(PWMnew));
    flag5 = 0;
  }

  flag5 = 0;
  Serial.println("Normal: " + String(PWMnew));

  return int(PWMnew);
  //Serial.println(Duty);
  //}
  // if (Duty == 0) {Serial.println("st");}
  //return Dduty;
}

int reduce(int PWM, long controlspeed, long velocity) {

  long errVel = controlspeed - velocity;
  static long prevVelocity;
  static long prevPWM;

  if (PWM != prevPWM && velocity != prevVelocity) {
    rkp = abs(velocity - prevVelocity) / abs(PWM - prevPWM);
  }

  prevPWM = PWM;            //Сохранение предыдущего значения ШИМ
  prevVelocity = velocity;  //Сохранение предыдущего значения скорости

  int PWMnew = PWM;
  //if (abs(velocity1) > abs(controlspeed)) {

  int err = abs(velocity - controlspeed);
  PWMnew = PWMnew + rkp * errVel;
  //}
  return PWMnew;
}

void movement(int _Duty, int motor) {  //Приведение ШИМ к минимальному. Передача сигнала

  if (motor == 1) {}
  Dduty = _Duty;


  if (Dduty > 0) {
    //Вращение при регулировании в направлении вперёд по умолчанию
    if (_minDuty != 0) {

      Dduty = Dduty * _k + _minDuty;


    }  // сжимаем диапазон
    //Serial.println(_duty);
  } else {
    // Вращение в обратном напра
    if (_minDuty != 0) { Dduty = Dduty * _k - _minDuty; }  // сжимаем диапазон
  }

  int PWM = abs(Dduty);
  //Serial.println(",");
  //Serial.println("PWMMOVE" + String(PWM));

  bool stopper = 0;
  bool stopper2 = 0;

  if ((abs(encoderValue1) + stopzone > _targetPos) && abs(_Duty) < 12) {
    //if (encoderValue1 > _targetPos + stopzone) {stopper = 1;}
    digitalWrite(IN_1, 1);
    digitalWrite(IN_2, 1);
    //analogWrite(SPEED_1, 255);
    ledcWrite(SPEED_1, _maxDuty);
    //Serial.println(Dduty);
    integral = 0;
    stopper = 1;
    // offtrig = 1;

    //delay(100);

  } else {
    stopper = 0;
  }

  if ((abs(encoderValue2) + stopzone2 > _targetPos) && abs(_Duty) < 12) {

    digitalWrite(IN_3, 1);
    digitalWrite(IN_4, 1);
    // analogWrite(SPEED_2, 255);
    ledcWrite(SPEED_2, _maxDuty);
    //Serial.println("dt" + String(_Duty));
    //Serial.print("motor" + String(motor));
    integral2 = 0;
    stopper2 = 1;
    //delay(100);

  } else {
    stopper2 = 0;
  }

  //if (_targetPos + 100 < encoderValue1) {stopper = 0;}

  if (motor == 1 && stopper != 1) {  //Для первого мотора

    if (Dduty > 0) {

      digitalWrite(IN_1, level);
      digitalWrite(IN_2, !level);
      //analogWrite(SPEED_1, PWM);
      ledcWrite(SPEED_1, PWM);
      //Serial.println(PWM);

    } else {

      digitalWrite(IN_1, !level);
      digitalWrite(IN_2, level);
      analogWrite(SPEED_1, PWM);
      ledcWrite(SPEED_1, PWM);
    }

  } else if (motor == 2 && stopper2 != 1) {  //Для второго мотора

    if (Dduty > 0) {

      digitalWrite(IN_3, level);
      digitalWrite(IN_4, !level);
      //analogWrite(SPEED_2, PWM);
      ledcWrite(SPEED_2, PWM);

    } else {

      digitalWrite(IN_3, !level);
      digitalWrite(IN_4, level);
      //analogWrite(SPEED_2, PWM);
      ledcWrite(SPEED_2, PWM);

    }  // else if (Dduty == 0) {

    //   digitalWrite(IN_3, level);
    //   digitalWrite(IN_4, level);
    //   analogWrite(SPEED_2, 128);
    // }
  }
}

////////////////////////////////////////////////////////////////////////
/////////////////////////////СЕТЕРЫ SET/////////////////////////////////

void setMillimeters(float millimeters) {

  if (millimeters < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  _targetPos = (float)((millimeters * (float)ratio) / 1.0);  //1.0 - шаг винта.
  Serial.println("_targetPos: " + String(_targetPos));
  //_targetPos = abs(_targetPos);
}

void setSpeedMMS(float millimetersSec) {

  _maxSpeed = millimetersSec * ratio;
  constrain(_maxSpeed, 0, 25000);
  Serial.println("_maxSpeed: " + String(_maxSpeed));
}

void setTarget(long TargetPos) {

  if (TargetPos < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = abs(TargetPos);
}

void setObor(float ob) {

  dir = 1;
  if (ob < 0) {  //Инверсия направления для отрицательной позиции
    //dir = -1;
    Serial.println("Revers");
  }

  Serial.println("Oborotov: " + String(ob));
  _targetPos = (round(ob * ratio));  //ratio - число тиков на оборот
  Serial.println("_targetPos: " + String(_targetPos));

  //_targetPos = abs(_targetPos);  //ratio - число тиков на оборот
}



void setDeg(long Deg) {

  if (Deg < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("Deg: " + String(Deg));
  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = Deg * ratio / 360;
}

void setRatio(uint16_t _ratio, uint8_t precision) {
  ratio = _ratio / (4 / precision);
  _maxSpeed = _maxSpeed / (4 / precision);
  _accel = _accel / (4 / precision);
}

void setMinDuty(int duty) {

  _minDuty = duty;
  _k = 1.0 - (float)_minDuty / _maxDuty;
}

void Calculus(int acceleration, int V, long targetPos, float dts) {  //Подсчитывает моменты времени и

  float Xta = (float)targetPos;  //Функция координаты от времени для t;
  float dti = dts;
  float Vmax = (float)V;
  float accel = acceleration;

  tb = Vmax / accel;  //момент времени завершения разгона
  tc = Xta / (Vmax);  //Момент времени завершения равномерного движения

  xb = (float)(Vmax * Vmax) / (2 * accel);  //координата завершения разгона

  float taorig = 0.0;


  if (_accel != 0) {

    if (xb < _targetPos / 2) {  //Если возможна кривая ускорения, равномерного движения и замедления

      taorig = (float)(Xta / Vmax + Vmax / accel);  //Завершающий момент времени 3 сегмента
      curve = 2;                                    //3 сегмента кривой

    } else if (xb > _targetPos / 2 && xb < _targetPos) {  //Ускорение и равномерное движение

      taorig = _targetPos / Vmax + tb / 2;
      tc = taorig;
      curve = 1;

    } else if (xb == _targetPos / 2) {  //Если возможно только ускорение и замедление

      taorig = (float)(2 * (Vmax / accel));
      curve = 0;

    } else if (xb >= _targetPos) {  //Если возможно только ускорение

      taorig = sqrt(2 * _targetPos / accel);
      curve = -1;
    }

  } else {

    taorig = _targetPos / Vmax;
  }

  ta = (int)(round(taorig * 100));
  ta = (float)ta / 100;

  N = round(ta / dti);

  Serial.println("Время ta_Calculus: " + String(ta));
  Serial.println("N: " + String(N));
  Serial.println("Время ускорения tb: " + String(tb));
  Serial.println("Позиция после ускорения xb: " + String(xb));

  //T = (0:dti:(ta+dti));
  //X = zeros(N+2, 1);
  Serial.println("Время начала торможения tc: " + String(tc));
  Serial.println("Curve: " + String(curve));
}

///////////////////////////////////////////////////////////////
/////////////////////СБОР ДАННЫХ И ВЫВОДЫ//////////////////////

bool posflag = 1;
void POSITIONS() {  //Вывод позиций столбцами

  if (posflag == 1) {

    Serial.println();
    //Serial.println(_targetPos);
    Serial.print(controlPos);
    //Serial.print(Velocity1);
    //Serial.print(_duty);
    //Serial.print(controlPos);
    //Serial.print(", ");
    //Serial.print(abs(encoderValue1));
    //Serial.print(Velocity1);
    //Serial.print(_duty);
    //Serial.print(controlPos);
    Serial.print(", ");

    //Serial.print(_duty2);
    Serial.print(abs(encoderValue2));

    // Serial.print(", ");
    // Serial.print(abs(integral));

    // Serial.print(", ");
    // Serial.print(abs(integral2));


    //Serial.print(controlSpeed/1000);
    //Serial.print(", ");
    //Serial.print(Velocity2);
  }
}

void PWMPORT() {

  Serial.println();

  Serial.print(", ");
  Serial.print(PWM1);

  Serial.print(", ");
  Serial.print(PWM2);

  Serial.print(", ");
  Serial.print(PWM1);
}

void VELS() {  //Вывод скоростей  столбцами

  Serial.println();
  //Serial.println(_targetPos);
  Serial.print(controlSpeed);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");
  Serial.print(Velocity2);
  //Serial.print(Velocity1);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");

  //Serial.print(_duty2);
  Serial.print(Velocity2N);
  //Serial.print(controlSpeed/1000);
  //Serial.print(", ");
  //Serial.print(Velocity2);
}

////////////////////ОБРАБОТКА ВХОДА//////////////////

void inputData() {

  if (Serial.available() > 0) {  //если есть доступные данные

    //Serial.println("Serial.available()");
    char buffer[] = { "" };
    String dannie = "";
    int j = 0;

    while (Serial.available()) {

      while (Serial.available()) {
        dannie = dannie + Serial.readString();
      }
      //   if (Serial.readBytes(buffer, 1)) {
      //     //Serial.print("I received: ");
      //     //Serial.println(buffer[0]);
      //     dannie = dannie + buffer[0];
      //   }
      //   // j++;
      //   // Serial.print(j);
      //   // Serial.print(" :  ");
      //   //Serial.println(buffer[0]);
    }

    dannie.trim();
    //Serial.println("");
    Serial.println("Received: " + dannie);

    if (dannie.indexOf("vel") != -1) {

      uint8_t vpos = dannie.indexOf("vel") + 3;
      String vel = dannie.substring(vpos, vpos + 6);
      int receivedvelocity = constrain(vel.toInt(), 0, 17000);
      Serial.println("received velocity: " + String(receivedvelocity));
      //recalculation(1, receivedspeed);
      _maxSpeed = receivedvelocity;
      defaultPWMFlag = 0;
    }

    if (dannie.indexOf("velMMS") != -1) {

      uint8_t velMMSpos = dannie.indexOf("velMMS") + 6;
      String velMMS = dannie.substring(velMMSpos, velMMSpos + 6);
      float receivedvelMMS = constrain(velMMS.toFloat(), 0, 3.5);
      Serial.println("received velocity in mm/s: " + String(receivedvelMMS));
      //recalculation(1, receivedspeed);
      setSpeedMMS(receivedvelMMS);
      defaultPWMFlag = 0;
    }

    if (dannie.indexOf("tpos") != -1) {

      uint8_t tppos = dannie.indexOf("tpos") + 4;
      String tposrec = dannie.substring(tppos);
      long receivedtpos = constrain(tposrec.toInt(), -250000, 250000);
      Serial.println("received target position: " + String(receivedtpos));
      _targetPos = receivedtpos;
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    if (dannie.indexOf("acc") != -1) {

      uint8_t accpos = dannie.indexOf("acc") + 3;
      String accrec = dannie.substring(accpos, accpos + 6);
      int receivedacc = constrain(accrec.toInt(), 0, 40000);
      Serial.println("received acceleration: " + String(receivedacc));
      _accel = receivedacc;
    }

    if (dannie.indexOf("rk") != -1) {

      uint8_t rkpos = dannie.indexOf("rk") + 2;
      String rkrec = dannie.substring(rkpos, rkpos + 6);
      float receivedrk = constrain(rkrec.toFloat(), 0, 100);
      rkpmax = receivedrk;
      Serial.println("received REDUCTION rk: " + String(rkpmax));
    }

    if (dannie.indexOf("rconst") != -1) {

      uint8_t rconstpos = dannie.indexOf("rconst") + 6;
      String rconstrec = dannie.substring(rconstpos, rconstpos + 6);
      float receivedrconst = constrain(rconstrec.toFloat(), 0, 100);
      rconst = receivedrconst;
      Serial.println("received REDUCTION CONST: " + String(rconst));
    }

    if (dannie.indexOf("kp") != -1) {

      uint8_t kppos = dannie.indexOf("kp") + 2;
      String kprec = dannie.substring(kppos, kppos + 6);
      float receivedkp = constrain(kprec.toFloat(), 0, 100);
      Serial.println("received PID kp: " + String(receivedkp));
      kp = receivedkp;
    }

    if (dannie.indexOf("ki") != -1) {

      uint8_t kipos = dannie.indexOf("ki") + 2;
      String kirec = dannie.substring(kipos, kipos + 8);
      double receivedki = constrain(kirec.toDouble(), 0.0, 100.0);
      Serial.println("received PID ki: " + String(receivedki));
      ki = receivedki;
    }

    if (dannie.indexOf("kd") != -1) {

      uint8_t kdpos = dannie.indexOf("kd") + 2;
      String kdrec = dannie.substring(kdpos, kdpos + 6);
      double receivedkd = constrain(kdrec.toDouble(), 0.0, 100.0);
      Serial.println("received PID kd: " + String(receivedkd));
      kd = receivedkd;
    }

    if (dannie.indexOf("minDuty") != -1) {

      uint8_t minDutypos = dannie.indexOf("minDuty") + 7;
      String minDutyrec = dannie.substring(minDutypos, minDutypos + 6);
      int receivedminDuty = constrain(minDutyrec.toInt(), 0, _maxDuty);
      Serial.println("received PID MIN Duty: " + String(receivedminDuty));
      setMinDuty(receivedminDuty);
    }

    if (dannie.indexOf("maxDuty") != -1) {

      uint8_t maxDutypos = dannie.indexOf("maxDuty") + 7;
      String maxDutyrec = dannie.substring(maxDutypos, maxDutypos + 6);
      long receivedmaxDuty = constrain(maxDutyrec.toInt(), 0, 1000000);
      Serial.println("received PID MAX Duty: " + String(receivedmaxDuty));
      _maxDuty = receivedmaxDuty;
    }

    if (dannie.indexOf("Z") != -1) {

      uint8_t Zpos = dannie.indexOf("Z") + 1;
      String Zrec = dannie.substring(Zpos, Zpos + 6);
      long receivedZ = constrain(Zrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in mm: " + String(receivedZ));
      _targetPos += receivedZ * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    if (dannie.indexOf("Zob") != -1) {

      uint8_t Zobpos = dannie.indexOf("Zob") + 3;
      String Zobrec = dannie.substring(Zobpos, Zobpos + 6);
      long receivedZob = constrain(Zobrec.toInt(), -200, 200);
      Serial.println("received piston pusher Z shift in turns: " + String(receivedZob));
      _targetPos += receivedZob * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    if (dannie.indexOf("Ztick") != -1) {

      uint8_t Ztickpos = dannie.indexOf("Ztick") + 5;
      String Ztickrec = dannie.substring(Ztickpos, Ztickpos + 6);
      long receivedZtick = constrain(Ztickrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in ticks: " + String(receivedZtick));
      _targetPos += receivedZtick;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    if (dannie.indexOf("tposMMS") != -1) {

      uint8_t tposMMSpos = dannie.indexOf("tposMMS") + 7;
      String tposMMSrec = dannie.substring(tposMMSpos, tposMMSpos + 7);
      float receivedtposMMS = constrain(tposMMSrec.toFloat(), -200, 200);
      Serial.println("received target position in mm: " + String(receivedtposMMS));
      setMillimeters(receivedtposMMS);
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    if (dannie.indexOf("tposob") != -1) {

      uint8_t tposobpos = dannie.indexOf("tposob") + 6;
      String tposobrec = dannie.substring(tposobpos, tposobpos + 6);
      float receivedtposob = constrain(tposobrec.toFloat(), -200, 200);
      Serial.println("received target position in mm: " + String(receivedtposob));
      setObor(receivedtposob);
      defaultPWMFlag = 0;
      defaultPWM = 0;
    }

    volatile long bauds[] = { 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 250000, 460800, 500000 };

    if (dannie.indexOf("COM") != -1) {

      bool supported = false;

      uint8_t COMBaudpos = dannie.indexOf("COM") + 3;
      PrintDataFlag = 0;
      Serial.println("/////Stop Data sending/////");
      long rec = dannie.substring(COMBaudpos).toInt();
      Serial.println("COM baud REC: " + String(rec));
      long NewBaud = constrain(rec, 1200, 500000);

      for (int i = 0; i < sizeof(bauds) / 4; i++) {
        Serial.println(bauds[i]);
        if (bauds[i] == NewBaud) {

          supported = true;
          Serial.println("Supported baud: " + String(bauds[i]));
          break;

        } else {
          supported = false;
        }
      }

      if (supported == false) {

        Serial.println("Invalid/Insupported baud");
        Serial.println("NewBaud Is: " + String(NewBaud));
        NewBaud = 115200;
      }

      Serial.println("NEW BAUD: " + String(NewBaud));

      if (NewBaud >= 1200) {

        Serial.end();
        delay(1500);

        Serial.begin(NewBaud);
        delay(1000);
        Serial.println("Successfully applied baud: " + String(NewBaud));
      }
    }

    if (dannie.indexOf("PD") != -1) {

      Serial.println("PD");
      PrintDataFlag = !PrintDataFlag;

      if (PrintDataFlag == 1) {

        Serial.println("///////////Data:///////////");

      } else {

        Serial.println("/////Stop Data sending/////");
      }
    }
    //Serial.println(dannie);
  }
}

void Velocities(int period) {  //Функция определения скорости моторов

  // if (millis() - t1 >= period) {

  //   //8344 = 28 * 298 импульсов на оборот
  // t1 = millis();

  //noInterrupts();
  encREF1copy = encREF1;  //int16_t
  encREF2copy = encREF2;  //int16_t
  encREF1 = encREF2 = 0;
  //interrupts();
  Velocity1 = (float)encREF1copy * 1000.0 / (float)period;
  //Velocity1 = filter(Velocity1);
  Velocity2 = (float)encREF2copy * 1000.0 / (float)period;
  //Velocity1 = filter(Velocity1);
  //Velocity2 = filter(Velocity2);
  //Serial.println(Velocity1);
  //Serial.println(encREF1copy);
  //controlPosFl = (float)controlPos/ratio;
  //Serial.println(encoderValue2);
  //}
}

void VelocitiesN() {  //Функция определения скорости моторов

  static long periodF;
  static long time;
  periodF = (micros() - time)/1000; 
  time = micros();

  encREF1copyN = encREF1N;  //int16_t
  encREF2copyN = encREF2N;  //int16_t
  encREF1N = encREF2N = 0;
  
  Velocity1N = (float)encREF1copyN * 1000.0 / (float)periodF;
  Velocity2N = (float)encREF2copyN * 1000.0 / (float)periodF;
}

int filter(int velocity) {
  _buf[_count] = velocity;
  if (++_count >= 3) _count = 0;
  int middle = 0;
  if ((_buf[0] <= _buf[1]) && (_buf[0] <= _buf[2])) {
    middle = (_buf[1] <= _buf[2]) ? _buf[1] : _buf[2];
  } else {
    if ((_buf[1] <= _buf[0]) && (_buf[1] <= _buf[2])) {
      middle = (_buf[0] <= _buf[2]) ? _buf[0] : _buf[2];
    } else {
      middle = (_buf[0] <= _buf[1]) ? _buf[0] : _buf[1];
    }
  }
  _middle_f += (middle - _middle_f);
  return _middle_f;
}
