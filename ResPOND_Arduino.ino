#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define TCA_ADDR 0x70
#define SENSOR_COUNT 3
const uint8_t tcaChannels[SENSOR_COUNT] = {2, 3, 4};
#define WAVE_CHANNEL 7
#define VIBRATION_PIN A0
#define LED_PIN 12

const uint8_t motor1Pins[4] = {2, 4, 3, 5};
const uint8_t motor2Pins[4] = {6, 8, 7, 9};

unsigned int forward[4] = {0x03, 0x06, 0x0C, 0x09};
unsigned int reverse[4] = {0x03, 0x09, 0x0C, 0x06};

const float diffThreshold = 4.0;
const int requiredFishTriggers = 2;
const int vibrationThreshold = 20;
const int STEPS_PER_ROTATION = 2048;

Adafruit_MPU6050 mpu[SENSOR_COUNT];
Adafruit_MPU6050 mpuWave;

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void SetMotor(uint8_t pins[4], uint8_t InputData) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], (InputData >> i) & 0x01);
  }
}

void motor_rotate_angle(uint8_t pins[4], float angle, int direction, int delay_ms) {
  int totalSteps = (int)(STEPS_PER_ROTATION * (angle / 360.0));
  for (int step = 0; step < totalSteps; step++) {
    for (int j = 0; j < 4; j++) {
      SetMotor(pins, direction == 1 ? forward[j] : reverse[j]);
      delay(delay_ms > 2 ? delay_ms : 2);
    }
  }
  SetMotor(pins, 0x00);
}

void motor_circle(uint8_t pins[4], int n, int direction, int delay_ms) {
  for (int i = 0; i < n * 8; i++) {
    for (int j = 0; j < 4; j++) {
      SetMotor(pins, direction == 1 ? forward[j] : reverse[j]);
      delay(delay_ms > 2 ? delay_ms : 2);
    }
  }
  SetMotor(pins, 0x00);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(motor1Pins[i], OUTPUT);
    pinMode(motor2Pins[i], OUTPUT);
  }

  // 初始化 3 个鱼传感器
  for (int i = 0; i < SENSOR_COUNT; i++) {
    tcaSelect(tcaChannels[i]);
    mpu[i].begin();
    mpu[i].setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu[i].setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  // 初始化波动传感器
  tcaSelect(WAVE_CHANNEL);
  mpuWave.begin();
  mpuWave.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuWave.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(1000);
}

void loop() {
  // 获取波动传感器 XY 合加速度
  tcaSelect(WAVE_CHANNEL);
  sensors_event_t aw, gw, tw;
  mpuWave.getEvent(&aw, &gw, &tw);
  float waveXY = sqrt(aw.acceleration.x * aw.acceleration.x + aw.acceleration.y * aw.acceleration.y);

  float fishXY[SENSOR_COUNT];
  int mismatchCount = 0;

  // 获取每个鱼传感器数据并与环境参考对比
  for (int i = 0; i < SENSOR_COUNT; i++) {
    tcaSelect(tcaChannels[i]);
    sensors_event_t a, g, t;
    mpu[i].getEvent(&a, &g, &t);
    fishXY[i] = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y);
    if (abs(fishXY[i] - waveXY) > diffThreshold) {
      mismatchCount++;
    }
  }

  // 读取压电震动传感器模拟值
  int vib = analogRead(VIBRATION_PIN);

  // === 发送数据到串口（供 Processing 接收） ===
  Serial.print(waveXY, 3); Serial.print(",");
  Serial.print(fishXY[0], 3); Serial.print(",");
  Serial.print(fishXY[1], 3); Serial.print(",");
  Serial.print(fishXY[2], 3); Serial.print(",");
  Serial.println(vib);

  // === 满足判断条件，启动两个马达 ===
  if (mismatchCount >= requiredFishTriggers && vib > vibrationThreshold) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("⚠️ Mismatch & Vibration Detected! Starting motors...");

    // motor1 正转 360°
    motor_rotate_angle(motor1Pins, 90.0, 1, 2);
    delay(300);

    // motor2 正转 360°
    // motor_rotate_angle(motor2Pins, 360.0, 1, 2);

    digitalWrite(LED_PIN, LOW);
  }

  delay(200);
}
