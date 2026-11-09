#include <Wire.h>
#include <math.h>

// IMPORTANTE: los structs deben ir ANTES de la primera funcion del sketch.
// El IDE genera los prototipos de las funciones y los coloca antes del
// primer codigo, asi que un struct declarado despues no se reconoce.
struct Biquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
};

// Calibracion guardada en flash: ultimo sector (sector 11) del STM32F405RG (1 MB).
// Si tu chip es de 512 KB (F405 solo con 512K) la direccion seria 0x08060000 y sector 7.
#define FLASH_CAL_ADDR 0x080E0000UL
#define CAL_MAGIC      0xB160CA11UL
struct CalData {
  uint32_t magic;
  float ax, ay, az, gx, gy, gz;
  uint32_t checksum;   // debe ser el ultimo campo
};

// =====================================================
// ============ ENCODERS (TIM2, TIM3, TIM5, TIM8) ======
// E1: TIM2 PA15/PB3 (AF1, 32 bits)   E2: TIM3 PA6/PA7 (AF2, 16 bits)
// E3: TIM5 PA0/PA1  (AF2, 32 bits)   E4: TIM8 PC6/PC7 (AF3, 16 bits)
// El conteo de cada pulso lo hace el timer por hardware.
// Una ISR de TIM4 a 1 kHz acumula el delta con signo (sin perder vueltas).
// =====================================================

#define FILTRO 6          // filtro de entrada (0..15)
#define ENC_RATE_HZ 1000  // frecuencia de la ISR de encoders

struct Enc {
  TIM_TypeDef *tim;
  bool es32;
  uint16_t last16;
  uint32_t last32;
  volatile int32_t total;
};

Enc enc[4] = {
  { TIM2, true,  0, 0, 0 },
  { TIM3, false, 0, 0, 0 },
  { TIM5, true,  0, 0, 0 },
  { TIM8, false, 0, 0, 0 },
};

HardwareTimer *timEnc;

void pinAF(GPIO_TypeDef *port, uint32_t pin, uint32_t af) {
  GPIO_InitTypeDef g = {0};
  g.Pin       = pin;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;      // GPIO_PULLUP si el encoder es de colector abierto
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = af;
  HAL_GPIO_Init(port, &g);
}

void configEncoder(TIM_TypeDef *t, bool es32) {
  t->CR1   = 0;
  t->PSC   = 0;
  t->ARR   = es32 ? 0xFFFFFFFF : 0xFFFF;
  t->CCMR1 = (1 << 0) | (1 << 8) | (FILTRO << 4) | (FILTRO << 12);
  t->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
  t->SMCR  = 3;                   // SMS=011: encoder x4
  t->CNT   = 0;
  t->CR1  |= TIM_CR1_CEN;
}

// ISR: se ejecuta cada 1 ms, muy corta (solo lee registros)
void isrEncoders() {
  for (int i = 0; i < 4; i++) {
    if (enc[i].es32) {
      uint32_t now = enc[i].tim->CNT;
      enc[i].total += (int32_t)(now - enc[i].last32);
      enc[i].last32 = now;
    } else {
      uint16_t now = (uint16_t)enc[i].tim->CNT;
      enc[i].total += (int16_t)(now - enc[i].last16);
      enc[i].last16 = now;
    }
  }
}

void leerEncoders(int32_t out[4]) {
  noInterrupts();
  for (int i = 0; i < 4; i++) out[i] = enc[i].total;
  interrupts();
}

void ceroEncoders() {
  noInterrupts();
  for (int i = 0; i < 4; i++) enc[i].total = 0;
  interrupts();
}

void initEncoders() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  pinAF(GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2);
  pinAF(GPIOB, GPIO_PIN_3,  GPIO_AF1_TIM2);
  pinAF(GPIOA, GPIO_PIN_6,  GPIO_AF2_TIM3);
  pinAF(GPIOA, GPIO_PIN_7,  GPIO_AF2_TIM3);
  pinAF(GPIOA, GPIO_PIN_0,  GPIO_AF2_TIM5);
  pinAF(GPIOA, GPIO_PIN_1,  GPIO_AF2_TIM5);
  pinAF(GPIOC, GPIO_PIN_6,  GPIO_AF3_TIM8);
  pinAF(GPIOC, GPIO_PIN_7,  GPIO_AF3_TIM8);

  for (int i = 0; i < 4; i++) configEncoder(enc[i].tim, enc[i].es32);

  // Timer de muestreo (TIM4, libre en tu mapa de pines)
  timEnc = new HardwareTimer(TIM4);
  timEnc->setOverflow(ENC_RATE_HZ, HERTZ_FORMAT);
  timEnc->attachInterrupt(isrEncoders);
  timEnc->resume();
}

// =====================================================
// =========== FILTRO BIQUAD ===========================
// =====================================================
void initBiquad(Biquad &f, float fc, float fs) {
  float w0    = 2.0 * M_PI * fc / fs;
  float alpha = sin(w0) / (2.0 * 0.707);
  float cosw0 = cos(w0);
  float a0    = 1.0 + alpha;
  f.b0 = ((1.0 - cosw0) / 2.0) / a0;
  f.b1 = ( 1.0 - cosw0)        / a0;
  f.b2 = f.b0;
  f.a1 = (-2.0 * cosw0)        / a0;
  f.a2 = ( 1.0 - alpha)        / a0;
  f.x1 = f.x2 = f.y1 = f.y2 = 0.0;
}

float applyBiquad(Biquad &f, float x) {
  float y = f.b0*x + f.b1*f.x1 + f.b2*f.x2
              - f.a1*f.y1 - f.a2*f.y2;
  f.x2 = f.x1; f.x1 = x;
  f.y2 = f.y1; f.y1 = y;
  return y;
}

// ================= BMI160 VARIABLES =================
int bmi_addr = 0x68;

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float yaw_offset = 0.0;
unsigned long lastTime = 0;

#define MI_SDA_PIN PB7
#define MI_SCL_PIN PB6
float gyro_deadband = 0.5;

Biquad bq_gx, bq_gy, bq_gz;

// =====================================================
// ============== FILTRO KALMAN EXTENDIDO ==============
// =====================================================
float X[6] = {0, 0, 0, 0, 0, 0};
float P[6][6];

const float Q_angle = 0.001;
const float Q_bias  = 0.003;
const float R_angle = 0.03;

void initEKF() {
  for (int i = 0; i < 6; i++) X[i] = 0.0;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      P[i][j] = (i == j) ? 1.0 : 0.0;
}

void updateEKF(float gx, float gy, float gz,
               float ax, float ay, float az, float dt) {
  float gx_c = gx - X[3];
  float gy_c = gy - X[4];
  float gz_c = gz - X[5];

  float sr = sin(X[0]), cr = cos(X[0]);
  float cp = cos(X[1]);
  float tp = sin(X[1]) / cp;

  X[0] += dt * (gx_c + sr * tp * gy_c + cr * tp * gz_c);
  X[1] += dt * (cr * gy_c - sr * gz_c);
  X[2] += dt * (sr / cp * gy_c + cr / cp * gz_c);

  while (X[2] >  M_PI) X[2] -= 2 * M_PI;
  while (X[2] < -M_PI) X[2] += 2 * M_PI;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      P[i][j] += dt * dt * P[i+3][j+3];
    P[i][i] += Q_angle * dt;
  }
  for (int i = 3; i < 6; i++)
    P[i][i] += Q_bias * dt;

  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.5 || mag > 2.0) return;

  float roll_meas  = atan2(ay, az);
  float pitch_meas = atan2(-ax, sqrt(ay*ay + az*az));

  float y_roll  = roll_meas  - X[0];
  float y_pitch = pitch_meas - X[1];

  float S_roll  = P[0][0] + R_angle;
  float S_pitch = P[1][1] + R_angle;

  float K_roll[6], K_pitch[6];
  for (int i = 0; i < 6; i++) {
    K_roll[i]  = P[i][0] / S_roll;
    K_pitch[i] = P[i][1] / S_pitch;
  }

  for (int i = 0; i < 6; i++) {
    X[i] += K_roll[i]  * y_roll;
    X[i] += K_pitch[i] * y_pitch;
  }

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) {
      P[i][j] -= K_roll[i]  * P[0][j];
      P[i][j] -= K_pitch[i] * P[1][j];
    }
}

// =====================================================
// ============= FUNCIONES BMI160 =====================
// =====================================================
void configurarSensorRapido() {
  Wire.beginTransmission(bmi_addr); Wire.write(0x7E); Wire.write(0x11); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(bmi_addr); Wire.write(0x7E); Wire.write(0x15); Wire.endTransmission(); delay(100);
  Wire.beginTransmission(bmi_addr); Wire.write(0x41); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(bmi_addr); Wire.write(0x43); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(bmi_addr); Wire.write(0x42); Wire.write(0x2C); Wire.endTransmission();
  Wire.beginTransmission(bmi_addr); Wire.write(0x40); Wire.write(0x2C); Wire.endTransmission();
  Serial.println("BMI160: +/-2g, 2000dps, 1600Hz");
}

void leerRawData(int16_t *ax, int16_t *ay, int16_t *az,
                 int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(bmi_addr); Wire.write(0x0C); Wire.endTransmission();
  Wire.requestFrom(bmi_addr, 12);
  if (Wire.available() == 12) {
    *gx = (int16_t)(Wire.read() | (Wire.read() << 8));
    *gy = (int16_t)(Wire.read() | (Wire.read() << 8));
    *gz = (int16_t)(Wire.read() | (Wire.read() << 8));
    *ax = (int16_t)(Wire.read() | (Wire.read() << 8));
    *ay = (int16_t)(Wire.read() | (Wire.read() << 8));
    *az = (int16_t)(Wire.read() | (Wire.read() << 8));
  }
}

void calibrarBMI() {
  const int N = 3000;
  long ax_s=0, ay_s=0, az_s=0, gx_s=0, gy_s=0, gz_s=0;
  int16_t ax, ay, az, gx, gy, gz;
  Serial.println("=== CALIBRANDO (NO MOVER) ===");
  for (int i = 0; i < N; i++) {
    leerRawData(&ax, &ay, &az, &gx, &gy, &gz);
    ax_s+=ax; ay_s+=ay; az_s+=az;
    gx_s+=gx; gy_s+=gy; gz_s+=gz;
    delayMicroseconds(500);
  }
  ax_offset = ax_s/N; ay_offset = ay_s/N; az_offset = (az_s/N) - 16384;
  gx_offset = gx_s/N; gy_offset = gy_s/N; gz_offset = gz_s/N;
  Serial.println("Calibracion finalizada.");
}

// ---------- Calibracion persistente en flash ----------
uint32_t calChecksum(const CalData &c) {
  const uint32_t *w = (const uint32_t *)&c;
  uint32_t s = 17;
  for (size_t i = 0; i < sizeof(CalData) / 4 - 1; i++) s = s * 31 + w[i];
  return s;
}

bool guardarCalibracion() {
  CalData c = { (uint32_t)CAL_MAGIC, ax_offset, ay_offset, az_offset,
                gx_offset, gy_offset, gz_offset, 0 };
  c.checksum = calChecksum(c);

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef e = {};
  e.TypeErase    = FLASH_TYPEERASE_SECTORS;
  e.Sector       = FLASH_SECTOR_11;
  e.NbSectors    = 1;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 2.7 - 3.6 V
  uint32_t err = 0;
  bool ok = (HAL_FLASHEx_Erase(&e, &err) == HAL_OK);

  const uint32_t *w = (const uint32_t *)&c;
  for (size_t i = 0; ok && i < sizeof(CalData) / 4; i++)
    ok = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CAL_ADDR + 4 * i, w[i]) == HAL_OK);
  HAL_FLASH_Lock();

  Serial.println(ok ? "Calibracion guardada en flash." : "ERROR al guardar en flash.");
  return ok;
}

bool cargarCalibracion() {
  const CalData *c = (const CalData *)FLASH_CAL_ADDR;
  if (c->magic != CAL_MAGIC || c->checksum != calChecksum(*c)) return false;
  ax_offset = c->ax; ay_offset = c->ay; az_offset = c->az;
  gx_offset = c->gx; gy_offset = c->gy; gz_offset = c->gz;
  return true;
}

// =====================================================
// ================== UPDATE / INIT ===================
// =====================================================
void updateIMU() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.1) dt = 0.001;

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  leerRawData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  float ax = (ax_raw - ax_offset) / 16384.0;
  float ay = (ay_raw - ay_offset) / 16384.0;
  float az = (az_raw - az_offset) / 16384.0;

  float gx = (gx_raw - gx_offset) / 16.4 * DEG_TO_RAD;
  float gy = (gy_raw - gy_offset) / 16.4 * DEG_TO_RAD;
  float gz = (gz_raw - gz_offset) / 16.4 * DEG_TO_RAD;

  float db = gyro_deadband * DEG_TO_RAD;
  if (abs(gx) < db) gx = 0;
  if (abs(gy) < db) gy = 0;
  if (abs(gz) < db) gz = 0;

  gx = applyBiquad(bq_gx, gx);
  gy = applyBiquad(bq_gy, gy);
  gz = applyBiquad(bq_gz, gz);

  updateEKF(gx, gy, gz, ax, ay, az, dt);

  roll  = X[0] * RAD_TO_DEG;
  pitch = X[1] * RAD_TO_DEG;
  float yTemp = (X[2] - yaw_offset) * RAD_TO_DEG;
  while (yTemp >  180.0) yTemp -= 360.0;
  while (yTemp < -180.0) yTemp += 360.0;
  yaw = yTemp;
}

void initIMU() {
  Wire.setSDA(MI_SDA_PIN);
  Wire.setSCL(MI_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  bmi_addr = 0;
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    bmi_addr = 0x68;
  } else {
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) bmi_addr = 0x69;
  }

  if (bmi_addr == 0) {
    Serial.println("BMI160 NO DETECTADO en I2C (PB6/PB7)");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  Serial.print("BMI160 detectado en 0x");
  Serial.println(bmi_addr, HEX);

  // El I2C del STM32F4 llega como maximo a 400 kHz (Fast mode).
  // No hay Fast-mode Plus (1 MHz) en esta familia.
  configurarSensorRapido();
  if (cargarCalibracion()) {
    Serial.println("Calibracion cargada desde flash (envia 'c' para recalibrar).");
  } else {
    Serial.println("Sin calibracion guardada: calibrando (NO MOVER, sensor plano)...");
    calibrarBMI();
    guardarCalibracion();
  }
  initEKF();

  initBiquad(bq_gx, 60.0, 1600.0);
  initBiquad(bq_gy, 60.0, 1600.0);
  initBiquad(bq_gz, 60.0, 1600.0);

  Serial.println("Estabilizando EKF...");
  for (int i = 0; i < 2000; i++) {
    updateIMU();
    delay(2);
  }
  yaw_offset = X[2];
  Serial.println("Frente fijado a 0.00 deg");
  lastTime = micros();
}

// =====================================================
// ====================== MAIN =========================
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);

  initEncoders();   // primero: la ISR ya corre mientras se calibra la IMU
  initIMU();
  ceroEncoders();   // posicion cero al terminar la calibracion

  Serial.println("=== ENCODERS + BMI160 + EKF LISTO ('z' = cero encoders) ===");
}

void loop() {
  updateIMU();

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'z') {
      ceroEncoders();
    } else if (cmd == 'c') {          // recalibrar IMU (sensor quieto y plano)
      calibrarBMI();
      guardarCalibracion();
      initEKF();
      lastTime = micros();
    }
  }

  static unsigned long tP = 0;
  if (millis() - tP > 40) {
    tP = millis();
    int32_t e[4];
    leerEncoders(e);
    Serial.print("P:");  Serial.print(pitch, 3);
    Serial.print("\tR:"); Serial.print(roll, 3);
    Serial.print("\tY:"); Serial.print(yaw, 3);
    Serial.print("\tE1:"); Serial.print(e[0]);
    Serial.print("\tE2:"); Serial.print(e[1]);
    Serial.print("\tE3:"); Serial.print(e[2]);
    Serial.print("\tE4:"); Serial.println(e[3]);
  }
}
