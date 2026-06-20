#include <Wire.h>
#include <Arduino.h>
#include <avr/wdt.h>
// --- REGISTRADORES E ESCALAS DO MPU6050
const uint8_t MPU_ADDR         = 0x68; 
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A; 
const uint8_t REG_ACCEL_XOUT_H = 0x3B;  
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const float   GYRO_LSB_PER_DPS = 131.0f;    
const float   ACC_LSB_PER_G    = 16384.0f;  
const uint32_t LOOP_US = 4000UL;

//GATING DO ACELEROMETRO NO KALMAN
const float R_MEASURE_BASE = 0.05f;
const float ACC_GATE_TOL   = 0.10f;
const float ACC_GATE_GAIN  = 8.0f;
const float R_MEASURE_MAX  = 5.0f;

//VARIAVEIS GLOBAIS
int16_t  accX, accY, accZ, gyroX;
float    gyroX_cal = 0.0f;
uint32_t timer_loop;
const uint8_t I2C_MAX_FALHAS = 10;

// FILTRO DE KALMAN 
struct Kalman {
    float Q_angle   = 0.001f;
    float Q_bias    = 0.003f;
    float R_measure = 0.05f;
    float angle     = 0.0f;
    float bias      = 0.0f;
    float P[2][2]   = {{10.0f, 0.0f}, {0.0f, 0.1f}};
};
Kalman kX;
bool kalman_inicializado = false;

//INTEGRACAO PURA DO GIROSCOPIO
float gyro_int_angle = 0.0f;

//CAPTURA
bool     capturando      = false;
bool     captura_primeira = false; 
uint32_t captura_fim_ms  = 0;
const float CAP_SEG_PADRAO = 8.0f;
const float CAP_SEG_MAX    = 120.0f;

void  resetarI2C();
void  lerSerial();
void  processarComando(char* cmd);
float calcularKalman(float accAngle, float gyroRate, float dt);
bool  lerSensor();
bool  inicializarMPU6050();
void  resetarKalman();
void  iniciarCaptura(float segundos, bool fria);

void setup() {
    wdt_enable(WDTO_2S);
    Serial.begin(250000);
    Wire.begin();
    Wire.setClock(400000);

    if (!inicializarMPU6050()) {
        Serial.println(F("MPU6050 nao encontrado!"));
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            wdt_reset();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    Serial.print(F("# Calibrando gyro"));
    long gyro_sum = 0;
    for (int i = 0; i < 2000; i++) {
        wdt_reset();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_GYRO_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
        gyro_sum += (int16_t)(Wire.read() << 8 | Wire.read());
        delay(1);
    }
    gyroX_cal = gyro_sum / 2000.0f;
    Serial.print(F("OK offset="));
    Serial.println(gyroX_cal, 2);
    Serial.print(F("# Aquecendo Kalman"));
    const float DT_WARM = 0.004f;
    for (int i = 0; i < 500; i++) {
        wdt_reset();
        if (lerSensor()) {
            float accA = atan2f((float)accY,
                                sqrtf((float)accX * accX + (float)accZ * accZ)) * 180.0f / PI;
            float gR = ((float)gyroX - gyroX_cal) / GYRO_LSB_PER_DPS;
            calcularKalman(accA, gR, DT_WARM);
        }
        delay(4);
    }
    gyro_int_angle = kX.angle;
    Serial.print(F("OK angulo="));
    Serial.print(kX.angle, 2);
    Serial.println(F(" deg"));

    wdt_reset();
    timer_loop = micros();
    Serial.println(F("# PRONTO_VALIDACAO_KALMAN"));
    Serial.println(F("# Cmds: V<seg>=captura  C<seg>=partida_fria  Z=resync  X=stop  ?=status"));
    Serial.println(F("# Linha: V,t_ms,acc_angle,gyro_int_angle,kalman_angle,gyro_rate"));
}

void loop() {
    wdt_reset();
    uint32_t agora = micros();
    if ((agora - timer_loop) < LOOP_US) {
        lerSerial();
        return;
    }
    float dt = (float)(agora - timer_loop) / 1000000.0f;
    timer_loop = agora;
    dt = constrain(dt, 0.001f, 0.010f);
    {
        static uint8_t i2c_falhas = 0;
        if (!lerSensor()) {
            i2c_falhas++;
            if (i2c_falhas >= I2C_MAX_FALHAS) {
                Serial.println(F("# WARN: I2C reset"));
                resetarI2C();
                delay(2);
                inicializarMPU6050();
                i2c_falhas = 0;
                return;
            }
        } else {
            i2c_falhas = 0;
        }
    }
    float gyroRate = ((float)gyroX - gyroX_cal) / GYRO_LSB_PER_DPS;
    float accAngle = atan2f((float)accY,
                            sqrtf((float)accX * accX + (float)accZ * accZ)) * 180.0f / PI;
    //gating do acelerometro
    float acc_mag    = sqrtf((float)accX * accX + (float)accY * accY + (float)accZ * accZ);
    float acc_desvio = fabsf(acc_mag - ACC_LSB_PER_G) / ACC_LSB_PER_G;
    float r_extra    = acc_desvio - ACC_GATE_TOL;
    kX.R_measure     = (r_extra <= 0.0f)
                         ? R_MEASURE_BASE
                         : constrain(R_MEASURE_BASE * (1.0f + ACC_GATE_GAIN * r_extra),
                                     R_MEASURE_BASE, R_MEASURE_MAX);

    float kalman_angle = calcularKalman(accAngle, gyroRate, dt);

    // Integracao PURA do giroscopio sem correcao do Kalman
    if (capturando && captura_primeira) {
        gyro_int_angle   = kalman_angle;
        captura_primeira = false;
    } else {
        gyro_int_angle += gyroRate * dt;
    }

    if (capturando) {
        Serial.print('V');                       Serial.print(',');
        Serial.print(millis());                  Serial.print(',');
        Serial.print(accAngle, 2);               Serial.print(',');
        Serial.print(gyro_int_angle, 2);         Serial.print(',');
        Serial.print(kalman_angle, 2);           Serial.print(',');
        Serial.println(gyroRate, 2);

        if ((long)(millis() - captura_fim_ms) >= 0) {
            capturando = false;
            Serial.println(F("# FIM"));
        }
    }

    lerSerial();
}

bool inicializarMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
  
    if (Wire.available() < 1 || Wire.read() != 0x68) return false; 
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);             
  
    if (Wire.endTransmission(true) != 0) return false;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x03);                
    
  if (Wire.endTransmission(true) != 0) return false;
  
    return true;
}

bool lerSensor() {
    for (uint8_t t = 0; t < 3; t++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        if (Wire.requestFrom(MPU_ADDR, (uint8_t)10, (uint8_t)true) < 10) {
            delayMicroseconds(200);
            continue;
        }
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read();             
        int16_t gx = Wire.read() << 8 | Wire.read();
        accX = ax; accY = ay; accZ = az; gyroX = gx;
        return true;
    }
    return false;
}


float calcularKalman(float newAngle, float newRate, float dt) {
    // inicializacao para partida a fria
    if (!kalman_inicializado) {
        kX.angle = newAngle;
        kalman_inicializado = true;
    }

    kX.angle   += dt * (newRate - kX.bias);
    kX.P[0][0] += dt * (dt * kX.P[1][1] - kX.P[0][1] - kX.P[1][0] + kX.Q_angle);
    kX.P[0][1] -= dt * kX.P[1][1];
    kX.P[1][0] -= dt * kX.P[1][1];
    kX.P[1][1] += kX.Q_bias * dt;

    float S  = kX.P[0][0] + kX.R_measure;
    float K0 = kX.P[0][0] / S;
    float K1 = kX.P[1][0] / S;

    float y   = newAngle - kX.angle;
    kX.angle += K0 * y;
    kX.bias  += K1 * y;
    kX.bias   = constrain(kX.bias, -10.0f, 10.0f);

    float P00 = kX.P[0][0], P01 = kX.P[0][1];
    kX.P[0][0] -= K0 * P00;
    kX.P[0][1] -= K0 * P01;
    kX.P[1][0] -= K1 * P00;
    kX.P[1][1] -= K1 * P01;

    return kX.angle;
}

void resetarKalman() {
    kX.angle   = 0.0f;
    kX.bias    = 0.0f;
    kX.P[0][0] = 10.0f; kX.P[0][1] = 0.0f;
    kX.P[1][0] = 0.0f;  kX.P[1][1] = 0.1f;
    kX.R_measure = R_MEASURE_BASE;
    kalman_inicializado = false;
}
void iniciarCaptura(float segundos, bool fria) {
    segundos = constrain(segundos, 0.5f, CAP_SEG_MAX);
    if (fria) resetarKalman();
    capturando       = true;
    captura_primeira = true;
    captura_fim_ms   = millis() + (uint32_t)(segundos * 1000.0f);
    Serial.print(F("# INICIO "));
    Serial.print(fria ? F("C") : F("V"));
    Serial.print(F(" dur="));
    Serial.print(segundos, 1);
    Serial.println(F("s"));
}
void resetarI2C() {
    Wire.end();

    pinMode(SDA, OUTPUT);
    pinMode(SCL, OUTPUT);
    digitalWrite(SDA, HIGH);
    delayMicroseconds(5);

    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(SCL, LOW);  delayMicroseconds(5);
        digitalWrite(SCL, HIGH); delayMicroseconds(5);
    }
    digitalWrite(SDA, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    digitalWrite(SDA, HIGH); delayMicroseconds(5);

    Wire.begin();
    Wire.setClock(400000);
}
void lerSerial() {
    static char buffer[16];
    static byte idx = 0;
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buffer[idx] = '\0';
                processarComando(buffer);
                idx = 0;
            }
        } else if (idx < 15) {
            buffer[idx++] = c;
        }
    }
}
void processarComando(char* cmd) {
    char  tipo  = toupper(cmd[0]);
    float valor = atof(cmd + 1);

    switch (tipo) {

        case 'V': {
            float seg = (valor > 0.0f) ? valor : CAP_SEG_PADRAO;
            iniciarCaptura(seg, false);
            break;
        }

        case 'C': {
            float seg = (valor > 0.0f) ? valor : CAP_SEG_PADRAO;
            iniciarCaptura(seg, true);
            break;
        }

        case 'Z':
            gyro_int_angle = kX.angle;
            Serial.println(F("# RESYNC integracao=Kalman"));
            break;

        case 'X':
            capturando = false;
            Serial.println(F("# FIM (interrompido)"));
            break;

        case '?':
            Serial.print(F("# [status] kalman=")); Serial.print(kX.angle, 2);
            Serial.print(F(" deg  bias=")); Serial.print(kX.bias, 4);
            Serial.print(F(" dps  gyro_int=")); Serial.print(gyro_int_angle, 2);
            Serial.print(F(" deg  R=")); Serial.print(kX.R_measure, 3);
            Serial.print(F("  offset=")); Serial.print(gyroX_cal, 2);
            Serial.print(F("  capturando=")); Serial.println(capturando ? F("SIM") : F("NAO"));
            break;
        default:
            Serial.println(F("V<seg>/C<seg>/Z/X/?"));
            break;
    }
}
