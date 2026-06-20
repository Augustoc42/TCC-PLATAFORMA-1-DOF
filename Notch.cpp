#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <avr/wdt.h>

const uint8_t MPU_ADDR         = 0x68;  
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A;  
const uint8_t REG_ACCEL_XOUT_H = 0x3B;  
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const float   GYRO_LSB_PER_DPS = 131.0f;  
const uint8_t DLPF_PRODUCAO    = 0x03;  
const uint8_t DLPF_BANDA_LARGA = 0x00;-
const uint32_t LOOP_US = 4000UL;
const float    FS_HZ   = 250.0f;  
const uint8_t PINO_ESC_A = 9;
const uint8_t PINO_ESC_B = 10;
const int PWM_MIN        = 1000;  
const int PWM_OPERACAO   = 1350;  
const int PWM_MAX_SEGURO = 1700; 

//VARIAVEIS
int16_t  gyroX_raw_lsb;
int16_t  accX, accY, accZ;
float    gyroX_cal = 0.0f;
uint32_t timer_loop;
int      pwm_atual = PWM_MIN;
uint8_t  dlpf_atual = DLPF_BANDA_LARGA;
Servo escA, escB;
const uint8_t I2C_MAX_FALHAS = 10;

// FILTRO NOTCH BIQUAD
struct BiquadNotch {
    // a0 normalizado para 1
    float b0 = 1.0f, b1 = 0.0f, b2 = 1.0f, a1 = 0.0f, a2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
    float f0 = 0.0f, Q = 5.0f;
    bool  ativo = false;

    void configurar(float freq, float fatorQ, float fs) {
        Q  = (fatorQ > 0.1f) ? fatorQ : 0.1f;
        f0 = freq;
        if (freq <= 0.0f || freq >= 0.5f * fs) {
            b0 = 1.0f; b1 = 0.0f; b2 = 0.0f; a1 = 0.0f; a2 = 0.0f;
            ativo = false;
        } else {
            float w0    = 2.0f * PI * freq / fs;
            float alpha = sinf(w0) / (2.0f * Q);
            float a0    = 1.0f + alpha;
            b0 = 1.0f / a0;
            b2 = b0;                       
            b1 = -2.0f * cosf(w0) / a0;
            a1 = b1;                      
            a2 = (1.0f - alpha) / a0;
            ativo = true;
        }
        // zerar o estado para evitar transitorio ao reconfigurar
        x1 = x2 = y1 = y2 = 0.0f;
    }

    // Aplica o biquad forma otimizada abaixo usa b0==b2 e b1==a1
    float aplicar(float x) {
        if (!ativo) return x;           
        float y = b0 * (x + x2) + b1 * (x1 - y1) - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};
BiquadNotch notch;
//CAPTURA
bool     capturando     = false;
uint32_t captura_fim_ms = 0;
const float CAP_SEG_PADRAO = 6.0f;
const float CAP_SEG_MAX    = 30.0f;

void  resetarI2C();
void  lerSerial();
void  processarComando(char* cmd);
bool  lerSensor();
bool  inicializarMPU6050(uint8_t dlpf);
void  pararMotores();
void  definirPWM(int us);
void  iniciarCaptura(float segundos);

void setup() {
    wdt_enable(WDTO_2S);

  escA.attach(PINO_ESC_A);
    escB.attach(PINO_ESC_B);
    pararMotores();

    Serial.begin(250000);
    Wire.begin();
    Wire.setClock(400000);
   // banda larga para caracterizar o ruido
    dlpf_atual = DLPF_BANDA_LARGA;     
    if (!inicializarMPU6050(dlpf_atual)) {
        Serial.println(F("ERRO: MPU6050 nao encontrado!"));
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            wdt_reset();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    Serial.print(F("# Calibrando gyro (mantenha PARADO)..."));
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
    notch.configurar(0.0f, 5.0f, FS_HZ);

    wdt_reset();
    timer_loop = micros();
    Serial.println(F("# PRONTO_VALIDACAO_NOTCH"));
    Serial.println(F("# Cmds: W<seg>=captura  F<hz>=freq  G<q>=Q  M<us>=PWM  D<n>=DLPF  S=stop  X=stop_cap  ?=status"));
    Serial.println(F("# Linha: W,t_ms,gyro_raw,gyro_notch"));
    Serial.println(F("# SEGURANCA: estrutura TRAVADA; iniciar PWM baixo; 'S' para parar."));
}
void loop() {
    wdt_reset();
    uint32_t agora = micros();
    if ((agora - timer_loop) < LOOP_US) {
        lerSerial();
        return;
    }
    timer_loop = agora;
    {
        static uint8_t i2c_falhas = 0;
        if (!lerSensor()) {
            i2c_falhas++;
            if (i2c_falhas >= I2C_MAX_FALHAS) {
                Serial.println(F("# WARN: I2C reset"));
                resetarI2C();
                delay(2);
                inicializarMPU6050(dlpf_atual);
                i2c_falhas = 0;
                return;
            }
        } else {
            i2c_falhas = 0;
        }
    }

    // Taxa do giroscopio calibrada
    float gyro_raw   = ((float)gyroX_raw_lsb - gyroX_cal) / GYRO_LSB_PER_DPS;
    float gyro_notch = notch.aplicar(gyro_raw);

    if (capturando) {
        Serial.print('W');               Serial.print(',');
        Serial.print(millis());          Serial.print(',');
        Serial.print(gyro_raw, 3);       Serial.print(',');
        Serial.println(gyro_notch, 3);

        if ((long)(millis() - captura_fim_ms) >= 0) {
            capturando = false;
            Serial.println(F("# FIM"));
        }
    }

    lerSerial();
}

bool inicializarMPU6050(uint8_t dlpf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
    if (Wire.available() < 1 || Wire.read() != 0x68) return false;  // WHO_AM_I = 0x68

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);                 // sai do modo de repouso
    if (Wire.endTransmission(true) != 0) return false;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(dlpf);                 // DLPF_CFG (banda do sensor)
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

        accX = ax; accY = ay; accZ = az; gyroX_raw_lsb = gx;
        return true;
    }
    return false;
}

void pararMotores() {
    pwm_atual = PWM_MIN;
    escA.writeMicroseconds(PWM_MIN);
    escB.writeMicroseconds(PWM_MIN);
}
void definirPWM(int us) {
    us = constrain(us, PWM_MIN, PWM_MAX_SEGURO);
    pwm_atual = us;
    escA.writeMicroseconds(us);
    escB.writeMicroseconds(us);
}
void iniciarCaptura(float segundos) {
    segundos = constrain(segundos, 0.5f, CAP_SEG_MAX);
    capturando     = true;
    captura_fim_ms = millis() + (uint32_t)(segundos * 1000.0f);
    Serial.print(F("# INICIO W dur="));
    Serial.print(segundos, 1);
    Serial.print(F("s  pwm="));
    Serial.print(pwm_atual);
    Serial.print(F("  notch="));
    if (notch.ativo) { Serial.print(notch.f0, 1); Serial.print(F("Hz Q")); Serial.println(notch.Q, 1); }
    else             { Serial.println(F("OFF")); }
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

        case 'W': {
            float seg = (valor > 0.0f) ? valor : CAP_SEG_PADRAO;
            iniciarCaptura(seg);
            break;
        }

        case 'F': {
            notch.configurar(valor, notch.Q, FS_HZ);
            Serial.print(F("# NOTCH f0="));
            if (notch.ativo) { Serial.print(notch.f0, 1); Serial.print(F(" Hz Q=")); Serial.println(notch.Q, 1); }
            else             { Serial.println(F("OFF (passagem direta)")); }
            break;
        }

        case 'G': {
            notch.configurar(notch.f0, valor, FS_HZ);
            Serial.print(F("# NOTCH Q="));
            Serial.print(notch.Q, 2);
            Serial.print(F("  f0="));
            if (notch.ativo) { Serial.print(notch.f0, 1); Serial.println(F(" Hz")); }
            else             { Serial.println(F("OFF")); }
            break;
        }

        case 'M': {
            definirPWM((int)valor);
            Serial.print(F("# PWM="));
            Serial.print(pwm_atual);
            Serial.println(F(" us (ambos)"));
            break;
        }

        case 'D': {
            uint8_t cfg = (uint8_t)((int)valor) & 0x07;
            dlpf_atual = cfg;
            inicializarMPU6050(dlpf_atual);
            Serial.print(F("# DLPF=0x"));
            Serial.println(dlpf_atual, HEX);
            break;
        }

        case 'S':
            pararMotores();
            Serial.println(F("# STOP motores=1000us"));
            break;

        case 'X':
            capturando = false;
            Serial.println(F("# FIM (interrompido)"));
            break;

        case '?':
            Serial.print(F("# [status] pwm=")); Serial.print(pwm_atual);
            Serial.print(F("us  notch="));
            if (notch.ativo) { Serial.print(notch.f0, 1); Serial.print(F("Hz/Q")); Serial.print(notch.Q, 1); }
            else             { Serial.print(F("OFF")); }
            Serial.print(F("  fs=")); Serial.print(FS_HZ, 0);
            Serial.print(F("Hz  offset=")); Serial.print(gyroX_cal, 2);
            Serial.print(F("  dlpf=0x")); Serial.print(dlpf_atual, HEX);
            Serial.print(F("  capturando=")); Serial.println(capturando ? F("SIM") : F("NAO"));
            break;

        default:
            Serial.println(F("# ERR: use W<seg>/F<hz>/G<q>/M<us>/D<n>/S/X/?"));
            break;
    }
}
