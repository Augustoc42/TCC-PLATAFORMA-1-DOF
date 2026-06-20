#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <avr/wdt.h>

//HARDWARE 
const int PIN_M1 = 9;
const int PIN_M2 = 10;

//REGISTRADORES E ESCALAS DO MPU6050
const uint8_t MPU_ADDR         = 0x68;
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A;  // DLPF
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const float   GYRO_LSB_PER_DPS = 131.0f;    // FS_SEL=0 -> +/-250 graus/s
const float   ACC_LSB_PER_G    = 16384.0f;  // AFS_SEL=0 -> +/-2 g

// DLPF do MPU6050
uint8_t MPU_DLPF = 3;
const uint32_t LOOP_US = 4000UL;

//DIVISOR DA MALHA EXTERNA 50 Hz e 250
const uint8_t OUTER_DIV = 5;

//VELOCIDADE BASE DOS MOTORES
int VEL_BASE = 1260;     // (TCC cita 1350; usar o MESMO valor nos dois controladores)
const int VEL_MIN = 1100;
const int VEL_MAX = 1700;

// CONTROLADOR EM CASCATA
// Malha EXTERNA
float Kp_ang   = 2.00f;    
float Ki_ang   = 0.171f;    
float soma_ang = 0.0f;
float rate_setpoint = 0.0f;
float MAX_RATE_SP = 120.0f; /
const float I_LIMIT_ANG = 700.0f;   
const float I_DECAY_ANG = 0.9975f;

// Malha INTERNA
float Kp_rate   = 0.70f;    
float Ki_rate   = 0.0f;
float Kd_rate   = 0.0f;  
float soma_rate = 0.0f;

const float I_LIMIT_RATE = 300.0f;
const float FREEZE_RATE  = 0.5f;    // nao integra ruido de taxa perto de zero

//Anti-windup zonas mortas e os limites
const float I_LIMIT  = I_LIMIT_ANG;
const float D_LIMIT  = 150.0f;
const float KAW_BACK = 0.2f;
const float ZONA_I_DEG = 2.0f;
const float FREEZE_ANG = 0.3f;
// anti-windup por banda no integrador EXTERNO
const float I_BAND_DEG = 12.0f;     
float DEAD_ANG  = 0.4f;   // zona morta do P EXTERNO (comando H, graus)
float DEAD_RATE = 1.0f;   // zona morta do P INTERNO (comando E, graus/s)

float STICTION_KICK              = 15.0f;
const float STICTION_GYRO_THRESH = 15.0f;
const float STICTION_ERR_THRESH  = 3.0f; 
float pid_out = 0.0f;
//balanceamento 
int   balanceamento = -24;
bool  sistema_ligado = false;

float LIMITE_QUEDA = 45.0f;
const uint16_t FALL_TRIP    = 50;

//GATING DO ACELEROMETRO NO KALMAN 
const float R_MEASURE_BASE = 0.05f;
const float ACC_GATE_TOL   = 0.10f;
const float ACC_GATE_GAIN  = 8.0f;
const float R_MEASURE_MAX  = 5.0f;

//SETPOINT COM RAMPA 
float RAMP_DEGS  = 40.0f;   
float setpoint_target  = -30.0f;
float setpoint_ativo   = -30.0f;
const float PID_LIMIT      = 350.0f;   /
const float GYRO_SAT_DEG_S = 240.0f;

//VARIAVEIS
Servo    m1, m2;
int16_t  accX, accY, accZ, gyroX;
float    gyroX_cal = 0.0f;
uint32_t timer_loop;
uint32_t timer_print;

// FILTRO PT1
struct PT1Filter {
    float state = 0.0f;
    float update(float input, float dt, float cutoff_hz) {
        float RC    = 1.0f / (2.0f * PI * cutoff_hz);
        float alpha = dt / (dt + RC);
        state += alpha * (input - state);
        return state;
    }
    void reset(float v = 0.0f) { state = v; }
};

// Corte do PT2
float PT2_DTERM_HZ = 50.0f;
PT1Filter   pt2a, pt2b;
float       prev_gyroRate_D = 0.0f; 

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

const uint8_t I2C_MAX_FALHAS = 10;
void loop() {
    wdt_reset();
    uint32_t agora = micros();
    if ((agora - timer_loop) < LOOP_US) return;
    float dt = (float)(agora - timer_loop) / 1000000.0f;
    timer_loop = agora;
    dt = constrain(dt, 0.001f, 0.010f);
    //leitura do sensor com tolerancia a falhas I2C
    {
        static uint8_t i2c_falhas = 0;
        if (!lerSensor()) {
            i2c_falhas++;
            if (i2c_falhas >= I2C_MAX_FALHAS) {
                Serial.println(F("WARN: I2C reset"));
                resetarI2C();
                delay(2);
                inicializarMPU6050();
                soma_ang = 0.0f; soma_rate = 0.0f;
                i2c_falhas = 0;
                pararMotores();
                return;
            }
        } else {
            i2c_falhas = 0;
        }
    }

    //  DETECCAO DE GIRO CONGELADO
    {
        const uint8_t GYRO_STUCK_TRIP = 20;    
        static int16_t gyro_prev  = 0;
        static uint8_t gyro_stuck = 0;
        if (gyroX == gyro_prev) {
            if (++gyro_stuck >= GYRO_STUCK_TRIP) {
                if (sistema_ligado) {
                    sistema_ligado = false;
                    pararMotores();
                    soma_ang = 0.0f; soma_rate = 0.0f; pid_out = 0.0f; rate_setpoint = 0.0f;
                    Serial.println(F("FAILSAFE: giro congelado (I2C). DESARMADO."));
                }
                resetarI2C();
                delay(2);
                inicializarMPU6050();
                gyro_stuck = 0;
                return;
            }
        } else {
            gyro_stuck = 0;
            gyro_prev  = gyroX;
        }
    }
float gyroRate    = ((float)gyroX - gyroX_cal) / GYRO_LSB_PER_DPS;
float accAngle    = atan2f((float)accY, sqrtf((float)accX * accX + (float)accZ * accZ)) * 180.0f / PI;

    // gating do acelerometro: infla R_measure fora de 1 g (igual ao PID unico)
    float acc_mag    = sqrtf((float)accX * accX + (float)accY * accY + (float)accZ * accZ);
    float acc_desvio = fabsf(acc_mag - ACC_LSB_PER_G) / ACC_LSB_PER_G;
    float r_extra    = acc_desvio - ACC_GATE_TOL;
    kX.R_measure     = (r_extra <= 0.0f)
                         ? R_MEASURE_BASE : constrain(R_MEASURE_BASE * (1.0f + ACC_GATE_GAIN * r_extra), R_MEASURE_BASE, R_MEASURE_MAX);

    float angulo_real = calcularKalman(accAngle, gyroRate, dt);

    // PT2 (50 Hz) sobre a taxa BRUTA
    float gyroRate_D1 = pt2a.update(gyroRate, dt, PT2_DTERM_HZ);
    float gyroRate_D  = pt2b.update(gyroRate_D1, dt, PT2_DTERM_HZ);   // = omega filtrado

    // rampa do setpoint
    if (sistema_ligado) {
        float delta_max = RAMP_DEGS * dt;
        float diff      = setpoint_target - setpoint_ativo;
        if      (diff >  delta_max) setpoint_ativo += delta_max;
        else if (diff < -delta_max) setpoint_ativo -= delta_max;
        else                        setpoint_ativo  = setpoint_target;
    }

    bool saturado = false;
    int  cmd_pwm1 = 1000, cmd_pwm2 = 1000;

    if (sistema_ligado) {
        float erro_ang = setpoint_ativo - angulo_real;   // calculado todo ciclo (failsafe/kick/externo)
        // FAILSAFE DE INCLINACAO
        static uint16_t fall_count = 0;
        if (fabsf(erro_ang) > LIMITE_QUEDA) {
            if (++fall_count >= FALL_TRIP) {
                sistema_ligado = false;
                pararMotores();
                soma_ang = 0.0f; soma_rate = 0.0f; pid_out = 0.0f; rate_setpoint = 0.0f;
                fall_count = 0;
                Serial.print(F(">> FAILSAFE: |erro| > "));
                Serial.print(LIMITE_QUEDA, 0);
                Serial.println(F(" deg. DESARMADO."));
                return;
            }
        } else fall_count = 0;

        bool gyro_saturado = fabsf(gyroRate) >= GYRO_SAT_DEG_S;
        if (gyro_saturado) saturado = true;

        // MALHA EXTERNA 
        static uint8_t outer_count   = 0;
        static float   dt_outer_acc  = 0.0f;
        dt_outer_acc += dt;
        if (++outer_count >= OUTER_DIV) {
            outer_count  = 0;
            float dt_o   = dt_outer_acc;   
            dt_outer_acc = 0.0f;

            // integrador externo: integracao condicional
            float abs_e = fabsf(erro_ang);
            if (!gyro_saturado) {
                if (abs_e > I_BAND_DEG) {
                    soma_ang *= I_DECAY_ANG;          
                } else if (abs_e >= FREEZE_ANG) {
                    if (abs_e < ZONA_I_DEG) soma_ang *= I_DECAY_ANG;
                    soma_ang = constrain(soma_ang + erro_ang * dt_o, -I_LIMIT_ANG, I_LIMIT_ANG);
                }
            }
            float P_ang   = (abs_e > DEAD_ANG) ? Kp_ang * erro_ang : 0.0f;
            float rsp_uns = P_ang + Ki_ang * soma_ang;
            rate_setpoint = constrain(rsp_uns, -MAX_RATE_SP, MAX_RATE_SP);

            // anti-windup
            float excesso = rsp_uns - rate_setpoint;
            if (Ki_ang > 1e-4f && fabsf(excesso) > 1e-3f) {
                soma_ang -= KAW_BACK * excesso / Ki_ang;
                soma_ang = constrain(soma_ang, -I_LIMIT_ANG, I_LIMIT_ANG);
            }
        }

        // MALHA INTERNA (taxa -> PWM), 250 Hz

        float erro_rate = rate_setpoint - gyroRate_D;
        if (!gyro_saturado && fabsf(erro_rate) >= FREEZE_RATE) {
            soma_rate = constrain(soma_rate + erro_rate * dt, -I_LIMIT_RATE, I_LIMIT_RATE);
        }

        float P_r = (fabsf(erro_rate) > DEAD_RATE) ? Kp_rate * erro_rate : 0.0f;
        float I_r = Ki_rate * soma_rate;
        float gyroAccel = (gyroRate_D - prev_gyroRate_D) / dt;
        prev_gyroRate_D = gyroRate_D;
        float D_r = -Kd_rate * gyroAccel;
        D_r = constrain(D_r, -D_LIMIT, D_LIMIT);
        // anti-windup
        float pid_unsat = P_r + I_r + D_r;
        float pid_sat   = constrain(pid_unsat, -PID_LIMIT, PID_LIMIT);
        float excesso   = pid_unsat - pid_sat;
        if (!gyro_saturado && Ki_rate > 1e-4f && fabsf(excesso) > 1e-3f) {
            soma_rate -= KAW_BACK * excesso / Ki_rate;
            soma_rate = constrain(soma_rate, -I_LIMIT_RATE, I_LIMIT_RATE);
            I_r = Ki_rate * soma_rate;
        }
        pid_out = P_r + I_r + D_r;
        if (STICTION_KICK > 0.0f && !gyro_saturado) {
            float gyro_factor = constrain(1.0f - fabsf(gyroRate) / STICTION_GYRO_THRESH, 0.0f, 1.0f);
            float err_factor  = constrain((fabsf(erro_ang) - STICTION_ERR_THRESH) / STICTION_ERR_THRESH, 0.0f, 1.0f);
            float kick = STICTION_KICK * gyro_factor * err_factor;
            if (kick > 0.0f) pid_out += (erro_ang > 0.0f) ? kick : -kick;
        }

        if (fabsf(pid_out) >= PID_LIMIT) saturado = true;
        pid_out = constrain(pid_out, -PID_LIMIT, PID_LIMIT);

    } else {
        pararMotores();
        soma_ang = 0.0f; soma_rate = 0.0f;
        pid_out = 0.0f; rate_setpoint = 0.0f;
        pt2a.reset(); pt2b.reset();
        prev_gyroRate_D = 0.0f;
        setpoint_ativo = kX.angle;
    }

    lerSerial();

    // telemetria nao bloqueante (20 Hz). Campos compativeis com os scripts +
    // RSP (setpoint de taxa), Ia (integ. externo), Ir (integ. interno).
    if ((millis() - timer_print) > 50 && Serial.availableForWrite() >= 60) {
        float err_display = setpoint_ativo - angulo_real;
        Serial.print(F("T:"));    Serial.print(millis());
        Serial.print(F(" M1:"));  Serial.print(cmd_pwm1);
        Serial.print(F(" M2:"));  Serial.print(cmd_pwm2);
        Serial.print(F(" SP:"));  Serial.print(setpoint_ativo, 1);
        Serial.print(F(" Ang:")); Serial.print(angulo_real, 2);
        Serial.print(F(" AccA:"));Serial.print(accAngle, 2);
        Serial.print(F(" Err:")); Serial.print(err_display, 2);
        Serial.print(F(" RSP:")); Serial.print(rate_setpoint, 1);
        Serial.print(F(" GR:"));  Serial.print(gyroRate, 1);
        Serial.print(F(" Ia:"));  Serial.print(soma_ang, 1);
        Serial.print(F(" Ir:"));  Serial.print(soma_rate, 1);
        if (saturado) {
            if (fabsf(gyroRate) >= GYRO_SAT_DEG_S) Serial.print(F(" [GSAT]"));
            else                                     Serial.print(F(" [SAT]"));
        }
        Serial.println();
        timer_print = millis();
    }
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
    Wire.write(MPU_DLPF & 0x07);
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
        Wire.read(); Wire.read();              // TEMP (descartado)
        int16_t gx = Wire.read() << 8 | Wire.read();
        accX = ax; accY = ay; accZ = az; gyroX = gx;
        return true;
    }
    return false;
}

float calcularKalman(float newAngle, float newRate, float dt) {
    static bool inicializado = false;
    if (!inicializado) { kX.angle = newAngle; inicializado = true; }

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

void pararMotores() {
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
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

    switch (tipo) {
        //MALHA EXTERNA
        case 'A':
            if (valor >= 0.0f && valor <= 50.0f) {
                Kp_ang = valor; Serial.print(F(" Kp_ang=")); Serial.println(Kp_ang);
            } else Serial.println(F("ERR: A [0..50]"));
            break;
        case 'N':
            if (valor >= 0.0f && valor <= 5.0f) {
                Ki_ang = valor; soma_ang = 0.0f;
                Serial.print(F(" Ki_ang=")); Serial.println(Ki_ang);
            } else Serial.println(F("ERR: N [0..5]"));
            break;
        case 'Q':
            if (valor >= 30.0f && valor <= 500.0f) {
                MAX_RATE_SP = valor;
                Serial.print(F(" MAX_RATE_SP=")); Serial.print(MAX_RATE_SP, 0); Serial.println(F(" dps"));
            } else Serial.println(F("ERR: Q [30..500] dps"));
            break;

        //MALHA INTERNA
        case 'P':
            if (valor >= 0.0f && valor <= 20.0f) {
                Kp_rate = valor; Serial.print(F(" Kp_rate=")); Serial.println(Kp_rate);
            } else Serial.println(F("ERR: P [0..20]"));
            break;
        case 'I':
            if (valor >= 0.0f && valor <= 5.0f) {
                Ki_rate = valor; soma_rate = 0.0f;
                Serial.print(F(" Ki_rate=")); Serial.println(Ki_rate);
            } else Serial.println(F("ERR: I [0..5]"));
            break;
        case 'D':
            if (valor >= 0.0f && valor <= 2.0f) {
                Kd_rate = valor; Serial.print(F(" Kd_rate=")); Serial.println(Kd_rate);
            } else Serial.println(F("ERR: D [0..2]"));
            break;
        case 'E':
            if (valor >= 0.0f && valor <= 10.0f) {
                DEAD_RATE = valor;
                Serial.print(F(" DeadRate(int)=")); Serial.print(DEAD_RATE); Serial.println(F(" dps"));
            } else Serial.println(F("ERR: E [0..10]"));
            break;
        case 'H':
            if (valor >= 0.0f && valor <= 5.0f) {
                DEAD_ANG = valor;
                Serial.print(F(" DeadAng(ext)=")); Serial.print(DEAD_ANG); Serial.println(F(" deg"));
            } else Serial.println(F("ERR: H [0..5]"));
            break;
        case 'F':
            if (valor >= 1.0f && valor <= 120.0f) {
                PT2_DTERM_HZ = valor; pt2a.reset(); pt2b.reset();
                Serial.print(F(" Corte PT2 (D)=")); Serial.print(PT2_DTERM_HZ, 1); Serial.println(F(" Hz"));
            } else Serial.println(F("ERR: F [1..120] Hz"));
            break;
        case 'K':
            if (valor >= 0.0f && valor <= 80.0f) {
                STICTION_KICK = valor;
                Serial.print(F(" StictionKick=")); Serial.print(STICTION_KICK); Serial.println(F(" PWM"));
            } else Serial.println(F("ERR: K [0..80]"));
            break;
        case 'B':
            if (valor >= -200.0f && valor <= 200.0f) {
                balanceamento = (int)valor; Serial.print(F(" B=")); Serial.println(balanceamento);
            } else Serial.println(F("ERR: B [-200..200]"));
            break;
        case 'V':
            if (valor >= (float)VEL_MIN && valor <= (float)VEL_MAX) {
                VEL_BASE = (int)valor; Serial.print(F(" V=")); Serial.println(VEL_BASE);
            } else Serial.println(F("ERR: V [1100..1700]"));
            break;
        case 'C':
            if (valor >= 10.0f && valor <= 90.0f) {
                LIMITE_QUEDA = valor;
                Serial.print(F(" Failsafe=")); Serial.print(LIMITE_QUEDA, 0); Serial.println(F(" deg"));
            } else Serial.println(F("ERR: C [10..90]"));
            break;
        case 'R':
            if (valor >= 1.0f && valor <= 200.0f) {
                RAMP_DEGS = valor;
                Serial.print(F(" Rampa=")); Serial.print(RAMP_DEGS, 1); Serial.println(F(" deg/s"));
            } else Serial.println(F("ERR: R [1..200]"));
            break;
        case 'Z':
            if (valor >= 0.0f && valor <= 40.0f) {
                DITHER_AMP = valor;
                Serial.print(F(" Dither=")); Serial.print(DITHER_AMP, 1);
                Serial.print(F(" PWM (")); Serial.print(250.0f / (2.0f * DITHER_DIV), 1); Serial.println(F(" Hz)"));
            } else Serial.println(F("ERR: Z [0..40]"));
            break;
        case 'U':
            if (valor >= 1.0f && valor <= 30.0f) {
                DITHER_DIV = (uint8_t)valor;
                Serial.print(F(" Dither freq=")); Serial.print(250.0f / (2.0f * DITHER_DIV), 1); Serial.println(F(" Hz"));
            } else Serial.println(F("ERR: U [1..30]"));
            break;
        case 'M':
            if (valor >= 0.0f && valor <= 6.0f) {
                MPU_DLPF = (uint8_t)valor;
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(REG_CONFIG); Wire.write(MPU_DLPF & 0x07);
                Wire.endTransmission(true);
                Serial.print(F(" MPU DLPF=")); Serial.println(MPU_DLPF);
            } else Serial.println(F("ERR: M [0..6]"));
            break;
        case 'L':
            sistema_ligado = !sistema_ligado;
            if (sistema_ligado) {
                soma_ang = 0.0f; soma_rate = 0.0f;
                pid_out = 0.0f; rate_setpoint = 0.0f;
                pt2a.reset(); pt2b.reset(); prev_gyroRate_D = 0.0f;
                setpoint_ativo = kX.angle;
                Serial.print(F(" LIGADO | rampa "));
                Serial.print(kX.angle, 1); Serial.print(F(" -> "));
                Serial.print(setpoint_target, 1); Serial.println(F(" deg"));
                if (fabsf(setpoint_target - kX.angle) > 20.0f)
                    Serial.println(F("diferenca inicial > 20 deg. 'S' captura alvo no equilibrio."));
            } else {
                pararMotores(); Serial.println(F("DESLIGADO"));
            }
            break;
        case 'S':
            setpoint_target = kX.angle; setpoint_ativo = kX.angle;
            Serial.print(F("Alvo capturado: ")); Serial.print(setpoint_target, 2); Serial.println(F(" deg"));
            break;
        case 'T':
            if (valor >= -180.0f && valor <= 180.0f) {
                setpoint_target = valor;
                Serial.print(F(">> Alvo=")); Serial.print(setpoint_target, 1); Serial.println(F(" deg"));
            } else Serial.println(F("ERR: T [-180..180]"));
            break;

        case '?':
            Serial.println(F("=== PID CASCATA ==="));
            Serial.print(F(" [EXT] Kp_ang=")); Serial.print(Kp_ang);
            Serial.print(F(" Ki_ang=")); Serial.print(Ki_ang);
            Serial.print(F(" MAX_RATE_SP=")); Serial.print(MAX_RATE_SP, 0); Serial.println(F(" dps"));
            Serial.print(F(" [INT] Kp_rate=")); Serial.print(Kp_rate);
            Serial.print(F(" Ki_rate=")); Serial.print(Ki_rate);
            Serial.print(F(" Kd_rate=")); Serial.println(Kd_rate);
            Serial.print(F(" PT2(D)=")); Serial.print(PT2_DTERM_HZ, 1);
            Serial.print(F("Hz DeadAng=")); Serial.print(DEAD_ANG);
            Serial.print(F("deg DeadRate=")); Serial.print(DEAD_RATE); Serial.println(F("dps"));
            Serial.print(F(" Kick=")); Serial.print(STICTION_KICK);
            Serial.print(F(" Failsafe=")); Serial.print(LIMITE_QUEDA, 0);
            Serial.print(F(" Rampa=")); Serial.print(RAMP_DEGS, 1); Serial.println(F("deg/s"));
            Serial.print(F(" B=")); Serial.print(balanceamento);
            Serial.print(F(" V=")); Serial.print(VEL_BASE);
            Serial.print(F(" DLPF=")); Serial.print(MPU_DLPF);
            Serial.print(F(" Alvo=")); Serial.print(setpoint_target, 1);
            Serial.print(F(" Liga=")); Serial.println(sistema_ligado ? F("SIM") : F("NAO"));
            break;
        default:
            Serial.println(F("ERR: A/N/Q/P/I/D/E/H/F/K/B/V/C/R/Z/U/M/W/L/S/T<val>/?"));
            break;
    }
}
