#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <avr/wdt.h>

const int PIN_M1 = 9;
const int PIN_M2 = 10;
const uint8_t MPU_ADDR         = 0x68;  
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A; 
const uint8_t REG_ACCEL_XOUT_H = 0x3B;  
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const float   GYRO_LSB_PER_DPS = 131.0f;    
const float   ACC_LSB_PER_G    = 16384.0f;  
const uint32_t LOOP_US = 4000UL;

// VELOCIDADE BASE DOS MOTORES
int VEL_BASE = 1260;
const int VEL_MIN = 1100;
const int VEL_MAX = 1700;

// PID ÚNICO
float Kp     = 1.65f;  
float Ki     = 0.122f;
float Kd     = 0.60f;
float soma_e = 0.0f;

const float I_LIMIT  = 700.0f;
const float I_DECAY  = 0.9995f;
const float D_LIMIT  = 150.0f;
const float KAW_BACK = 0.2f;
const float ZONA_I_DEG = 2.0f;
const float FREEZE_ANG = 0.3f;
const float I_BAND_DEG = 12.0f;
float DEAD_ANG  = 0.4f;
float DEAD_RATE = 1.0f;
float STICTION_KICK = 15.0f;
const float STICTION_GYRO_THRESH = 15.0f;
const float STICTION_ERR_THRESH  = 3.0f;
float pid_out      = 0.0f;
int   balanceamento = -20;
bool  sistema_ligado = false;
float Kff = 0.0f;
float LIMITE_QUEDA = 45.0f;   
const uint16_t FALL_TRIP    = 50;      
const float R_MEASURE_BASE = 0.05f;   
const float ACC_GATE_TOL   = 0.10f;   
const float ACC_GATE_GAIN  = 8.0f;    
const float R_MEASURE_MAX  = 5.0f;   

//SETPOINT COM RAMPA 
float RAMP_DEGS  = 15.0f; 
float setpoint_target  = -30.0f;
float SP_WEIGHT_B = 1.0f; 
const float SP_LPF_HZ   = 0.5f; 

//VARIÁVEIS
Servo    m1, m2;
int16_t  accX, accY, accZ, gyroX;
float    gyroX_cal = 0.0f;
uint32_t timer_loop;
uint32_t timer_print;
uint8_t  MPU_DLPF = 0;

// FILTRO PT1
struct PT1Filter {
    float state = 0.0f;
    float update(float input, float dt, float cutoff_hz) {
        float RC = 1.0f / (2.0f * PI * cutoff_hz);
        float alpha = dt / (dt + RC);
        state += alpha * (input - state);
        return state;
    }
    void reset(float v = 0.0f) { state = v; }
};

float PT2_DTERM_HZ = 50.0f;
PT1Filter   pt2a, pt2b;
PT1Filter   sp_lpf;        

// FILTRO DE KALMAN
struct Kalman {
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.05f;
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{10.0f, 0.0f}, {0.0f, 0.1f}};
};
Kalman kX;

const uint8_t I2C_MAX_FALHAS = 10;

void setup() {
    wdt_enable(WDTO_2S);

    Serial.begin(250000);
    Wire.begin();
    Wire.setClock(400000);
    m1.attach(PIN_M1);
    m2.attach(PIN_M2);
    pararMotores();

    if (!inicializarMPU6050()) {
        Serial.println(F("ERRO MPU6050"));
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            wdt_reset();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    Serial.print(F("Calibrando gyro..."));
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
    Serial.print(F("Aquecendo Kalman..."));
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
    Serial.print(F("OK angulo="));
    Serial.print(kX.angle, 1);
    Serial.println(F(" deg"));
    setpoint_ativo = kX.angle;

    wdt_reset();
    timer_loop = micros();
    Serial.println(F("SISTEMA PRONTO - PID UNICO."));
    Serial.print(F("Alvo: "));
    Serial.print(setpoint_target);
    Serial.print(F(" deg  |  Atual: "));
    Serial.print(kX.angle, 1);
    Serial.println(F(" deg"));
    Serial.println(F("      L=liga/desliga  S=captura alvo  T<val>=alvo numerico"));
    Serial.println(F("      P<val>=Kp  I<val>=Ki  D<val>=Kd"));
    Serial.println(F("      E<val>=deadband_D  H<val>=deadband_P"));
    Serial.println(F("      K<val>=stiction_kick  B<val>=balance  V<val>=vel_base  ?=status"));
    Serial.println(F("      A<val>=feedforward  C<val>=failsafe_deg"));
    Serial.println(F("      F<Hz>=corte_filtro_D  R<deg/s>=rampa  J<0..1>=peso_setpoint_P"));
    Serial.println(F("      Z<PWM>=dither_amp  U<div>=dither_freq  M<0..6>=MPU_DLPF"));
}

void loop() {
    wdt_reset();
    uint32_t agora = micros();
    if ((agora - timer_loop) < LOOP_US) return;
    float dt = (float)(agora - timer_loop) / 1000000.0f;
    timer_loop = agora;
    dt = constrain(dt, 0.001f, 0.010f);

    {
        static uint8_t i2c_falhas = 0;
        if (!lerSensor()) {
            i2c_falhas++;
            if (i2c_falhas >= I2C_MAX_FALHAS) {
                Serial.println(F("WARN: I2C reset"));
                resetarI2C();
                delay(2);
                inicializarMPU6050();
                soma_e = 0.0f;
                i2c_falhas = 0;
                pararMotores();
                return;
            }
        } else {
            i2c_falhas = 0;
        }
    }

    float gyroRate    = ((float)gyroX - gyroX_cal) / GYRO_LSB_PER_DPS;
    float accAngle    = atan2f((float)accY,
                               sqrtf((float)accX * accX + (float)accZ * accZ)) * 180.0f / PI;.
    float acc_mag    = sqrtf((float)accX * accX + (float)accY * accY + (float)accZ * accZ);
    float acc_desvio = fabsf(acc_mag - ACC_LSB_PER_G) / ACC_LSB_PER_G;
    float r_extra    = acc_desvio - ACC_GATE_TOL;
    kX.R_measure     = (r_extra <= 0.0f) ? R_MEASURE_BASE : 
    constrain(R_MEASURE_BASE * (1.0f + ACC_GATE_GAIN * r_extra), R_MEASURE_BASE, R_MEASURE_MAX);
    float angulo_real = calcularKalman(accAngle, gyroRate, dt);
    float gyroRate_D1 = pt2a.update(gyroRate, dt, PT2_DTERM_HZ);
    float gyroRate_D  = pt2b.update(gyroRate_D1, dt, PT2_DTERM_HZ);
  //taxa de variacao do setpoint feedforward
    float sp_rate = 0.0f;   
    if (sistema_ligado) {
        float sp_anterior = setpoint_ativo;
        float delta_max   = RAMP_DEGS * dt;
        float diff        = setpoint_target - setpoint_ativo;
        if      (diff >  delta_max) setpoint_ativo += delta_max;
        else if (diff < -delta_max) setpoint_ativo -= delta_max;
        else                        setpoint_ativo  = setpoint_target;
        sp_rate = (setpoint_ativo - sp_anterior) / dt;
    }
    bool saturado = false;
    int  cmd_pwm1 = 1000; 
    int  cmd_pwm2 = 1000;
    if (sistema_ligado) {

        float erro = setpoint_ativo - angulo_real;

        //failsafe de inclinacao
        // LIMITE_QUEDA
        static uint16_t fall_count = 0;
        if (fabsf(erro) > LIMITE_QUEDA) {
            if (++fall_count >= FALL_TRIP) {
                sistema_ligado = false;
                pararMotores();
                soma_e     = 0.0f;
                pid_out    = 0.0f;
                fall_count = 0;
                Serial.print(F(">> FAILSAFE: |erro| > "));
                Serial.print(LIMITE_QUEDA, 0);
                Serial.println(F(" deg. Sistema DESARMADO."));
                return;
            }
        } else {
            fall_count = 0;
        }

        bool gyro_saturado = fabsf(gyroRate) >= GYRO_SAT_DEG_S;
        if (gyro_saturado) saturado = true;

        float abs_erro_i = fabsf(erro);
        if (!gyro_saturado) {
            if (abs_erro_i > I_BAND_DEG) {
                // erro grande: anti-windup
                soma_e *= I_DECAY;
            } else if (abs_erro_i >= FREEZE_ANG) {
                if (abs_erro_i < ZONA_I_DEG) {
                    soma_e *= I_DECAY;
                }
                soma_e = constrain(soma_e + erro * dt, -I_LIMIT, I_LIMIT);
            }
        }
        float sp_lento = sp_lpf.update(setpoint_ativo, dt, SP_LPF_HZ);
        float erro_p   = erro - (1.0f - SP_WEIGHT_B) * (setpoint_ativo - sp_lento);
        float P_t = (fabsf(erro) > DEAD_ANG) ? Kp * erro_p : 0.0f;
        float I_t = Ki * soma_e;
        float D_t;
        float abs_gr = fabsf(gyroRate_D);
        if (abs_gr <= DEAD_RATE) {
            D_t = 0.0f;
        } else if (abs_gr >= 2.0f * DEAD_RATE) {
            D_t = -Kd * gyroRate_D;
        } else {
            float fade = (abs_gr - DEAD_RATE) / DEAD_RATE;
            D_t = -Kd * gyroRate_D * fade;
        }
        D_t = constrain(D_t, -D_LIMIT, D_LIMIT);
        float pid_unsat = P_t + I_t + D_t;
        float pid_sat   = constrain(pid_unsat, -PID_LIMIT, PID_LIMIT);
        float excesso   = pid_unsat - pid_sat;
        if (!gyro_saturado && Ki > 0.0001f && fabsf(excesso) > 0.001f) {
            soma_e -= KAW_BACK * excesso / Ki;
            soma_e = constrain(soma_e, -I_LIMIT, I_LIMIT);
            I_t = Ki * soma_e;
        }
        pid_out = P_t + I_t + D_t;
        //feedforward de setpoint
        pid_out += Kff * sp_rate;
        if (STICTION_KICK > 0.0f && sistema_ligado && !gyro_saturado) {
            float gyro_factor = 1.0f - fabsf(gyroRate) / STICTION_GYRO_THRESH;
            gyro_factor = constrain(gyro_factor, 0.0f, 1.0f);

            float err_factor = (fabsf(erro) - STICTION_ERR_THRESH) / STICTION_ERR_THRESH;
            err_factor = constrain(err_factor, 0.0f, 1.0f);

            float kick = STICTION_KICK * gyro_factor * err_factor;
            if (kick > 0.0f) {
                pid_out += (erro > 0.0f) ? kick : -kick;
            }
        }
        if (fabsf(pid_out) >= PID_LIMIT) saturado = true;
        pid_out = constrain(pid_out, -PID_LIMIT, PID_LIMIT);
    lerSerial();

    if ((millis() - timer_print) > 50 && Serial.availableForWrite() >= 60) {
        float err_display = setpoint_ativo - angulo_real;
        Serial.print(F("T:"));    Serial.print(millis());
        Serial.print(F(" M1:"));  Serial.print(cmd_pwm1);
        Serial.print(F(" M2:"));  Serial.print(cmd_pwm2);
        Serial.print(F(" SP:"));  Serial.print(setpoint_ativo, 1);
        Serial.print(F(" Ang:")); Serial.print(angulo_real, 2);
        Serial.print(F(" AccA:")); Serial.print(accAngle, 2);   // angulo do acelerometro CRU (diag estimacao)
        Serial.print(F(" Err:")); Serial.print(err_display, 2);
        Serial.print(F(" GR:"));  Serial.print(gyroRate, 1);
        Serial.print(F(" I:"));   Serial.print(soma_e, 1);
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
       // sai do modo de repouso
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
         // TEMP_OUT
        Wire.read(); Wire.read();              
        int16_t gx = Wire.read() << 8 | Wire.read();
        accX = ax; accY = ay; accZ = az; gyroX = gx;
        return true;
    }
    return false;
}
//kalman
float calcularKalman(float newAngle, float newRate, float dt) {
    static bool inicializado = false;
    if (!inicializado) {
        kX.angle = newAngle;
        inicializado = true;
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

        case 'A':
            if (valor >= 0.0f && valor <= 50.0f) { Kp_ang = valor; Serial.print(F(">> Kp_ang=")); Serial.println(Kp_ang); }
            else Serial.println(F("ERR: A [0..50]"));
            break;

        case 'N':
            if (valor >= 0.0f && valor <= 5.0f) { Ki_ang = valor; soma_ang = 0.0f; Serial.print(F(">> Ki_ang=")); Serial.println(Ki_ang); }
            else Serial.println(F("ERR: N [0..5]"));
            break;

        // Loop interno
        case 'P':
            if (valor >= 0.0f && valor <= 20.0f) { Kp_rate = valor; Serial.print(F(">> Kp_rate=")); Serial.println(Kp_rate); }
            else Serial.println(F("ERR: P [0..20]"));
            break;

        case 'I':
            if (valor >= 0.0f && valor <= 5.0f) { Ki_rate = valor; soma_rate = 0.0f; Serial.print(F(">> Ki_rate=")); Serial.println(Ki_rate); }
            else Serial.println(F("ERR: I [0..5]"));
            break;

        case 'D':
            if (valor >= 0.0f && valor <= 2.0f) { Kd_rate = valor; Serial.print(F(">> Kd_rate=")); Serial.println(Kd_rate); }
            else Serial.println(F("ERR: D [0..2]"));
            break;

        //Filtro notch
        case 'F':
            //configura notch
            if (valor >= 0.0f && valor < 125.0f) {
                NOTCH_HZ = valor;
                notchFilter.configure(NOTCH_HZ, NOTCH_Q, 250.0f);
                if (NOTCH_HZ > 0.0f) { Serial.print(F(">> Notch=")); Serial.print(NOTCH_HZ); Serial.print(F(" Hz Q=")); Serial.println(NOTCH_Q); }
                else                   Serial.println(F(">> Notch desabilitado"));
            } else Serial.println(F("ERR: F [0..124]"));
            break;

        case 'G':
            //Fator de qualidade do notch
            if (valor >= 0.5f && valor <= 20.0f) {
                NOTCH_Q = valor;
                notchFilter.configure(NOTCH_HZ, NOTCH_Q, 250.0f);
                Serial.print(F(">> Notch Q=")); Serial.print(NOTCH_Q);
                Serial.print(F(" @ ")); Serial.print(NOTCH_HZ); Serial.println(F(" Hz"));
            } else Serial.println(F("ERR: G [0.5..20]"));
            break;

        //Dead zone do P interno
        case 'E':
            if (valor >= 0.0f && valor <= 10.0f) { DEAD_RATE = valor; Serial.print(F(">> DeadRate=")); Serial.print(DEAD_RATE); Serial.println(F(" dps")); }
            else Serial.println(F("ERR: E [0..10]"));
            break;

        //Sistema
        case 'B':
            if (valor >= -200.0f && valor <= 200.0f) { balanceamento = (int)valor; Serial.print(F(">> B=")); Serial.println(balanceamento); }
            else Serial.println(F("ERR: B [-200..200]"));
            break;

        case 'V':
            if (valor >= (float)VEL_MIN && valor <= (float)VEL_MAX) { VEL_BASE = (int)valor; Serial.print(F(">> V=")); Serial.println(VEL_BASE); }
            else { Serial.print(F("ERR: V [")); Serial.print(VEL_MIN); Serial.print(F("..")); Serial.print(VEL_MAX); Serial.println(F("]")); }
            break;

        case 'L':
            sistema_ligado = !sistema_ligado;
            if (sistema_ligado) {
                soma_ang        = 0.0f;
                soma_rate       = 0.0f;
                pid_out         = 0.0f;
                rate_setpoint   = 0.0f;
                pt2a.reset();
                pt2b.reset();
                notchFilter.reset();
                prev_gyroRate_D = 0.0f;
                setpoint_ativo  = kX.angle;
                Serial.print(F(">> LIGADO | rampa ")); Serial.print(kX.angle, 1);
                Serial.print(F(" -> ")); Serial.print(setpoint_target, 1); Serial.println(F(" deg"));
            } else {
                pararMotores();
                Serial.println(F(">> DESLIGADO"));
            }
            break;

        case 'S':
            setpoint_target = kX.angle;
            setpoint_ativo  = kX.angle;
            Serial.print(F(">> Novo alvo capturado: ")); Serial.print(setpoint_target, 2); Serial.println(F(" deg"));
            break;

        case 'T':
            if (valor >= -180.0f && valor <= 180.0f) {
                setpoint_target = valor;
                Serial.print(F(">> Alvo=")); Serial.print(setpoint_target, 1); Serial.println(F(" deg"));
            } else Serial.println(F("ERR: T [-180..180]"));
            break;

        case '?':
            Serial.print(F("Kp_ang=")); Serial.print(Kp_ang);
            Serial.print(F("Ki_ang="));Serial.println(Ki_ang);
            Serial.print(F("Kp_rate=")); Serial.print(Kp_rate);
            Serial.print(F("Ki_rate="));Serial.print(Ki_rate);
            Serial.print(F("Kd_rate="));Serial.println(Kd_rate);
            Serial.print(F(">> [Flt] Notch="));Serial.print(NOTCH_HZ);
            Serial.print(F("Q="));Serial.print(NOTCH_Q);
            Serial.print(F("PT2="));Serial.print(PT2_DTERM_HZ);
            Serial.print(F("DeadRate="));Serial.print(DEAD_RATE);
            Serial.print(F("B="));Serial.print(balanceamento);
            Serial.print(F("V="));Serial.print(VEL_BASE);
            Serial.print(F("Alvo=")); Serial.print(setpoint_target, 1);
            Serial.print(F("Liga=")); Serial.println(sistema_ligado ? F("SIM") : F("NAO"));
            break;
        default:
            Serial.println(F("ERR: Use A/N/P/I/D/F/G/E/B/V/L/S/T<val>/?"));
            break;
    }
}
