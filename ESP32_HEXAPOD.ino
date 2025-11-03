#include <INA226_WE.h>
#include <U8g2lib.h>
#include <MPU6050.h>
#include <ps5Controller.h>
#include <EEPROM.h>

#define EEPROM_SIZE 400  

ps5Controller PS5;

// =================== Pantalla OLED ===================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// =================== INA226 ===================
#define I2C_ADDRESS_INA226 0x40
INA226_WE ina226(I2C_ADDRESS_INA226);

// =================== MPU6050 ===================
MPU6050 mpu;
int16_t ax, ay, az;

// =================== UART ===================
#define UART_TX 17
#define UART_RX 16
HardwareSerial SerialRobot(1);

// =================== GPIO ===================
#define GPIO_CONTROL 13
#define BOTON_PIN 27

// =================== Estados ===================
const float VOLTAJE_MIN = 9.8;
const float VOLTAJE_CORTE_POR_SEGURIDAD = 9.5;
const float VOLTAJE_MAX = 12.6;

bool found_INA226 = false, found_OLED = false, found_MPU = false;
bool emparejando = false, conectado = false;

//calibracion servos
bool CALIBRANDO, CALIBRANDOprev;

//UART comprobacion si son diferentes para enviar o no
int lxPrev = 0, lyPrev = 0, rxPrev = 0, ryPrev = 0, L2ValPrev = 0, R2ValPrev = 0, PitchPrev = 0, RollPrev = 0;
bool conectadoPrev = false, modoActivoPrev = false, RIGHTprev = false, LEFTprev = false, UPprev = false, DOWNprev = false, SQUAREprev = false, CROSSprev = false, CIRCLEprev = false, 
     TRIANGLEprev = false, L1prev = false, R1prev = false, L2prev = false, R2prev = false, SHAREprev = false, OPTIONSprev = false, L3prev = false, R3prev = false, PSButtonPrev = false, 
     TouchpadPrev = false;


// ===================== VARIABLES EEPROM =========================
  #define MaxVariables 100

  float Variable[MaxVariables+1];
  float VariablePrevEEPROAM[MaxVariables+1];
  float VariablePrevENVIOS[MaxVariables+1];


// =================== RECIBIDO DESDE UART ==================
struct Recivo {
bool LEVANTADO;
bool ACOSTADO;
bool CALIBRANDO;
int ALTURA;
int Yaw;
int Pitch;
int Roll;
};
Recivo DesdeServo2040;



#define FlancoSubida(entrada) ({                \
   static bool anterior = false;                \
   bool flanco = (!anterior && (entrada));      \
   anterior = (entrada);                        \
   flanco; })

#define FlancoBajada(entrada) ({                \
   static bool anterior = false;                \
   bool flanco = (anterior && !(entrada));      \
   anterior = (entrada);                        \
   flanco; })   
   
// =================== FUNCIONES ===================

// =================== TEMPORIZADOR ===================
#define MAX_TIMERS 10  // Cambia si necesitas más
unsigned long tiempos[MAX_TIMERS] = {0};  // Array de tiempos anteriores
bool TimerEvery(byte id, unsigned long intervalo) {
  if (id >= MAX_TIMERS) return false;
  unsigned long ahora = millis();
  if (ahora - tiempos[id] >= intervalo) {
    tiempos[id] = ahora;
    return true;
  }
  return false;
}


// ===================== TIMERS TON ======================
struct TimerOFF {
  unsigned long t = 0;
  bool activo = false;

  void set(unsigned long d) {
    t = millis() + d;
    activo = true;
  }

  bool ON() {
    if (activo && millis() > t) {
      activo = false;
    }
    return activo;
  }
};
TimerOFF TOFF[10];  // usa toff[1], toff[2], ...


// =================== INICIALIZAR INA226 ===================
void inicializarINA226() {
  ina226.init();
  ina226.setResistorRange(0.002, 8.0);
  ina226.setAverage(AVERAGE_16);
  ina226.setConversionTime(CONV_TIME_588);
  ina226.setMeasureMode(CONTINUOUS);
}

// =================== ESCANEAR I2C ===================
void escanearI2C() {
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (address == 0x40) found_INA226 = true;
      if (address == 0x68 || address == 0x69) found_MPU = true;
      if (address == 0x3C || address == 0x3D) found_OLED = true;
    }
  }
}

// =================== LOOP: COMPROBAR BATERÍA ===================
void comprobarVoltsyAmps() {
  float vbus = ina226.getBusVoltage_V();
  float i = ina226.getCurrent_A();
  if (vbus >= VOLTAJE_MIN && vbus <= VOLTAJE_MAX) {TOFF[1].set(500);}
  if (i <= 2.5) {TOFF[2].set(1200);} 

  if (TOFF[1].ON() && TOFF[2].ON() && vbus >= VOLTAJE_CORTE_POR_SEGURIDAD) 
{digitalWrite(GPIO_CONTROL,HIGH);}else{digitalWrite(GPIO_CONTROL,LOW);}
}

// --- TODO en una única función ---
void filtrarYEnviarMPU() {

// ====================== CONFIG AJUSTABLE MPU → UART ======================
#define MPU_FC_HZ          0.5f
#define MPU_STEP_DEG       0.5f
#define MPU_ON_TH_FACTOR   1.5f
#define MPU_OFF_TH_FACTOR  0.4f
#define MPU_SEND_DECIMALS  1
#define MPU_DEC_PLACES     1
#define MPU_CLAMP_MIN     -45.0f
#define MPU_CLAMP_MAX      45.0f
#define MPU_RATE_LIMIT_MS  100

// ====== AJUSTES anti-golpes / complementario ======
#define ACC_LSB_PER_G            16384.0f   // MPU6050 ±2g
#define GYRO_LSB_PER_DPS          131.0f    // MPU6050 ±250 dps
#define CF_A_LOW                  0.94f
#define CF_A_HIGH                 0.995f
#define ACC_DEV_LO                0.03f
#define ACC_DEV_HI                0.20f
#define SHOCK_DEV_THRESHOLD       0.35f
#define SHOCK_HOLD_MS             80
#define ACC_MAX_DEG_PER_SAMPLE    2.0f

// Signos (ajústalos si tu IMU está montada distinto)
#define GYRO_SIGN_PITCH           (+1.0f)
#define GYRO_SIGN_ROLL            (+1.0f)   // Ojo: ahora probamos +1.0f para roll

// ====== PARÁMETROS DE SALIDA ======
#define MPU_OUT_SWAP_PR           0         // 0 = NO swappear (recomendado)
#define MPU_OUT_INV_PITCH         0
#define MPU_OUT_INV_ROLL          0
// ===================================


  // ===== 1) Lectura cruda y dt =====
  int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
  mpu.getAcceleration(&ax_i, &ay_i, &az_i);
  mpu.getRotation(&gx_i, &gy_i, &gz_i);

  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  float dt = (lastUs==0) ? 0.025f : (nowUs - lastUs) / 1e6f;
  lastUs = nowUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f)  dt = 0.05f;

  // ===== 2) Calibración de bias del gyro (una sola vez, en reposo) =====
  static bool   initOnce = false;
  static float  gxBias=0, gyBias=0, gzBias=0;
  static float  pitchCF=0, rollCF=0;
  static uint32_t shockUntilMs = 0;
  if (!initOnce) {
    const int N = 200;
    long sx=0, sy=0, sz=0;
    int16_t tx, ty, tz;
    for (int i=0;i<N;i++){ mpu.getRotation(&tx,&ty,&tz); sx+=tx; sy+=ty; sz+=tz; delay(2); }
    gxBias = (float)sx/N; gyBias = (float)sy/N; gzBias = (float)sz/N;

    float axf=(float)ax_i, ayf=(float)ay_i, azf=(float)az_i;
    float pitchAcc0 = atan2f(-axf, sqrtf(ayf*ayf + azf*azf)) * 180.0f / M_PI;
    float rollAcc0  = atan2f( ayf,             azf         ) * 180.0f / M_PI;
    pitchCF = pitchAcc0;  rollCF = rollAcc0;
    initOnce = true;
  }

  // ===== 3) Convierte a unidades útiles =====
  float axf = (float)ax_i, ayf = (float)ay_i, azf = (float)az_i;
  float gxf = ((float)gx_i - gxBias) / GYRO_LSB_PER_DPS; // deg/s  (gyro X)
  float gyf = ((float)gy_i - gyBias) / GYRO_LSB_PER_DPS; // deg/s  (gyro Y)
  // float gzf = ((float)gz_i - gzBias) / GYRO_LSB_PER_DPS;

  // ===== 4) Ángulos por ACC + LPF configurable =====
  auto alphaFromHz = [](float fc, float dt){
    float x = 2.0f * M_PI * fc * dt;
    return 1.0f - expf(-x);
  };
  static float pitchAccLPF=0, rollAccLPF=0;
  float pitchAcc = atan2f(-axf, sqrtf(ayf*ayf + azf*azf)) * 180.0f / M_PI;
  float rollAcc  = atan2f( ayf,             azf         ) * 180.0f / M_PI;
  float aLPF = alphaFromHz(MPU_FC_HZ, dt);
  pitchAccLPF += aLPF * (pitchAcc - pitchAccLPF);
  rollAccLPF  += aLPF * (rollAcc  - rollAccLPF);

  // ===== 5) Detector de golpes / confianza ACC =====
  float aNorm = sqrtf(axf*axf + ayf*ayf + azf*azf) / ACC_LSB_PER_G; // en g
  float dev   = fabsf(aNorm - 1.0f);
  auto map01  = [](float x, float lo, float hi){
    float t = (x - lo) / (hi - lo); if (t<0) t=0; if (t>1) t=1; return 1.0f - t;
  };
  float accTrust = map01(dev, ACC_DEV_LO, ACC_DEV_HI);  // 1=fiable, 0=no
  uint32_t nowMs = millis();
  if (dev > SHOCK_DEV_THRESHOLD) shockUntilMs = nowMs + SHOCK_HOLD_MS;
  bool inShockHold = (nowMs < shockUntilMs);

  // ===== 6) Complementario gyro+acc con peso dinámico =====
  // *** MAPEADO CORRECTO: roll ← gyro X, pitch ← gyro Y ***
  float rollGyro  = rollCF  + (GYRO_SIGN_ROLL  * gxf) * dt;  // ROLL  con gyro X
  float pitchGyro = pitchCF + (GYRO_SIGN_PITCH * gyf) * dt;  // PITCH con gyro Y

  float aComp = CF_A_LOW + (1.0f - accTrust) * (CF_A_HIGH - CF_A_LOW);
  if (inShockHold) aComp = CF_A_HIGH;

  auto clamp = [](float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); };
  float dRA = clamp( rollAccLPF -  rollGyro, -ACC_MAX_DEG_PER_SAMPLE, ACC_MAX_DEG_PER_SAMPLE);
  float dPA = clamp(pitchAccLPF - pitchGyro, -ACC_MAX_DEG_PER_SAMPLE, ACC_MAX_DEG_PER_SAMPLE);
  float rollAccCorr  = rollGyro  + dRA;
  float pitchAccCorr = pitchGyro + dPA;

  rollCF  = aComp * rollGyro  + (1.0f - aComp) * rollAccCorr;
  pitchCF = aComp * pitchGyro + (1.0f - aComp) * pitchAccCorr;

  // ===== 7) Cuantiza + límites =====
  auto quantize = [](float v, float s){ return (s<=0)?v : roundf(v/s)*s; };
  float rOut = quantize(rollCF,  MPU_STEP_DEG);
  float pOut = quantize(pitchCF, MPU_STEP_DEG);
  rOut = constrain(rOut, MPU_CLAMP_MIN, MPU_CLAMP_MAX);
  pOut = constrain(pOut, MPU_CLAMP_MIN, MPU_CLAMP_MAX);

  // ===== 7.1) SALIDA: swap/invert opcional =====
#if MPU_OUT_SWAP_PR
  { float tmp = pOut; pOut = rOut; rOut = tmp; }
#endif
#if MPU_OUT_INV_PITCH
  pOut = -pOut;
#endif
#if MPU_OUT_INV_ROLL
  rOut = -rOut;
#endif

  // ===== 8) Histéresis + rate-limit + envío =====
  static float lastSentP = 0.0f, lastSentR = 0.0f;
  static bool  pAct=false, rAct=false;
  const float ON_TH  = MPU_STEP_DEG * MPU_ON_TH_FACTOR;
  const float OFF_TH = MPU_STEP_DEG * MPU_OFF_TH_FACTOR;
  auto schmitt = [](float e, bool &act, float on, float off){
    if (!act) { if (fabsf(e) >= on) { act = true;  return true; } return false; }
    else      { if (fabsf(e) <= off){ act = false; return false;} return true; }
  };
  bool sendP = schmitt(pOut - lastSentP, pAct, ON_TH, OFF_TH);
  bool sendR = schmitt(rOut - lastSentR, rAct, ON_TH, OFF_TH);

  static uint32_t lastSendMsP = 0, lastSendMsR = 0;
  if (sendP && (nowMs - lastSendMsP >= MPU_RATE_LIMIT_MS)) {
    SerialRobot.print("PIT,");
  #if MPU_SEND_DECIMALS
    char buf[16]; dtostrf(pOut, 0, MPU_DEC_PLACES, buf);
    SerialRobot.print(buf);
  #else
    SerialRobot.print((int)lroundf(pOut));
  #endif
    SerialRobot.println("+");
    lastSentP   = pOut;
    lastSendMsP = nowMs;
  }

  if (sendR && (nowMs - lastSendMsR >= MPU_RATE_LIMIT_MS)) {
    SerialRobot.print("ROL,");
  #if MPU_SEND_DECIMALS
    char buf[16]; dtostrf(rOut, 0, MPU_DEC_PLACES, buf);
    SerialRobot.print(buf);
  #else
    SerialRobot.print((int)lroundf(rOut));
  #endif
    SerialRobot.println("+");
    lastSentR   = rOut;
    lastSendMsR = nowMs;
  }
}

// =================== MANDO CONECTADO O DESCONECTADO ===================
void ps5MandoConectado() {conectado = true;}
void ps5MandoDesconectado() {conectado = false;}

// =================== SETUP ===================
void iniciarSistema() {
  Serial.begin(250000);
  SerialRobot.begin(250000, SERIAL_8N1, UART_RX, UART_TX);
  Wire.begin();
  pinMode(GPIO_CONTROL, OUTPUT);
  digitalWrite(GPIO_CONTROL, LOW);
  pinMode(BOTON_PIN, INPUT_PULLUP);
  u8g2.begin();
  mpu.initialize();
  inicializarINA226();
  comprobarVoltsyAmps();
  PS5.attachOnConnect(ps5MandoConectado);
  PS5.attachOnDisconnect(ps5MandoDesconectado);
}

// =================== ENVIAR DATOS UART ===================
void enviarDatosUART() {
  const int TMA = 30;                   //Tamaño Maximo Array
  const int NumMaxMensajesXciclo = 2;   //Numero maximo de mensajes a enviar por ciclo

  static bool EnvioInicial = true;
  static char mensajePendiente[TMA+2][32];
  static char mensajePendientePrev[NumMaxMensajesXciclo][32];
  static int Num = 0;

  #define GuardarDatoINT(patron, variable, prev)                                                     \
    do {                                                                                             \
    if (Num < TMA) {                                                                                 \
    snprintf(mensajePendiente[Num], sizeof(mensajePendiente[Num]),"%s,%d+\n", patron, variable);     \
    prev = variable;                                                                                 \
    Num++;                                                                                           \
    }                                                                                                \
  } while(0)                                                                                         \

  #define GuardarDatoFloat(patron, variable, prev) \
  do { \
    if (Num < TMA) { \
      snprintf(mensajePendiente[Num], sizeof(mensajePendiente[Num]), "%s,%.2f+\n", patron, variable); \
      prev = variable; \
      Num++; \
    } \
  } while(0)        


  int lx = PS5.LStickX();
  int ly = PS5.LStickY();
  int rx = PS5.RStickX();
  int ry = PS5.RStickY();
  int L2 = PS5.L2Value();
  int R2 = PS5.R2Value();

if (!EnvioInicial) {

  // ---- Joysticks analógicos (solo si cambian) ----
    if (abs(lx - lxPrev) >= 2) { GuardarDatoINT("JIX", lx, lxPrev); }
    if (abs(ly - lyPrev) >= 2) { GuardarDatoINT("JIY", ly, lyPrev); }
    if (abs(rx - rxPrev) >= 2) { GuardarDatoINT("JDX", rx, rxPrev); }
    if (abs(ry - ryPrev) >= 2) { GuardarDatoINT("JDY", ry, ryPrev); }

    // ---- Gatillos L2/R2 ----
    if (abs(L2 - L2ValPrev) >= 2) { GuardarDatoINT("L2V", L2, L2ValPrev); }
    if (abs(R2 - R2ValPrev) >= 2) { GuardarDatoINT("R2V", R2, R2ValPrev); }

    // ---- Estados/botones ----
    if (conectado != conectadoPrev)      { GuardarDatoINT("ON",  conectado, conectadoPrev); }
    if (PS5.Up()       != UPprev)        { GuardarDatoINT("UP",  PS5.Up(),       UPprev); }
    if (PS5.Down()     != DOWNprev)      { GuardarDatoINT("DN",  PS5.Down(),     DOWNprev); }
    if (PS5.Left()     != LEFTprev)      { GuardarDatoINT("LF",  PS5.Left(),     LEFTprev); }
    if (PS5.Right()    != RIGHTprev)     { GuardarDatoINT("RT",  PS5.Right(),    RIGHTprev); }

    if (PS5.Square()   != SQUAREprev)    { GuardarDatoINT("SQ",  PS5.Square(),   SQUAREprev); }
    if (PS5.Cross()    != CROSSprev)     { GuardarDatoINT("CR",  PS5.Cross(),    CROSSprev); }
    if (PS5.Circle()   != CIRCLEprev)    { GuardarDatoINT("CI",  PS5.Circle(),   CIRCLEprev); }
    if (PS5.Triangle() != TRIANGLEprev)  { GuardarDatoINT("TR",  PS5.Triangle(), TRIANGLEprev); }

    if (PS5.L1()       != L1prev)        { GuardarDatoINT("L1",  PS5.L1(),       L1prev); }
    if (PS5.R1()       != R1prev)        { GuardarDatoINT("R1",  PS5.R1(),       R1prev); }
    if (PS5.L2()       != L2prev)        { GuardarDatoINT("L2",  PS5.L2(),       L2prev); }
    if (PS5.R2()       != R2prev)        { GuardarDatoINT("R2",  PS5.R2(),       R2prev); }
    if (PS5.Share()    != SHAREprev)     { GuardarDatoINT("SH",  PS5.Share(),    SHAREprev); }
    if (PS5.Options()  != OPTIONSprev)   { GuardarDatoINT("OP",  PS5.Options(),  OPTIONSprev); }
    if (PS5.L3()       != L3prev)        { GuardarDatoINT("L3",  PS5.L3(),       L3prev); }
    if (PS5.R3()       != R3prev)        { GuardarDatoINT("R3",  PS5.R3(),       R3prev); }
    if (PS5.PSButton() != PSButtonPrev)  { GuardarDatoINT("PS",  PS5.PSButton(), PSButtonPrev); }
    if (PS5.Touchpad() != TouchpadPrev)  { GuardarDatoINT("TP",  PS5.Touchpad(), TouchpadPrev); }
    if (CALIBRANDO     != CALIBRANDOprev){ GuardarDatoINT("CALI", CALIBRANDO, CALIBRANDOprev); }


    // ENVIO DE VARIABLES GUARDADADES EN LA EEPROM DE LA SERVO2040
  for (int i = 0; i < MaxVariables; i++) {
    static char etiqueta[20];
    sprintf(etiqueta, "EEPROM%d", i);
    if (Variable[i] != VariablePrevENVIOS[i]) {GuardarDatoFloat(etiqueta, Variable[i], VariablePrevENVIOS[i]);}
  }
}


    //ENVIO INICIAL DE TODAS LAS VARIAVBLES GUARDADADES EN LA EEPROM DE LA SERVO2040
  if (EnvioInicial) {
    static int i = 0;
    static char etiqueta[20];
      sprintf(etiqueta, "EEPROM%d", i);
      GuardarDatoFloat(etiqueta, Variable[i], VariablePrevENVIOS[i]);
      i++;
      if (i > MaxVariables) {EnvioInicial = false;}      
  }

  //ENVIO DE DATOS UART SOLO SI SON DIFERENTES A LOS ANTERIORES
  for (int i = 0; i < NumMaxMensajesXciclo; ++i) {
    if (strcmp(mensajePendiente[i], mensajePendientePrev[i]) != 0) {
      SerialRobot.print(mensajePendiente[i]);    
      strcpy(mensajePendientePrev[i], mensajePendiente[i]);
    }
  }


  // desplaza [1..Num-1] -> [0..Num-2]
  for (int i = NumMaxMensajesXciclo; i < Num; ++i) {
    strcpy(mensajePendiente[i - NumMaxMensajesXciclo], mensajePendiente[i]);
  }

  
  if (Num > 0) Num--;
}

// =================== RECIBIR DESDE UART ===================
void recibirYprocesarDesdeUART() {

  // Mantener entre llamadas
  static std::string buffer = "";
  static std::string RecivoBuffer = "";

  while (SerialRobot.available()) {

    // PROCESADOR DE LÍNEAS
{
    char c = (char)SerialRobot.read();
    if (c == '\r') continue;

    if (c != '\n') {
      if (buffer.size() < 127) buffer.push_back(c);
      else buffer.clear();  // overflow -> descartamos línea corrupta
      continue;
    }

    // c == '\n' -> tenemos una línea completa
    RecivoBuffer = buffer;
    buffer.clear();

    // ----- PARSEO DE ESTA LINEA -----

    // Compara "prefijo," al inicio de la línea
    #undef Comparar
    #define Comparar(LINEA, CLAVE) \
      (strncmp((LINEA).c_str(), (CLAVE), sizeof(CLAVE)-1) == 0 && (LINEA).size() > (sizeof(CLAVE)-1) && (LINEA)[sizeof(CLAVE)-1] == ',')

    // Extrae int o float (sin exigir '+', funciona igual con o sin él)
    #undef ExtraerINT
    #define ExtraerINT(LINEA, DEST) do{ \
      int _tmp=0; \
      if (sscanf((LINEA).c_str(), "%*[^,],%d", &_tmp) == 1) (DEST) = _tmp; \
    }while(0)

    #undef ExtraerFLOAT
    #define ExtraerFLOAT(LINEA, DEST) do{ \
      float _tmp=0; \
      if (sscanf((LINEA).c_str(), "%*[^,],%f", &_tmp) == 1) (DEST) = _tmp; \
    }while(0)

  }
    
    // PROCESAR COMANDOS DE ESTA LINEA (una sola pasada por línea)
       if (Comparar(RecivoBuffer, "LEVANTADO"))   { ExtraerINT(RecivoBuffer, DesdeServo2040.LEVANTADO); }  // LEVANTADO = 1
  else if (Comparar(RecivoBuffer, "ACOSTADO"))    { ExtraerINT(RecivoBuffer, DesdeServo2040.ACOSTADO); }  // ACOSTADO = 1
  else if (Comparar(RecivoBuffer, "CALIBRANDO"))  { ExtraerINT(RecivoBuffer, DesdeServo2040.CALIBRANDO); }  // LEVANTADO = 1
  else if (Comparar(RecivoBuffer, "ALTURA"))      { ExtraerINT(RecivoBuffer, DesdeServo2040.ALTURA); }
  else if (Comparar(RecivoBuffer, "YAW"))         { ExtraerINT(RecivoBuffer, DesdeServo2040.Yaw); }
  else if (Comparar(RecivoBuffer, "PITCH"))       { ExtraerINT(RecivoBuffer, DesdeServo2040.Pitch); }
  else if (Comparar(RecivoBuffer, "ROLL"))        { ExtraerINT(RecivoBuffer, DesdeServo2040.Roll); }

  }
}

// =================== GESTION PANTALLA ===================
void pantalla() {
  float vbus = ina226.getBusVoltage_V();
  float vshunt = ina226.getShuntVoltage_V();
  float current = ina226.getCurrent_A();
  float power = vbus * current * 1.0;
  static bool BotonPulsadoDurante1S = false;
  static bool PrimerCiclo = true;

{
  bool boton = (digitalRead(BOTON_PIN) == LOW);
  static bool Pulsado;
  static long Antes;

  if (boton || Pulsado) { 
      if (!Pulsado) {
      Antes = millis() + 1000;
      Pulsado = true;
    } else {
      if (Antes < millis()) {BotonPulsadoDurante1S = true; Pulsado = false;} 
    }
  } else BotonPulsadoDurante1S = false;

}

  // ============================ MACROS =================================
  #define Aumentar_cada_(hacer,tiempo) {                                                    \
    static unsigned long ANTES; if(ANTES + tiempo < millis()) {hacer++; ANTES = millis();}}

  #define Activar_DespuesDe_(hacer,tiempo) {                                                \
    static unsigned long ANTES; if(ANTES + tiempo < millis()) {hacer = true;}}
  
  #define Desactivar_DespuesDe_(hacer,tiempo) {                                             \
    static unsigned long ANTES; if(ANTES + tiempo < millis()) {hacer = false;}}
  
  #define IndicadorBateria(x, y) {                                                          \
    u8g2.setFont(u8g2_font_6x10_tf);                                                        \
    float voltajeTotal = vbus + vshunt;                                                     \
    int porcentaje = map(voltajeTotal * 100, VOLTAJE_MIN *100, VOLTAJE_MAX *100, 0, 100);   \
    porcentaje = constrain(porcentaje, 0, 99);                                              \
    int ancho = 24, alto = 8;                                                               \
    u8g2.drawFrame(x, y, ancho, alto);                                                      \
    u8g2.drawBox(x + ancho, y + 2, 2, 5);                                                   \
    int nivel = map(porcentaje, 0, 100, 0, ancho - 3);                                      \
    u8g2.drawBox(x + 2, y + 2, nivel, alto - 3);                                            \
    u8g2.setCursor(x - 20, y + 7);                                                          \
    u8g2.print(porcentaje); u8g2.print("%");                                                \
  }                                                                                       

  #define MostrarV(x,y) {                                                                          \
    u8g2.setFont(u8g2_font_6x10_tf); u8g2.setCursor(x, y+7); u8g2.print(vbus, 1); u8g2.print("V"); \
  }

  #define MostrarA(x,y) {                                                                             \
    u8g2.setFont(u8g2_font_6x10_tf); u8g2.setCursor(x, y+7); u8g2.print(current, 1); u8g2.print("A"); \
  }

  #define Escribir(X_Cruda, Y_Cruda, Tamano, Color, texto) {                              \    
  static int offset = 0;                                                                  \
  static int X , Y ;                                                                      \                                         
    if (Tamano == 1) {u8g2.setFont(u8g2_font_6x10_tf); offset = 7;}                       \
    if (Tamano == 2) {u8g2.setFont(u8g2_font_8x13_tf); offset = 9;}                       \
    if (Tamano == 3) {u8g2.setFont(u8g2_font_10x20_tf); offset = 13;}                     \
    if (Color == 1)  {u8g2.setDrawColor(0);}                                              \
    if (Color == 2) {u8g2.setDrawColor(1);}                                               \
    u8g2.setCursor(X_Cruda, Y_Cruda+offset); u8g2.print(texto); u8g2.setDrawColor(1);     \
  }

  #define Dibujar_Circulo_Mpu(centroX,centroY,diametro) {                                                           \
    mpu.getAcceleration(&ax, &ay, &az);                                                                    \
    int offsetX = map(ay, -17000, 17000, 30, -30);                                                         \
    int offsetY = map(ax, -17000, 17000, 20, -20);                                                         \
    u8g2.drawCircle(centroX, centroY, diametro);                                                           \
    u8g2.drawDisc(centroX + offsetX, centroY + offsetY, 2);                                                \
  }

  #define Dibujar_Circulo_Joistick_Derecho(centroX,centroY,Radio) {                                             \
    int Desplazamiento = Radio*55/100, RadioCirInt = Radio*110/100;                                              \
    int JDY = PS5.RStickY(), JDX = PS5.RStickX();                                                               \
    int offsetJDY = map(JDY, -127, 128, Desplazamiento, -Desplazamiento);                                       \
    int offsetJDX = map(JDX, -127, 128, -Desplazamiento, Desplazamiento);                                       \
    int offsetY2 = map(offsetJDY, -Desplazamiento, Desplazamiento, -2, 2);                                       \
    int offsetX2 = map(offsetJDX, -Desplazamiento, Desplazamiento, -2, 2);                                       \
    int offsetY3 = map(offsetJDY, -Desplazamiento, Desplazamiento, -1, 1);                                       \
    int offsetX3 = map(offsetJDX, -Desplazamiento, Desplazamiento, -1, 1);                                       \
    u8g2.drawDisc(centroX, centroY, Radio);    /*DIBUJAR CIRCULO EXTERIOR BLANCO*/                              \ 
    u8g2.setDrawColor(0);                                                                                       \
    u8g2.drawDisc(centroX, centroY, Radio-2);  /*DIBUJAR CIRCULO EXTERIOR NEGRO MAS PEQUEÑO*/                   \   
    u8g2.setDrawColor(1);                                                                                       \ 
    u8g2.drawDisc(centroX + offsetJDX, centroY + offsetJDY, RadioCirInt); /*DIBUJAR CIRCULO INTERIOR BLANCO*/   \
    u8g2.setDrawColor(0);                                                                                       \
    u8g2.drawDisc(centroX + offsetJDX + offsetX2, centroY + offsetJDY + offsetY2, RadioCirInt-3);                                     \   
    u8g2.setDrawColor(1);                                                                                       \  
    u8g2.drawDisc(centroX + offsetJDX + offsetX3, centroY + offsetJDY + offsetY3, RadioCirInt-5);                                     \        
    Escribir(centroX + offsetJDX + offsetX3-5,centroY + offsetJDY + offsetY3-3,Small,Negro,"R3");                                                                       \ 
  }

  #define Dibujar_Circulo_Joistick_Izquierdo(centroX,centroY,Radio) {                                             \
    int Desplazamiento = Radio*55/100, RadioCirInt = Radio*110/100;                                              \
    int JIY = PS5.LStickY(), JIX = PS5.LStickX();                                                               \
    int offsetJIY = map(JIY, -127, 128, Desplazamiento, -Desplazamiento);                                       \
    int offsetJIX = map(JIX, -127, 128, -Desplazamiento, Desplazamiento);                                       \
    int offsetY2 = map(offsetJIY, -Desplazamiento, Desplazamiento, -2, 2);                                       \
    int offsetX2 = map(offsetJIX, -Desplazamiento, Desplazamiento, -2, 2);                                       \
    int offsetY3 = map(offsetJIY, -Desplazamiento, Desplazamiento, -1, 1);                                       \
    int offsetX3 = map(offsetJIX, -Desplazamiento, Desplazamiento, -1, 1);                                       \
    u8g2.drawDisc(centroX, centroY, Radio);    /*DIBUJAR CIRCULO EXTERIOR BLANCO*/                              \ 
    u8g2.setDrawColor(0);                                                                                       \
    u8g2.drawDisc(centroX, centroY, Radio-2);  /*DIBUJAR CIRCULO EXTERIOR NEGRO MAS PEQUEÑO*/                   \   
    u8g2.setDrawColor(1);                                                                                       \ 
    u8g2.drawDisc(centroX + offsetJIX, centroY + offsetJIY, RadioCirInt); /*DIBUJAR CIRCULO INTERIOR BLANCO*/   \
    u8g2.setDrawColor(0);                                                                                       \
    u8g2.drawDisc(centroX + offsetJIX + offsetX2, centroY + offsetJIY + offsetY2, RadioCirInt-3);                                     \   
    u8g2.setDrawColor(1);                                                                                       \  
    u8g2.drawDisc(centroX + offsetJIX + offsetX3, centroY + offsetJIY + offsetY3, RadioCirInt-5);                                     \        
    Escribir(centroX + offsetJIX + offsetX3-5,centroY + offsetJIY + offsetY3-3,Small,Negro,"L3");                                                                       \ 
  }

  #define DibujarPlantillaCompleta() {                                                                                \
    u8g2.setDrawColor(0); u8g2.drawBox(0,0,128,9); u8g2.setDrawColor(1);                                              \
    MostrarV(0,0); MostrarA(41,0); IndicadorBateria(101,0); u8g2.drawLine(0,9,128,9); u8g2.drawLine(0,55,128,55);     \
  }

  auto DibujarGrafico = [&](int x, int y, int ancho, int alto, int valor, int valorMin, int valorMax, int Direccion) {                                         
    int Porcentaje = constrain(map(valor,valorMin,valorMax,0,100),0,100);                           
    int offsetBordeAncho = map(ancho,0,100,0,0) + 2;  int An = offsetBordeAncho;                                     
    int offsetBordeAlto  = map(alto,0,100,0,0) + 2;   int Al = offsetBordeAlto;                                          
    u8g2.setDrawColor(1);                                                                                            
    u8g2.drawFrame(x,y,ancho,alto); /*RECTANGULO EXTERIOR*/                                                          
    if(Direccion == 0) {int PorcentajeGrafico = map(Porcentaje,0,100,0,ancho-An);                                    
                        u8g2.drawBox(x+An,y+Al,PorcentajeGrafico,alto-Al);}                                          
    if(Direccion == 1) {int PorcentajeGrafico = map(Porcentaje,0,100,0,ancho-An);                                    
                        u8g2.drawBox(x+ancho-PorcentajeGrafico-An,y+Al,PorcentajeGrafico,alto-Al);}                  
    if(Direccion == 2) {int PorcentajeGrafico = map(Porcentaje,0,100,0,alto-Al);                                     
                    u8g2.drawBox(x+An,y+Al,ancho-An,PorcentajeGrafico);}                                              
    if(Direccion == 3) {int PorcentajeGrafico = map(Porcentaje,0,100,0,alto-Al);                                     
                u8g2.drawBox(x+An,y+alto-PorcentajeGrafico-An,ancho-An,PorcentajeGrafico);}                                                                              
  };

  auto DibujarFlecha = [&](int x, int y,int ancho, int alto, int Direccion, int Color) {
    int X_Inicio,Y_Inicio,X_Punta,Y_Punta,X_Interior,Y_Interior,X_Final,Y_Final;
    if (Direccion==0){X_Inicio = x; Y_Inicio = y; X_Punta=x+ancho; Y_Punta=y+alto/2; X_Final=x; Y_Final=y+alto; X_Interior=x+ancho/4; Y_Interior=y+alto/2;}
    if (Direccion==1){X_Inicio = x+ancho; Y_Inicio = y; X_Punta=x; Y_Punta=y+alto/2; X_Final=x+ancho; Y_Final=y+alto; X_Interior=x+ancho-ancho/4; Y_Interior=y+alto/2;}
    if (Direccion==2){X_Inicio = x; Y_Inicio = y; X_Punta=x+ancho/2; Y_Punta=y+alto; X_Final=x+ancho; Y_Final=y; X_Interior=x+ancho/2; Y_Interior=y+alto/4;}
    if (Direccion==3){X_Inicio = x; Y_Inicio = y+alto; X_Punta=x+ancho/2; Y_Punta=y; X_Final=x+ancho; Y_Final=y+alto; X_Interior=x+ancho/2; Y_Interior=y+alto-alto/4;}

    if (Color==1) {u8g2.setDrawColor(0);}else{u8g2.setDrawColor(1);}
    u8g2.drawLine(X_Inicio,Y_Inicio,X_Punta,Y_Punta); u8g2.drawLine(X_Punta,Y_Punta,X_Final,Y_Final);
    u8g2.drawLine(X_Inicio,Y_Inicio,X_Interior,Y_Interior); u8g2.drawLine(X_Final,Y_Final,X_Interior,Y_Interior);
    u8g2.setDrawColor(1);
  };

  //TAMAÑOS
    #define Small 1
    #define Medium 2
    #define Big 3
  //COLORES
    #define Negro 1
    #define Blanco 2
  //DIRECCIONES GRAFICOS
  #define Izquierda_A_Derecha 0
  #define Derecha_A_Izquierda 1
  #define Arriba_A_Abajo 2
  #define Abajo_A_Arriba 3  

// ========================================================================= INICIO PROGRAMA PANTALLA ============================================================   
    
    u8g2.clearBuffer();

  if (PrimerCiclo) 
  {
    static long Antes = millis();
    if (millis() <= Antes + 1500) {
    Escribir(4, 16, Big, Blanco, "ARES HEXAPOD");
    Escribir(0, 35, Small, Blanco, "Iniciando...");
    DibujarGrafico(0,55,128,9, millis(), Antes, 1500, Izquierda_A_Derecha);
    DibujarPlantillaCompleta();   
    } else {PrimerCiclo = false;}
  }

  if (!PrimerCiclo) 
  {
    //GESTOR DE MENU
  enum HMI {
    P_MENU_PRINCIPAL,
    P_MENU_PARAMETROS,
    P_MENU_CALIBRACION,
    P_MENU_DEBUG,
    P_LEVANTADO,
    P_EMPAREJAR,
    P_NADA
  };
  static HMI PANTALLA = P_MENU_PRINCIPAL;

  enum SOL {
    S_MENU_EN_MARCHA,
    S_MENU_PRINCIPAL,
    S_MENU_PARAMETROS,
    S_MENU_CALIBRACION,
    S_MENU_DEBUG,
    S_LEVANTADO,
    S_EMPAREJAR,
    S_NADA
  };

  static SOL SOLICITUD = S_NADA; 
  
  bool PantallasMenuAcostado =
      (PANTALLA == P_MENU_PRINCIPAL ||
      PANTALLA == P_MENU_PARAMETROS ||
      PANTALLA == P_MENU_CALIBRACION ||
      PANTALLA == P_MENU_DEBUG ||
      PANTALLA == P_EMPAREJAR);
      
  bool PantallasLevantado =
      (PANTALLA == P_LEVANTADO ||
      PANTALLA == P_EMPAREJAR);

  
{
bool OK = 0;
int CountOK = 0;
  while (!OK && CountOK <= 10)
  {
    if (DesdeServo2040.ACOSTADO == true && !PantallasMenuAcostado) {
      SOLICITUD=S_MENU_PRINCIPAL;
    }
      if (DesdeServo2040.ACOSTADO == false && !PantallasLevantado) {
      SOLICITUD=S_LEVANTADO;
    }

    if (SOLICITUD==S_MENU_PRINCIPAL) {
      if (PANTALLA==P_MENU_PRINCIPAL) {SOLICITUD=S_NADA; OK=1;} else {PANTALLA=P_MENU_PRINCIPAL; OK=1;}
    }

    if (SOLICITUD==S_LEVANTADO) {
      if (PANTALLA==P_LEVANTADO) {SOLICITUD=S_NADA; OK=1;} else {PANTALLA=P_LEVANTADO; OK=1;}
    }

    if (BotonPulsadoDurante1S) {SOLICITUD=S_EMPAREJAR;}

    if (SOLICITUD==S_EMPAREJAR) { if (PANTALLA==P_EMPAREJAR) {SOLICITUD=S_NADA;} else {PANTALLA=P_EMPAREJAR;} }

  CountOK = CountOK + 1;
  }
}

bool Flanco_Arriba = FlancoSubida(PS5.Up()); {static long Antes; if (Flanco_Arriba) {Antes = millis();} if (millis() > Antes+500 && PS5.Up()) {Flanco_Arriba = true;} }
bool Flanco_Abajo = FlancoSubida(PS5.Down()); {static long Antes; if (Flanco_Abajo) {Antes = millis();} if (millis() > Antes+500 && PS5.Down()) {Flanco_Abajo = true;} }
bool Flanco_Derecha = FlancoSubida(PS5.Right()); {static long Antes; if (Flanco_Derecha) {Antes = millis();} if (millis() > Antes+500 && PS5.Right()) {Flanco_Derecha = true;} }
bool Flanco_Izquierda = FlancoSubida(PS5.Left()); {static long Antes; if (Flanco_Izquierda) {Antes = millis();} if (millis() > Antes+500 && PS5.Left()) {Flanco_Izquierda = true;} }

if (PANTALLA==P_MENU_CALIBRACION) {CALIBRANDO = true;} else {CALIBRANDO = false;}

switch(PANTALLA) {
  case P_LEVANTADO:{
    if (PS5.Left()){PANTALLA=P_MENU_PRINCIPAL;}

    DibujarGrafico(0,12,20,40,DesdeServo2040.ALTURA,10,120,Abajo_A_Arriba);       Dibujar_Circulo_Mpu(60,17,10); Escribir(52,37,Small,Blanco,"MPU");
    DibujarGrafico(103,19,25,7,DesdeServo2040.Yaw,-20,20,Derecha_A_Izquierda);
    DibujarGrafico(103,29,25,7,DesdeServo2040.Pitch,-20,20,Derecha_A_Izquierda);
    DibujarGrafico(103,39,25,7,DesdeServo2040.Roll,-20,20,Derecha_A_Izquierda);

    Escribir(93,19,Small,Blanco,"Y");
    Escribir(93,29,Small,Blanco,"P");
    Escribir(93,39,Small,Blanco,"R");

    DibujarPlantillaCompleta();
    Escribir(0,57,Small,Blanco,"ALT:"); Escribir(26,57,Small,Blanco,DesdeServo2040.ALTURA);
    Escribir(98,57,Small,Blanco,"INCL:");
    
  }break;  

  case P_MENU_PRINCIPAL:{

    static int i = 0;

    char MENU[10][64];
      strcpy(MENU[0],"Parametros");     if (Flanco_Derecha && i==0 ){PANTALLA=P_MENU_PARAMETROS;}
      strcpy(MENU[1],"Calibracion");    if (Flanco_Derecha && i==1 ){PANTALLA=P_MENU_CALIBRACION;}
      strcpy(MENU[2],"Debug");          if (Flanco_Derecha && i==2 ){PANTALLA=P_MENU_DEBUG;}
      strcpy(MENU[3],"MENU 1");
      strcpy(MENU[4],"MENU 2");
      strcpy(MENU[5],"MENU 3");
      strcpy(MENU[6],"MENU 4");
      strcpy(MENU[7],"MENU 5");
      strcpy(MENU[8],"MENU 6");
      strcpy(MENU[9],"MENU 7");                 
    
      bool PermisoEntrar = i == 0 || i == 1 || i == 2;

    if (Flanco_Abajo && i < 9){i++;}
    if (Flanco_Arriba && i > 0){i--;}


    if (i > 0) {DibujarFlecha(0,12,5,5,Abajo_A_Arriba,Blanco); Escribir(9,11,Medium,Blanco,MENU[i-1]);}
    DibujarFlecha(2,25,11,11,Izquierda_A_Derecha,Blanco); Escribir(18,25,Big,Negro,MENU[i]); u8g2.drawBox(16,22,u8g2.getStrWidth(MENU[i])+4,20); Escribir(18,25,Big,Negro,MENU[i]);
    if (i < 9) {DibujarFlecha(0,45,5,5,Arriba_A_Abajo,Blanco); Escribir(9,44,Medium,Blanco,MENU[i+1]);}
        
    DibujarPlantillaCompleta();
    if (PermisoEntrar) {DibujarFlecha(119,57,7,7,Izquierda_A_Derecha,Blanco); Escribir(80,57,Small,Blanco,"Entrar");}
    
  }break;

  case P_MENU_PARAMETROS: {
    
  enum P_M_P {
  SeleccionParametro,
  AjusteParametro
  };

  static P_M_P P_Menu_Parametros = SeleccionParametro;

  static int ParametroSeleccionado;

  #define MenusTotales 8

  char MENU[2][MenusTotales+1][32];

{ // MENUS 
  strcpy(MENU[0][0],"0-Zona Muerta Joistick");     strcpy(MENU[1][0],"ticks");  
  strcpy(MENU[0][1],"1-Velocidad Max");            strcpy(MENU[1][1],"mm*s");  
  strcpy(MENU[0][2],"2-Distancia Paso 1");         strcpy(MENU[1][2],"mm");  
  strcpy(MENU[0][3],"3-Distancia Paso 2");         strcpy(MENU[1][3],"mm");  
  strcpy(MENU[0][4],"4-Distancia Paso 3");         strcpy(MENU[1][4],"mm");  
  strcpy(MENU[0][5],"5-Velocidad Bajada Patas");   strcpy(MENU[1][5],"mm*s");  
  strcpy(MENU[0][6],"6-Velocidad Subida Patas");   strcpy(MENU[1][6],"mm*s");  
  strcpy(MENU[0][7],"7-Multiplicador Parabola");   strcpy(MENU[1][7],"x");
  strcpy(MENU[0][8],"8-Multiplicador Ayuda Patas");strcpy(MENU[1][8],"x");
 
}
  



  switch (P_Menu_Parametros) {

    case SeleccionParametro: {
      static int i = 0;
      static int iPrev = 0;
      static bool MuyLargo = false;
      static unsigned long Antes;
      static unsigned long OFFSET = 700;
      static int POS = 0;
      
      bool FlancoMuyLargo = FlancoSubida(MuyLargo);
      bool FlancoNoMuyLargo = FlancoBajada(MuyLargo);

      if ( Flanco_Izquierda )  {PANTALLA=P_MENU_PRINCIPAL;} 
      
      ParametroSeleccionado = i;
      if ( Flanco_Derecha ) {P_Menu_Parametros=AjusteParametro;}
      
      if (Flanco_Abajo && i < MenusTotales){i++;}
      if (Flanco_Arriba && i > 0){i--;}

      if (i != iPrev) {iPrev = i; OFFSET = 700; POS = 0; FlancoMuyLargo = 0; FlancoNoMuyLargo = 0; MuyLargo = 0; Antes = millis();}
      if (FlancoMuyLargo) {Antes = millis();}
      if (MuyLargo) {if (millis() > Antes + OFFSET) {OFFSET = 200; POS++; Antes = millis();}}
      if (FlancoNoMuyLargo) {Antes = millis(); OFFSET = 1000;}
      if (!MuyLargo) {if (millis() > Antes + OFFSET) {OFFSET = 700; POS = 0; Antes = millis();} }  

      strcpy(MENU[0][i],MENU[0][i]+POS);
      
      if (i > 0) {DibujarFlecha(0,12,5,5,Abajo_A_Arriba,Blanco); Escribir(9,11,Medium,Blanco,MENU[0][i-1]);}
      DibujarFlecha(2,25,11,11,Izquierda_A_Derecha,Blanco); Escribir(18,25,Big,Negro,MENU[0][i]); u8g2.drawBox(16,22, u8g2.getStrWidth(MENU[0][i])+4,20); Escribir(18,25,Big,Negro,MENU[0][i]);
      MuyLargo = (u8g2.getStrWidth(MENU[0][i]) > 128 - 18);
      if (i < MenusTotales) {DibujarFlecha(0,45,5,5,Arriba_A_Abajo,Blanco); Escribir(9,44,Medium,Blanco,MENU[0][i+1]);}

      DibujarFlecha(119,57,7,7,Izquierda_A_Derecha,Blanco); Escribir(80,57,Small,Blanco,"Entrar");      
    } break;


    case AjusteParametro: {

      if (Flanco_Izquierda) {P_Menu_Parametros=SeleccionParametro;}

      bool Variable_con_comas = (ParametroSeleccionado == 7 || ParametroSeleccionado == 8);
      float SUMA = Variable_con_comas ? 0.01f : 1.0f;


      if (Flanco_Arriba)  {Variable[ParametroSeleccionado+18] += SUMA;}
      if (Flanco_Abajo)   {Variable[ParametroSeleccionado+18] -= SUMA;}


      static char Parametro[32] = {};
      sprintf(Parametro,"%.2f %s",Variable[ParametroSeleccionado+18],MENU[1][ParametroSeleccionado]);


        Escribir(0,11,Small,Blanco,MENU[0][ParametroSeleccionado]);

        DibujarFlecha(5,26,9,9,Abajo_A_Arriba,Blanco);
        DibujarFlecha(5,39,9,9,Arriba_A_Abajo,Blanco);
        Escribir(18,31,Big,Blanco,Parametro); 
        
    } break;
  }


    DibujarPlantillaCompleta();
    DibujarFlecha(0,57,7,7,Derecha_A_Izquierda,Blanco); Escribir(13,57,Small,Blanco,"Menu"); 
  }break;

  case P_MENU_CALIBRACION: {
    
  static int PataSeleccionada = 0; static char CHAR_PataSeleccionada[16]; 
  static int ServoSeleccionado = 0; static char CHAR_ServoSeleccionado[16];

    enum P_Cal {
    SeleccionPata,
    SeleccionServo,
    AjusteServo
  };
  static P_Cal Pantalla_Calibraje=SeleccionPata;



  switch (Pantalla_Calibraje) {

    case SeleccionPata: {
      if ( Flanco_Izquierda ) {PANTALLA=P_MENU_PRINCIPAL;}
    static int i = 0;
    
    char MENU[6][64];
      strcpy(MENU[0],"Del.Izqu");   
      strcpy(MENU[1],"Del.Der");   
      strcpy(MENU[2],"Med.Izqu");          
      strcpy(MENU[3],"Med.Der");
      strcpy(MENU[4],"Atr.Izqu");
      strcpy(MENU[5],"Atr.Der");
  
      PataSeleccionada = i;
      strcpy(CHAR_PataSeleccionada,MENU[i]);
      if ( Flanco_Derecha ) {Pantalla_Calibraje=SeleccionServo;} 

      if (Flanco_Abajo && i < 5) {i++;}
      if (Flanco_Arriba && i > 0){i--;}
      
      if (i > 0) {DibujarFlecha(0,12,5,5,Abajo_A_Arriba,Blanco); Escribir(9,11,Medium,Blanco,MENU[i-1]);}
      DibujarFlecha(2,25,11,11,Izquierda_A_Derecha,Blanco); Escribir(18,25,Big,Negro,MENU[i]); u8g2.drawBox(16,22,u8g2.getStrWidth(MENU[i])+4,20); Escribir(18,25,Big,Negro,MENU[i]);  
      if (i < 5) {DibujarFlecha(0,45,5,5,Arriba_A_Abajo,Blanco); Escribir(9,44,Medium,Blanco,MENU[i+1]);}
      
      DibujarFlecha(119,57,7,7,Izquierda_A_Derecha,Blanco); Escribir(80,57,Small,Blanco,"Entrar"); 
    } break;

    case SeleccionServo: {
      static int i = 0;
      if ( Flanco_Izquierda )  {Pantalla_Calibraje=SeleccionPata;} 
      
      char MENU[3][64];
        strcpy(MENU[0],"Coxa");     
        strcpy(MENU[1],"Femur");   
        strcpy(MENU[2],"Tibia");

        ServoSeleccionado = i;
        strcpy(CHAR_ServoSeleccionado,MENU[i]);
        if ( Flanco_Derecha ) {Pantalla_Calibraje=AjusteServo;}
        
        if (Flanco_Abajo && i < 2){i++;}
        if (Flanco_Arriba && i > 0){i--;}
        
        if (i > 0) {DibujarFlecha(0,12,5,5,Abajo_A_Arriba,Blanco); Escribir(9,11,Medium,Blanco,MENU[i-1]);}
        DibujarFlecha(2,25,11,11,Izquierda_A_Derecha,Blanco); Escribir(18,25,Big,Negro,MENU[i]); u8g2.drawBox(16,22, u8g2.getStrWidth(MENU[i])+4,20); Escribir(18,25,Big,Negro,MENU[i]);  
        if (i < 2) {DibujarFlecha(0,45,5,5,Arriba_A_Abajo,Blanco); Escribir(9,44,Medium,Blanco,MENU[i+1]);}

      DibujarFlecha(119,57,7,7,Izquierda_A_Derecha,Blanco); Escribir(80,57,Small,Blanco,"Entrar");      
    } break;

    case AjusteServo: {

      if (Flanco_Izquierda) {Pantalla_Calibraje=SeleccionServo;}
      char TEXTO_SERVO_SELECCIONADO[32]; 
      snprintf(TEXTO_SERVO_SELECCIONADO, sizeof(TEXTO_SERVO_SELECCIONADO),"Servo %s %s ", CHAR_ServoSeleccionado, CHAR_PataSeleccionada);

    int ServoActual = PataSeleccionada + ServoSeleccionado * 6;

    if (Flanco_Arriba && Variable[ServoActual] < 90 && DesdeServo2040.CALIBRANDO)  {Variable[ServoActual]++;}
    if (Flanco_Abajo && Variable[ServoActual] > -90 && DesdeServo2040.CALIBRANDO)   {Variable[ServoActual]--;}
    
  {
    static bool MostrarMensaje = false;
    if ( ((Flanco_Arriba || Flanco_Abajo) && !DesdeServo2040.CALIBRANDO) || MostrarMensaje) 
    {
      Escribir(0,16,Small,Blanco,"Presiona Boton")
      Escribir(0,27,Small,Blanco,"PS Para Entrar En")
      Escribir(0,38,Small,Blanco,"Modo Calibracion")      
      static long Antes;
      if(!MostrarMensaje) {
      MostrarMensaje = true;
      Antes = millis() + 1500;
      } else {
        if (Antes < millis()) {MostrarMensaje = false;}
      }

    } else {
      Escribir(0,11,Small,Blanco,TEXTO_SERVO_SELECCIONADO);

      DibujarFlecha(15,26,9,9,Abajo_A_Arriba,Blanco);
      DibujarFlecha(15,39,9,9,Arriba_A_Abajo,Blanco);
      Escribir(28,31,Big,Blanco,Variable[ServoActual]);
    }
  }
        
    } break;
  }







    DibujarPlantillaCompleta();
    DibujarFlecha(0,57,7,7,Derecha_A_Izquierda,Blanco); Escribir(13,57,Small,Blanco,"Menu");    

  }break;
  
  case P_MENU_DEBUG: {
    if (PS5.Left()){PANTALLA=P_MENU_PRINCIPAL;}
    DibujarPlantillaCompleta();    
    DibujarFlecha(0,57,7,7,Derecha_A_Izquierda,Blanco); Escribir(13,57,Small,Blanco,"Menu");   
  }break;  

  case P_EMPAREJAR: {
  static bool started = false;
  static bool SolicitudSalir = false;
  static uint32_t t0 = 0;

  u8g2.setDrawColor(1);
  Escribir(0,16,Small,Blanco,"EMPAREJANDO MANDO...");
  DibujarGrafico(0,55,128,9, millis() - t0, 0, 10000, Izquierda_A_Derecha);

  if (!started) {
    PS5.begin("90:b6:85:a2:c1:12");   // ← solo una vez
    started = true;
    t0 = millis();
  }

  if (conectado) {
    u8g2.clearBuffer();      
    Escribir(0,16,Small,Blanco,"MANDO CONECTADO");
    SolicitudSalir = true;
  } else {
    if (millis() - t0 >= 10000) {
      u8g2.clearBuffer();
      Escribir(0,16,Small,Blanco,"EMPAREJAMIENTO FALLIDO");
      SolicitudSalir = true;
    }
  }

  if (SolicitudSalir) {    
  static bool Ahora = false;
  Activar_DespuesDe_(Ahora,1000);
  if (Ahora) {
    started = false;
    PANTALLA = P_MENU_PRINCIPAL;
    Ahora = false;
  }
  }

  DibujarPlantillaCompleta();

} break;

}
}

 u8g2.sendBuffer();
// ========================================================================= FINAL PROGRAMA PANTALLA ============================================================   
} 

// =================== Gestion_EEPROM ===================
void Gestion_EEPROM (bool PrimerCiclo) {

  if (PrimerCiclo)
  {
    EEPROM.begin(EEPROM_SIZE);

    //RECOGER VARIABLES DE LA EEMPROM AL INICIAR
    for (int i = 0; i < MaxVariables; i++) { EEPROM.get(i * 4, Variable[i]); EEPROM.get(i * 4, VariablePrevEEPROAM[i]);}
  } 

  if (! PrimerCiclo)
  {  
    // GUARDAR EN LA EEPROM SI ALGUNA VARIABLE CAMBIA
    for (int i = 0; i < MaxVariables; i++) { if (Variable[i] != VariablePrevEEPROAM[i]) { EEPROM.put(i*4,Variable[i]); EEPROM.commit(); VariablePrevEEPROAM[i] = Variable[i]; } }
  }

}



// =================== SET UP ===================
void setup() {
  iniciarSistema();
  escanearI2C();
  Gestion_EEPROM(1);
}

// =================== LOOP ===================
void loop() {
  if (TimerEvery(0, 100)) {
  comprobarVoltsyAmps();
  }
  if (TimerEvery(1, 80)) {
  pantalla();
  }
  if (TimerEvery(2, 50)) {
  Gestion_EEPROM(0);
  }
  if (TimerEvery(3, 5)) {
  enviarDatosUART();
  recibirYprocesarDesdeUART();
  }

}



