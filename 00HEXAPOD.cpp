#include "pico/stdlib.h"
#include <string>
#include "servo2040.hpp"
#include "servo_cluster.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include <cstring> // para memset

using namespace servo;

#ifndef constrain
template<typename T>
static inline T constrain(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}
#endif


//================RECIVO DE DATOS UART============
//PS5
int JdxVal = 0;
int JdyVal = 0;
int JixVal = 0;
int JiyVal = 0;
int L2Val = 0;
int R2Val = 0;
float pitchValMPU = 0;
float rollValMPU = 0;
bool botonPS = false;
bool mandoConectado = false;
bool RIGHT = false, LEFT = false, UP = false, DOWN = false, SQUARE = false, CROSS = false, CIRCLE = false, TRIANGLE = false, L1 = false, R1 = false, L2 = false, R2 = false, SHARE = false, OPTIONS = false, L3 = false, R3 = false, PSButton = false, Touchpad = false;
bool CALIBRANDO = false;
bool CALIBRANDOaceptado = false;
//Calibracion servos
static float CalServo[18] = {0.0f};
using namespace servo;
const uint NUM_SERVOS = 18;
const uint START_PIN = servo2040::SERVO_1;
ServoCluster servos(pio0, 0, START_PIN, NUM_SERVOS);
bool servosActivados = false;
bool servosIniciados = false;

//Parametros guardados en la EPROM de la esp32
struct ParaRecivir {
  int ZonaMuertaJoistick ;
  float Velocidad_Max_mm_s ; 
  float Paso1_mm ;
  float Paso2_mm ;
  float Paso3_mm ;
  float Velocidad_Bajada_Patas_mm_s ;
  float Velocidad_Subida_Patas_mm_s ;
  float MultiplicadorParabola ;
  float MultiplicadorAyudaPatas ;
};
ParaRecivir DesdeESP32;



//================ENVIO DE DATOS UART============
struct ParaEnvios {
  bool LEVANTADO, LEVANTADOprev;
  bool ACOSTADO, ACOSTADOprev;
  bool CALIBRANDO, CALIBRANDOprev;
  int Altura, AlturaPrev;
  int Yaw, YawPrev;
  int Pitch, PitchPrev;
  int Roll, RollPrev;
};

ParaEnvios HaciaESP32;


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

static float wrap180f(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static float stepAxis(float act, float obj, float vmax_deg_s, float dt) {
  float e = wrap180f(obj - act);
  float maxStep = vmax_deg_s * dt;   // grados que puedo avanzar en este ciclo
  if (maxStep <= 0.0f) return act;
  if (fabsf(e) <= maxStep) return obj;
  return act + (e > 0.0f ? maxStep : -maxStep);
}

// ===================== FUNCION MAPEO ======================
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ===================== TIMERS TON ======================
struct TimerON {
  unsigned long t = 0;
  bool a = false;
  bool f = false;

  void set(unsigned long d) {
    if (!a) { t = millis() + d; a = true; f = false; }
  }

  bool done() {
    if (a && !f && millis() >= t) f = true;
    return f;
  }

  void reset() { a = false; f = false; }
};
TimerON TON[50]; 

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
TimerOFF TOFF[50];  // usa toff[1], toff[2], ...

// ===================== MEDIR CORRIENTE DENTRO DE SERVO2040 =====================
void Servo2040_PrintPower_Simple() {
  // Objetos de medida (se construyen una sola vez)
  static Analog cur_adc(servo2040::SHARED_ADC,
                        servo2040::CURRENT_GAIN,
                        servo2040::SHUNT_RESISTOR,
                        servo2040::CURRENT_OFFSET);
  static Analog vin_adc(servo2040::SHARED_ADC);
  static AnalogMux mux(servo2040::ADC_ADDR_0,
                       servo2040::ADC_ADDR_1,
                       servo2040::ADC_ADDR_2,
                       PIN_UNUSED,
                       servo2040::SHARED_ADC);

  // Temporización para no spamear la consola
  static uint32_t last_ms = 0;
  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (now - last_ms < 250) return;
  last_ms = now;

  // Corriente total (canal de corriente del 4051)
  mux.select(servo2040::CURRENT_SENSE_ADDR);
  sleep_us(5);
  float I = cur_adc.read_current();     // A

  // Tensión de entrada (V_SENSE suele ser Y6 del 4051)
  mux.select(6);
  sleep_us(5);
  float v_sense = vin_adc.read_voltage();  // V en el ADC (tras divisor)

  // Factor del divisor para obtener VIN real (ajusta si fuera necesario)
  constexpr float VDIV = 3.30f;
  float VIN = v_sense * VDIV;

  printf("VIN=%.2f V  I=%.2f A\r\n", VIN, I);
}

// ==================== SETUP SERVOS =====================
void setupServos() {
  if (mandoConectado && !servosIniciados) {
    servos.init();
    servosIniciados = true;
    printf("✅ Servos inicializados\n");
  }
}

// ================== MOVIMIENTO DE PATAS =====================
void MOVIMIENTOPATAS () {
  static bool BOTONopciones = false, BOTONopcionesPREV = true;
  static float altura = 0.0f;
  static float VelocidadMovimiento[7] = {}; // Velocidad de movimiento en milisegundos
  static bool patasTerminadas[7] = {true,true,true,true,true,true,true};  // Índice 1 a 6
  static bool PARABOLICO[7] = {false};


  //GESTION ALTURA
  static float alturaBase = 55.0f;  // Valor inicial de altura
  const float alturaMin = 10.0f;
  const float alturaMax = 120.0f;
  const float pasoAltura = 4.0f;
  static bool UPprev = false;
  static bool DOWNprev = false;

  static float DistanciaP1P2[7] = {};
  (void) DistanciaP1P2;

  struct XYZfloat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float total() const {          // 👈 devuelve la suma de los 3 valores
    return x + y + z;
      }
  };

  struct Yaw_Pitch_Roll_FLOAT {
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
  };

  struct XYZYawPitchRoll_FLOAT {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
  };

  const XYZfloat baseReposo1[7] = {
  { 0.0f, 0.0f, 0.0f }, // NA 
  { 78.933f, 0.0f, 11.205f }, // pata 1
  { 78.933f, 0.0f, 11.205f }, // pata 2
  { 78.933f, 0.0f, 11.205f }, // pata 3
  { 78.933f, 0.0f, 11.205f }, // pata 4
  { 78.933f, 0.0f, 11.205f }, // pata 5
  { 78.933f, 0.0f, 11.205f }  // pata 6
};
  (void) baseReposo1;

  const XYZfloat baseReposo2[7] = {
  { 0.0f, 0.0f, 0.0f }, // NA 
  { 55.814f , 55.814f  , 11.205f }, // pata 1
  { 55.814f , 55.814f  , 11.205f }, // pata 2
  { 78.933f , 0.0f     , 11.205f }, // pata 3
  { 78.933f , 0.0f     , 11.205f }, // pata 4
  { 55.814f , -55.814f , 11.205f }, // pata 5
  { 55.814f , -55.814f , 11.205f }  // pata 6
};
  (void) baseReposo2;

const XYZfloat POS_CIRCULAR[7] = {
  { 0.0f, 0.0f, 0.0f }, // NA 
  { 40.5f , 103.5f  , 11.205f }, // pata 1
  { 40.5f , 103.5f  , 11.205f }, // pata 2
  { 103.5f , 0.0f     , 11.205f }, // pata 3
  { 103.5f , 0.0f     , 11.205f }, // pata 4
  { 40.5f , -103.5f  , 11.205f }, // pata 5
  { 40.5f , -103.5f  , 11.205f }  // pata 6
};

// const XYZfloat POS_CIRCULAR[7] = {
//   { 0.0f, 0.0f, 0.0f }, // NA 
//   { 45.0f , 115.00f  , 11.205f }, // pata 1
//   { 45.0f , 115.00f  , 11.205f }, // pata 2
//   { 115.0f , 0.0f     , 11.205f }, // pata 3
//   { 115.0f , 0.0f     , 11.205f }, // pata 4
//   { 45.0f , -115.00f  , 11.205f }, // pata 5
//   { 45.0f , -115.00f  , 11.205f }  // pata 6
// };



const XYZfloat POS_EXTENDIDA[7] = {
  { 0.0f, 0.0f, 0.0f }, // NA 
  { 75.814f , 75.814f  , 11.205f }, // pata 1
  { 75.814f , 75.814f  , 11.205f }, // pata 2
  { 98.933f , 0.0f     , 11.205f }, // pata 3
  { 98.933f , 0.0f     , 11.205f }, // pata 4
  { 95.814f , -75.814f , 11.205f }, // pata 5
  { 75.814f , -75.814f , 11.205f }  // pata 6
};

  static XYZfloat Punta_Pata_Pos_Act[7] = {
  { 0.0f, 0.0f, 0.0f }, // NA 
  { 78.933f, 0.0f, 11.205f }, // pata 1
  { 78.933f, 0.0f, 11.205f }, // pata 2
  { 78.933f, 0.0f, 11.205f }, // pata 3
  { 78.933f, 0.0f, 11.205f }, // pata 4
  { 78.933f, 0.0f, 11.205f }, // pata 5
  { 78.933f, 0.0f, 11.205f }  // pata 6
  };

  

  const XYZYawPitchRoll_FLOAT CONST_POSICNES_ORIGEN_PATA[7] = {
  { 0.0f      , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // NA 
  { 71.106f   ,  93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 1
  { -71.106f  ,  93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 2
  { 94.35f    , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 3
  { -94.35f   , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 4
  { 71.106f   , -93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 5
  { -71.106f  , -93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // CONSTANTE POSICION X Y Z Yaw Pitch Roll PATA 6
  };

  static XYZYawPitchRoll_FLOAT POS_ACT_POSICNES_ORIGEN_PATA[7] = {
  { 0.0f      , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // NA 
  { 71.106f   ,  93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 1
  { -71.106f  ,  93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 2
  { 94.35f    , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 3
  { -94.35f   , 0.0f      , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 4
  { 71.106f   , -93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 5
  { -71.106f  , -93.115f  , 0.0f  , 0.0f  , 0.0f  , 0.0f }, // POS_ACT POSICION X Y Z Yaw Pitch Roll PATA 6
  };
  (void) POS_ACT_POSICNES_ORIGEN_PATA;


  XYZfloat POS_ACTUAL_ROBOT; // Posicion actual del robot
  const XYZYawPitchRoll_FLOAT CONST_CentroRobot = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  static XYZYawPitchRoll_FLOAT POS_CALCULADO_CENTRO_ROBOT = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  static XYZfloat Punta_Pata_RespectoCentro_ACTUAL[7];
  static XYZfloat Punta_Pata_RespectoCentro_GUARDADO[7];
  static XYZfloat Punta_Pata_RespectoCentro_extendida[7];
  static XYZfloat Punta_Pata_RespectoCentro_circular[7];
  static XYZfloat Punta_Pata_RespectoCentro_REPOSO1[7];

  
// ============================================================= VARIABLES DETECCIÓN DE DEMANDA DE MOVIMIENTO (por pata) ========================================================
// --- DETECCIÓN DE DEMANDA DE MOVIMIENTO (por pata) ---
static bool demandaMovimiento[7] = {false};   // 1..6
static float deltaX[7] = {0.0f};  // diferencia pendiente en X
static float deltaY[7] = {0.0f};  // diferencia pendiente en Y
static float deltaZ[7] = {0.0f};  // diferencia pendiente en Z
(void) deltaX;
(void) deltaY;
(void) deltaZ;

static float objPrevX[7] = {0}, objPrevY[7] = {0}, objPrevZ[7] = {0};
static bool  PrimerCICLO    = false;      // para inicializar solo una vez
static bool  cambioObjetivo[7] = {0};  // true si el objetivo cambió este ciclo

static float p0x[7]={}, p0y[7]={}, p0z[7]={};
static float p1x[7]={}, p1y[7]={}, p1z[7]={};
static XYZfloat Punta_Pata_RespectoCentro_INTERP[7]; // destino instantáneo
static XYZfloat Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[7]; // destino instantáneo
// =============================================================================================================================================================================





// ==================================================================== VARIABLES DETECCIÓN CINEMATICA INVERSA =================================================================

  static float angCoxa[7] = {}, angFemur[7] = {0.0f}, angTibia[7] = {0.0f};
  static float angCoxaCalibrado[7] = {}, angFemurCalibrado[7] = {0.0f}, angTibiaCalibrado[7] = {0.0f};

    // CONFIGURACION CINEMATICA INVERSA
  const float Lcoxa = 34.6f;
  const float Lfemur = 76.009f;
  const float Ltibia = 110.304f;
  //ESTRUCTURA COMANDOS ROBOT 

  static XYZfloat CordsPataGLOBAL[7] = {};
  (void)CordsPataGLOBAL;

  
  struct ABC {
    float lonV_A_B = 0.0f;  // AB
    float lonV_B_C = 0.0f;  // BC
    float lonV_C_A = 0.0f;  // AC
    float ang_A = 0.0f;     // ∠A
    float ang_B = 0.0f;     // ∠B
    float ang_C = 0.0f;     // ∠C
  };

  struct XYZPV {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float parabolica = 0.0f;
    float velocidad = 0.0f;

  };


  struct XYZint {
    int x = 0.0f;
    int y = 0.0f;
    int z = 0.0f;
  };
// =============================================================================================================================================================================






{
  static bool PrimerCiclo = true; 
  if (PrimerCiclo) {
// CALCULO POSICION RELATIVA RESPECTO A CENTRO ROBOT DE CADA PUNTA DE CADA PATA
{Punta_Pata_RespectoCentro_ACTUAL[1].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[1].x + Punta_Pata_Pos_Act[1].x;
Punta_Pata_RespectoCentro_ACTUAL[1].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[1].y + Punta_Pata_Pos_Act[1].y;
Punta_Pata_RespectoCentro_ACTUAL[1].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[1].z + Punta_Pata_Pos_Act[1].z;

Punta_Pata_RespectoCentro_ACTUAL[2].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[2].x + -(Punta_Pata_Pos_Act[2].x);
Punta_Pata_RespectoCentro_ACTUAL[2].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[2].y + Punta_Pata_Pos_Act[2].y;
Punta_Pata_RespectoCentro_ACTUAL[2].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[2].z + Punta_Pata_Pos_Act[2].z;

Punta_Pata_RespectoCentro_ACTUAL[3].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[3].x + Punta_Pata_Pos_Act[3].x;
Punta_Pata_RespectoCentro_ACTUAL[3].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[3].y + Punta_Pata_Pos_Act[3].y;
Punta_Pata_RespectoCentro_ACTUAL[3].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[3].z + Punta_Pata_Pos_Act[3].z;

Punta_Pata_RespectoCentro_ACTUAL[4].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[4].x + -(Punta_Pata_Pos_Act[4].x);
Punta_Pata_RespectoCentro_ACTUAL[4].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[4].y + Punta_Pata_Pos_Act[4].y;
Punta_Pata_RespectoCentro_ACTUAL[4].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[4].z + Punta_Pata_Pos_Act[4].z;

Punta_Pata_RespectoCentro_ACTUAL[5].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[5].x + Punta_Pata_Pos_Act[5].x;
Punta_Pata_RespectoCentro_ACTUAL[5].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[5].y + Punta_Pata_Pos_Act[5].y;
Punta_Pata_RespectoCentro_ACTUAL[5].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[5].z + Punta_Pata_Pos_Act[5].z;

Punta_Pata_RespectoCentro_ACTUAL[6].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[6].x + -(Punta_Pata_Pos_Act[6].x);
Punta_Pata_RespectoCentro_ACTUAL[6].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[6].y + Punta_Pata_Pos_Act[6].y;
Punta_Pata_RespectoCentro_ACTUAL[6].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[6].z + Punta_Pata_Pos_Act[6].z;}
    PrimerCiclo = false;
  }
}

// CALCULO POSICION EXTENDIDA PREDEFINIDA RESPECTO A CENTRO ROBOT DE CADA PUNTA DE CADA PATA
{Punta_Pata_RespectoCentro_extendida[1].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[1].x + POS_EXTENDIDA[1].x;
Punta_Pata_RespectoCentro_extendida[1].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[1].y + POS_EXTENDIDA[1].y;
Punta_Pata_RespectoCentro_extendida[1].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[1].z + POS_EXTENDIDA[1].z + altura;

Punta_Pata_RespectoCentro_extendida[2].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[2].x + -(POS_EXTENDIDA[2].x);
Punta_Pata_RespectoCentro_extendida[2].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[2].y + POS_EXTENDIDA[2].y;
Punta_Pata_RespectoCentro_extendida[2].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[2].z + POS_EXTENDIDA[2].z + altura;

Punta_Pata_RespectoCentro_extendida[3].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[3].x + POS_EXTENDIDA[3].x;
Punta_Pata_RespectoCentro_extendida[3].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[3].y + POS_EXTENDIDA[3].y;
Punta_Pata_RespectoCentro_extendida[3].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[3].z + POS_EXTENDIDA[3].z + altura;

Punta_Pata_RespectoCentro_extendida[4].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[4].x + -(POS_EXTENDIDA[4].x);
Punta_Pata_RespectoCentro_extendida[4].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[4].y + POS_EXTENDIDA[4].y;
Punta_Pata_RespectoCentro_extendida[4].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[4].z + POS_EXTENDIDA[4].z + altura;

Punta_Pata_RespectoCentro_extendida[5].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[5].x + POS_EXTENDIDA[5].x;
Punta_Pata_RespectoCentro_extendida[5].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[5].y + POS_EXTENDIDA[5].y;
Punta_Pata_RespectoCentro_extendida[5].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[5].z + POS_EXTENDIDA[5].z + altura;

Punta_Pata_RespectoCentro_extendida[6].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[6].x + -(POS_EXTENDIDA[6].x);
Punta_Pata_RespectoCentro_extendida[6].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[6].y + POS_EXTENDIDA[6].y;
Punta_Pata_RespectoCentro_extendida[6].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[6].z + POS_EXTENDIDA[6].z + altura;}

// CALCULO POSICION CIRUCLAR PREDEFINIDA RESPECTO A CENTRO ROBOT DE CADA PUNTA DE CADA PATA
{Punta_Pata_RespectoCentro_circular[1].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[1].x + POS_CIRCULAR[1].x;
Punta_Pata_RespectoCentro_circular[1].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[1].y + POS_CIRCULAR[1].y;
Punta_Pata_RespectoCentro_circular[1].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[1].z + POS_CIRCULAR[1].z + altura;

Punta_Pata_RespectoCentro_circular[2].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[2].x + -(POS_CIRCULAR[2].x);
Punta_Pata_RespectoCentro_circular[2].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[2].y + POS_CIRCULAR[2].y;
Punta_Pata_RespectoCentro_circular[2].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[2].z + POS_CIRCULAR[2].z + altura;

Punta_Pata_RespectoCentro_circular[3].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[3].x + POS_CIRCULAR[3].x;
Punta_Pata_RespectoCentro_circular[3].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[3].y + POS_CIRCULAR[3].y;
Punta_Pata_RespectoCentro_circular[3].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[3].z + POS_CIRCULAR[3].z + altura;

Punta_Pata_RespectoCentro_circular[4].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[4].x + -(POS_CIRCULAR[4].x);
Punta_Pata_RespectoCentro_circular[4].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[4].y + POS_CIRCULAR[4].y;
Punta_Pata_RespectoCentro_circular[4].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[4].z + POS_CIRCULAR[4].z + altura;

Punta_Pata_RespectoCentro_circular[5].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[5].x + POS_CIRCULAR[5].x;
Punta_Pata_RespectoCentro_circular[5].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[5].y + POS_CIRCULAR[5].y;
Punta_Pata_RespectoCentro_circular[5].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[5].z + POS_CIRCULAR[5].z + altura;

Punta_Pata_RespectoCentro_circular[6].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[6].x + -(POS_CIRCULAR[6].x);
Punta_Pata_RespectoCentro_circular[6].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[6].y + POS_CIRCULAR[6].y;
Punta_Pata_RespectoCentro_circular[6].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[6].z + POS_CIRCULAR[6].z + altura;}

// CALCULO POSICION REPOSO1 PREDEFINIDA RESPECTO A CENTRO ROBOT DE CADA PUNTA DE CADA PATA
{Punta_Pata_RespectoCentro_REPOSO1[1].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[1].x + baseReposo1[1].x;
Punta_Pata_RespectoCentro_REPOSO1[1].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[1].y + baseReposo1[1].y;
Punta_Pata_RespectoCentro_REPOSO1[1].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[1].z + baseReposo1[1].z;

Punta_Pata_RespectoCentro_REPOSO1[2].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[2].x + -(baseReposo1[2].x);
Punta_Pata_RespectoCentro_REPOSO1[2].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[2].y + baseReposo1[2].y;
Punta_Pata_RespectoCentro_REPOSO1[2].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[2].z + baseReposo1[2].z;

Punta_Pata_RespectoCentro_REPOSO1[3].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[3].x + baseReposo1[3].x;
Punta_Pata_RespectoCentro_REPOSO1[3].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[3].y + baseReposo1[3].y;
Punta_Pata_RespectoCentro_REPOSO1[3].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[3].z + baseReposo1[3].z;

Punta_Pata_RespectoCentro_REPOSO1[4].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[4].x + -(baseReposo1[4].x);
Punta_Pata_RespectoCentro_REPOSO1[4].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[4].y + baseReposo1[4].y;
Punta_Pata_RespectoCentro_REPOSO1[4].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[4].z + baseReposo1[4].z;

Punta_Pata_RespectoCentro_REPOSO1[5].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[5].x + baseReposo1[5].x;
Punta_Pata_RespectoCentro_REPOSO1[5].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[5].y + baseReposo1[5].y;
Punta_Pata_RespectoCentro_REPOSO1[5].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[5].z + baseReposo1[5].z;

Punta_Pata_RespectoCentro_REPOSO1[6].x = CONST_CentroRobot.x + CONST_POSICNES_ORIGEN_PATA[6].x + -(baseReposo1[6].x);
Punta_Pata_RespectoCentro_REPOSO1[6].y = CONST_CentroRobot.y + CONST_POSICNES_ORIGEN_PATA[6].y + baseReposo1[6].y;
Punta_Pata_RespectoCentro_REPOSO1[6].z = CONST_CentroRobot.z + CONST_POSICNES_ORIGEN_PATA[6].z + baseReposo1[6].z;}



//ESTRUCTURA COMANDOS ROBOT 

  struct COMANDOS {
    /* SOLICITUDES */
  bool SOLICITUD_NINGUNA;
  bool SOLICITUD_LEVANTAR;
  bool SOLICITUD_CALIBRAR;
  bool SOLICITUD_ACOSTAR;
  bool SOLICITUD_PARARSE;
  bool SOLICITUD_MOVERSE_ESTATICO;
  bool SOLICITUD_MOVERSE_CANGREJO;
  bool SOLICITUD_MOVERSE_NORMAL;
  bool SOLICITUD_DESCANSAR;
  /* ESTADO */
  bool ESTADO_LEVANTADO;
  bool ESTADO_ACOSTADO;
  bool ESTADO_DESCANSADO;
  bool ESTADO_ESTATICO;
  bool ESTADO_CANGREJO;
  bool ESTADO_NORMAL;
  bool ESTADO_PARADO;
  bool ESTADO_CALIBRANDO;
  bool ESTADO_NINGUNO;
  //MODO
  bool NORMAL;
  bool CALIBRACION;
};

//ESTRUCTURA ACCIONES ROBOT 

struct ACCIONES {
  /* ACCIONES */
  bool ACCION_NINGUNA;
  bool ACCION_CALIBRAR;
  bool ACCION_LEVANTARSE;
  bool ACCION_ACOSTARSE;
  bool ACCION_DESCANSARSE;
  bool ACCION_MOVERSE_ESTATICO;
  bool ACCION_MOVERSE_CANGREJO;
  bool ACCION_MOVERSE_NORMAL;
  int ACCION_POSICIONARSE;
  int CONTROL_POSICION;
  bool ACCION_PARAR;
};

static COMANDOS C = {};
(void) C;

static ACCIONES A = {};
(void) A;

////////////////////////////////////////////////////////////////////////// GESTOR DE BOTONS PS5 /////////////////////////////////////////////////////////
{ //============================================== GESTION CALIBRANDO Y BOTON PS ================================================
if (CALIBRANDO && PSButton) {CALIBRANDOaceptado=1;} else {CALIBRANDOaceptado=0;}

}

{ //============================================== BOTON X ================================================
if (CROSS && C.ESTADO_ACOSTADO) {C.SOLICITUD_LEVANTAR=1;}
if (CROSS && C.ESTADO_LEVANTADO) {C.SOLICITUD_ACOSTAR=1;}
}

{//============================================== BOTON REDONDO ================================================
if (CIRCLE==true) {C.SOLICITUD_MOVERSE_ESTATICO=1;}
}

{//============================================== BOTON TRIANGULO ================================================
if (TRIANGLE==true) {C.SOLICITUD_MOVERSE_NORMAL=1;}
}

{//============================================== BOTON QUADRADO ================================================
if (SQUARE==true) {C.SOLICITUD_MOVERSE_CANGREJO=1;}
}

{//============================================== BOTON OPTIONS ==========================================
// COMUTACION BOTON OPCIONES PS5

  if (OPTIONS && !BOTONopcionesPREV) {
  // levantarse robot
  BOTONopciones = !BOTONopciones; // Cambia el estado de levantado
  } 
  BOTONopcionesPREV = OPTIONS; // Actualiza el estado previo 
  }

{// ============================================= GESTIÓN ALTURA CON REPETICIÓN CORRECTA =================

// Constantes
const unsigned long retardoInicial = 400;   // Primer retardo tras pulsar (ms)
const unsigned long retardoRepeticion = 80; // Repetición continua (ms)

// Temporizadores independientes
static unsigned long tiempoUltimoUP = 0;
static unsigned long tiempoUltimoDOWN = 0;

// ↑ SUBIR
if (UP && C.ESTADO_LEVANTADO) {
  if (!UPprev || millis() - tiempoUltimoUP >= (UPprev ? retardoRepeticion : retardoInicial)) {
    alturaBase += pasoAltura;
    if (alturaBase > alturaMax) alturaBase = alturaMax;
    tiempoUltimoUP = millis();
  }
}

// ↓ BAJAR
if (DOWN && C.ESTADO_LEVANTADO) {
  if (!DOWNprev || millis() - tiempoUltimoDOWN >= (DOWNprev ? retardoRepeticion : retardoInicial)) {
    alturaBase -= pasoAltura;
    if (alturaBase < alturaMin) alturaBase = alturaMin;
    tiempoUltimoDOWN = millis();
  }
}

// Actualización de flags previos
UPprev = UP;
DOWNprev = DOWN;

// Aplicar altura
altura = alturaBase;
}

/////////////////////////////////////////////////////////////////////////// GESTOR DE SOLICITUDES //////////////////////////////////////////////////////

static bool BLOQUEAR = 0;
if (!BLOQUEAR) {C={0};C.ESTADO_PARADO=1; C.ESTADO_ACOSTADO=1; A.ACCION_NINGUNA=1;}

bool OK = 0;
int CountOK = 0;

while (!OK && CountOK <= 10)
{
  
  //=====================COMUTACION ENTRE MODO NORMAL Y MODO CALIBRACION Y SUS RESPECTIVOS MODOS============================
  /*Si se recibe desde el esp32 que el robot esta calibrando, comprobar si el robot esta acostado, si es asi, pasar al modo calibracion, si no es asi, pasar al modo
  acostarse, y despues a modo calibrar  */ 
  if (CALIBRANDOaceptado) {C.SOLICITUD_CALIBRAR=1;} else {C.SOLICITUD_CALIBRAR=0;}
  if (C.SOLICITUD_CALIBRAR) {if (C.ESTADO_CALIBRANDO) {C.SOLICITUD_CALIBRAR=0; OK=1;} else {if (C.ESTADO_ACOSTADO) {A.ACCION_CALIBRAR=1; OK=1;}else{C.SOLICITUD_ACOSTAR=1;}}}

  //=======================================GESTION ACOSTARSE================================                                                                             
  /*Si se recive una solicitud de acostarse , si ya esta acostado, entonces quita la solicitud, si no esta acostado entonces si esta parado manda la orden de acostarse, 
  si no esta parado entonces solicita pararse*/                                                                                                                          
  if (C.SOLICITUD_ACOSTAR) {if (C.ESTADO_ACOSTADO) {C.SOLICITUD_ACOSTAR=0; OK=1;}else{if (C.ESTADO_PARADO) {A.ACCION_ACOSTARSE=1; OK=1;}else{C.SOLICITUD_PARARSE=1;}} }

  //=======================================GESTION PARARSE================================     
  if (C.SOLICITUD_PARARSE) {if (C.ESTADO_PARADO) {C.SOLICITUD_PARARSE=0; OK=1;}else{A.ACCION_PARAR =1; OK=1;}}

  //=======================================GESTION LEVANTARSE================================
  /*Solo se lleva a cabo la accion de levantar si esta parado y en el suelo*/
  if (C.SOLICITUD_LEVANTAR) {if (C.ESTADO_LEVANTADO) {C.SOLICITUD_LEVANTAR=0; OK=1;}else{if(C.ESTADO_PARADO){A.ACCION_LEVANTARSE=1; OK=1;}else{C.SOLICITUD_PARARSE=1;}} }

  //=======================================GESTION MOVERSE ESTATICO===================================
  if (C.SOLICITUD_MOVERSE_ESTATICO) {if (C.ESTADO_ESTATICO||!C.ESTADO_LEVANTADO) {C.SOLICITUD_MOVERSE_ESTATICO=0; OK=1;}else{if (C.ESTADO_PARADO) {A.ACCION_MOVERSE_ESTATICO=1; OK=1;}else{C.SOLICITUD_PARARSE=1;;}} }

  //=======================================GESTION MOVERSE CANGREJO===================================
  if (C.SOLICITUD_MOVERSE_CANGREJO) {if (C.ESTADO_CANGREJO||!C.ESTADO_LEVANTADO) {C.SOLICITUD_MOVERSE_CANGREJO=0; OK=1;}else{if (C.ESTADO_PARADO) {A.ACCION_MOVERSE_CANGREJO=1; OK=1;}else{C.SOLICITUD_PARARSE=1;}} }

  //=======================================GESTION MOVERSE NORMAL===================================
  //if (C.SOLICITUD_MOVERSE_NORMAL) {if (C.ESTADO_NORMAL||!C.ESTADO_LEVANTADO) {C.SOLICITUD_MOVERSE_NORMAL=0; OK=1;}else{if (C.ESTADO_PARADO) {A.ACCION_MOVERSE_NORMAL=1; OK=1;}else{C.SOLICITUD_PARARSE=1;}} }

  CountOK = CountOK + 1;
};

/////////////////////////////////////////////////////////////////////////// ENVIO DE DATOS UART //////////////////////////////////////////////////////
{
  HaciaESP32.LEVANTADO = C.ESTADO_LEVANTADO ? true : false ;
  HaciaESP32.ACOSTADO = C.ESTADO_ACOSTADO ? true : false ;
  HaciaESP32.CALIBRANDO = C.ESTADO_CALIBRANDO ? true : false ;
  HaciaESP32.Altura = altura;
  HaciaESP32.Yaw = POS_CALCULADO_CENTRO_ROBOT.yaw;
  HaciaESP32.Pitch = POS_CALCULADO_CENTRO_ROBOT.pitch;
  HaciaESP32.Roll = POS_CALCULADO_CENTRO_ROBOT.roll;
}


// ============================================================================ GESTION DE ACCIONES =============================================================================

static bool BloqueoEstadoLevantarse = false;
static bool BloqueoEstadoAcostarse = false;
static bool BloqueoEstadoMoverseESTATICO = false;
static bool BloqueoEstadoMoverseCANGREJO = false;
//static bool BloqueoEstadoMoverseNORMAL = false;
static int EstadoLevantarse = 0;
static int EstadoAcostarse = 0;
static int EstadoMoverseESTATICO = 0;
static int EstadoMoverseCANGREJO = 0;
static int EstadoPOSICIONAMIENTO = 0;
//static int EstadoMoverseNORMAL = 0;

// ============================================================================ ACCION PARAR =============================================================================
if(A.ACCION_PARAR)
{

}


// ============================================================================ ACCION CALIBRAR =============================================================================
if (A.ACCION_CALIBRAR) {
   
  /*ROBOT MODO CALIBRANDO*/
  for (int i = 0; i < 18; i++) {
        servos.value(i, CalServo[i]); // Manda el ángulo directo a cada servo
    }
  C.ESTADO_CALIBRANDO=true;
  servos.load();

}else{C.ESTADO_CALIBRANDO=false;}


// ============================================================================ ACCION LEVANTARSE =============================================================================
if (A.ACCION_LEVANTARSE) {

  if (!BloqueoEstadoLevantarse) {servos.enable_all(), EstadoLevantarse = 1, BloqueoEstadoLevantarse = true; A={0}; A.ACCION_LEVANTARSE=true; C.ESTADO_ACOSTADO = false;}

  if (EstadoLevantarse == 1)  // =================================== ALEJAR PATA 2 Y 5 ==============================================
  {
     if (A.CONTROL_POSICION != 1) {A.ACCION_POSICIONARSE =1;
  } else
  {EstadoLevantarse=2; A.ACCION_POSICIONARSE =0;}
  }  

  else if (EstadoLevantarse == 2)   // =================================== ALEJAR PATA 3 Y 4 ==============================================
  {
  C.ESTADO_LEVANTADO=true;
  } 

} else {BloqueoEstadoLevantarse = false, TON[1].reset();}


// ============================================================================ ACCION ACOSTARSE =============================================================================
if (A.ACCION_ACOSTARSE) {

  if (!BloqueoEstadoAcostarse) {EstadoAcostarse = 1, BloqueoEstadoAcostarse = true; C.ESTADO_LEVANTADO = false; A={0}; A.ACCION_ACOSTARSE=true;}

  if (EstadoAcostarse == 1)  // =================================== ALEJAR PATA 2 Y 5 ==============================================
  {
     if (A.CONTROL_POSICION != 2) {A.ACCION_POSICIONARSE =2;
  } else
  {EstadoAcostarse=2; A.ACCION_POSICIONARSE =0;}
  }  

  else if (EstadoAcostarse == 2)   // =================================== ALEJAR PATA 3 Y 4 ==============================================
  {
  C.ESTADO_ACOSTADO=true;
  } 


} else {BloqueoEstadoAcostarse = false, TON[2].reset();}


// ============================================================================ ACCION DESCANSARSE =============================================================================
if (A.ACCION_DESCANSARSE) {

}


// ============================================================================ ACCION MOVERSE ESTATICO =============================================================================
if (A.ACCION_MOVERSE_ESTATICO) {

  if (!BloqueoEstadoMoverseESTATICO) {C.ESTADO_ESTATICO=true; EstadoMoverseESTATICO = 1, BloqueoEstadoMoverseESTATICO = true, TON[5].reset(); A={0}; A.ACCION_MOVERSE_ESTATICO=true;}

   if (EstadoMoverseESTATICO==1) {
    if (A.CONTROL_POSICION != 3) {A.ACCION_POSICIONARSE =3;
  } else
  {EstadoMoverseESTATICO=2; A.ACCION_POSICIONARSE =0;}
   }

   if (EstadoMoverseESTATICO==2) // ============================ MODIFICAR POS EXTENDIDA CON VALORES JOISTICK ==============================================
    {
      static int jdxMap = 0;
      static int jdyMap = 0;

      static unsigned long ANTERIOR = 0;
      static unsigned long INTERVALO = 50; // ms

        if(ANTERIOR + INTERVALO < millis()) {
          jdxMap = map(JixVal, -128, 127, -65, 65);
          jdyMap = map(JiyVal, -128, 127, -65, 65);
          ANTERIOR = millis();
        }

      for (int i = 1; i <= 6; i++){   
      Punta_Pata_RespectoCentro_GUARDADO[i].x = Punta_Pata_RespectoCentro_extendida[i].x - jdxMap;
      Punta_Pata_RespectoCentro_GUARDADO[i].y = Punta_Pata_RespectoCentro_extendida[i].y - jdyMap;
      Punta_Pata_RespectoCentro_GUARDADO[i].z = Punta_Pata_RespectoCentro_extendida[i].z;
      VelocidadMovimiento[i] = 100; PARABOLICO[i] = 0;
      }

      if ((abs(JixVal) >= 7) || (abs(JiyVal) >= 7)) {C.ESTADO_PARADO=false;} else {C.ESTADO_PARADO=true;}
    }


} else {C.ESTADO_ESTATICO=false; BloqueoEstadoMoverseESTATICO = false, EstadoMoverseESTATICO = 0, TON[5].reset();}


// ============================================================================ ACCION MOVERSE CANGREJO =============================================================================
if (A.ACCION_MOVERSE_CANGREJO) {
  static bool PrimerEstadoMoverseCangrejo;
  static unsigned long He_Llegado_Al_Paso_2;
  bool REINICIAR = false;

  if (!BloqueoEstadoMoverseCANGREJO) {C.ESTADO_CANGREJO = true; EstadoMoverseCANGREJO = 1, BloqueoEstadoMoverseCANGREJO = true; A={0}; A.ACCION_MOVERSE_CANGREJO=true; PrimerEstadoMoverseCangrejo=true;}

  if (EstadoMoverseCANGREJO==1) {
    if (A.CONTROL_POSICION != 4) {A.ACCION_POSICIONARSE =4;
  } else
  {EstadoMoverseCANGREJO=2; A.ACCION_POSICIONARSE =0;}
  }

  if (EstadoMoverseCANGREJO == 2) {

  for (int i = 1; i <= 6; i++) {      
  Punta_Pata_RespectoCentro_GUARDADO[i] = Punta_Pata_RespectoCentro_ACTUAL[i];
  VelocidadMovimiento[i] = 100; PARABOLICO[i] = 0; }
  He_Llegado_Al_Paso_2 = millis();       
  EstadoMoverseCANGREJO=3;
  PrimerEstadoMoverseCangrejo = true;

}

  if (EstadoMoverseCANGREJO == 3) {

  static float DistanciaPataDesdeCentro[7] = {};       // Variable para saber la distancia desde el centro de la postura seleccionada hasta la punta de la pata
  static float DirX[7];                                // Direccion X del Joistick normalizada
  static float DirY[7];                                // Direccion Y del Joistick normalizada
  static float DistanciaJoistickIzquierdo;             // Variable para saber la distancia del Joistick Izquierdo respecto al centro
  static bool  enZonaMuerta;                           // Variable para saber si el Joistick esta en zona muerta
  static float PasoAvance[7];                          // Variable para saber los milimetros a avanzar en este ciclo para cumplir con la velocidad seleccionada
  static float MAX;                                    // Variable para saber la distancia de paso maximo segun los botones R1 y L1
  static float Parabolica[7];                          // Variable para la altura parabolica simple
  static bool ParabolicaActivaTRIPOD[7]={0, 0,1,1,0,0,1};    // Variable para saber si la pata es del gait TRIPOD activo
  static bool ParabolicaActivaBIPOD[7]={0, 1,0,0,1,0,0};
  static bool ParabolicaActiva[7]={0, 0,1,1,0,0,1};;
  static bool CambioRealizado[7] = {};                 // Variable para saber si se ha realizado el cambio de tripode en este ciclo
  static bool HaciaPostura[7] = {0, 0,0,0,0,0,0};      // Variable para saber si la pata se esta moviendo hacia la postura seleccionada
  static float DistHastaBorde[7];                      // Distancia hasta el borde siguiendo la trayectoria actual
  static float MultiplicadorAvanze[7];                 // Multiplicador de avanze que acelera o frena las patas en el aire para que lleguen todas a la vez al destino
  static bool PataQuieta[7];                           // Variable para saber si la pata esta quieta

  bool FlancoOPTIONS = FlancoSubida(OPTIONS);
  static bool GaitTripod = true;
  static bool GaitBipod = false;

  if(FlancoOPTIONS && GaitTripod) {GaitBipod = true; GaitTripod = false; for (int i = 1; i <=6; i++) ParabolicaActiva[i] = ParabolicaActivaBIPOD[i]; REINICIAR = true;}
  else if(FlancoOPTIONS && GaitBipod) {GaitBipod = false; GaitTripod = true; for (int i = 1; i <=6; i++) ParabolicaActiva[i] = ParabolicaActivaTRIPOD[i]; REINICIAR = true;}
  
  if (SQUARE) {REINICIAR = true;}

  if (REINICIAR) {
    // Selecciona el patrón activo según el gait actual
    if (GaitTripod) { for (int i = 1; i <= 6; i++) ParabolicaActiva[i] = ParabolicaActivaTRIPOD[i]; }
    if (GaitBipod)  { for (int i = 1; i <= 6; i++) ParabolicaActiva[i] = ParabolicaActivaBIPOD[i];  }

    // Rearmar modo cangrejo "a lo bruto" sin pasar por el scheduler
    C.ESTADO_CANGREJO = true;
    A={0}; A.ACCION_MOVERSE_CANGREJO = true;

    EstadoMoverseCANGREJO = 1;                 // vuelve a PASO 1 (posicionar a circular)
    PrimerEstadoMoverseCangrejo = true;
    He_Llegado_Al_Paso_2 = 0;                  // fuerza la ventana anti-zona-muerta

    for (int i = 1; i <= 6; i++) {
      CambioRealizado[i] = false;
      HaciaPostura[i]    = false;
      // opcional: clamp/parámetros de ayuda
      // Parabolica[i] = 0.0f;                  // si quieres reiniciar altura
    }

    // Opcional: si quieres que *visualmente* agarre POS_CIRCULAR ya:
    for (int i = 1; i <= 6; i++) {
      Punta_Pata_RespectoCentro_GUARDADO[i] = Punta_Pata_RespectoCentro_ACTUAL[i];
      VelocidadMovimiento[i] = 100; PARABOLICO[i] = 0;
    }

    REINICIAR = false;                         // consume el reinicio
  }

  if (PrimerEstadoMoverseCangrejo) {
    for (int i = 1; i <=6; i++) CambioRealizado[i] = false;
    for (int i = 1; i <=6; i++) HaciaPostura[i] = 0;
  }


  #define Postura()                     Punta_Pata_RespectoCentro_circular
  int ZonaMuertaJoistick =              DesdeESP32.ZonaMuertaJoistick == 0 ? 10 : DesdeESP32.ZonaMuertaJoistick;
  float Velocidad_Max_mm_s =            DesdeESP32.Velocidad_Max_mm_s == 0 ? 150.0f : (float)DesdeESP32.Velocidad_Max_mm_s;
  float Paso1_mm =                      DesdeESP32.Paso1_mm == 0 ? 30.0f : (float)DesdeESP32.Paso1_mm;
  float Paso2_mm =                      DesdeESP32.Paso2_mm == 0 ? 40.0f : (float)DesdeESP32.Paso2_mm;
  float Paso3_mm =                      DesdeESP32.Paso3_mm == 0 ? 50.0f : (float)DesdeESP32.Paso3_mm;
  float Velocidad_Bajada_Patas_mm_s =   DesdeESP32.Velocidad_Bajada_Patas_mm_s == 0 ? 50.0f : (float)DesdeESP32.Velocidad_Bajada_Patas_mm_s;
  float Velocidad_Subida_Patas_mm_s =   DesdeESP32.Velocidad_Subida_Patas_mm_s == 0 ? 120.0f : (float)DesdeESP32.Velocidad_Subida_Patas_mm_s;
  float MultiplicadorParabola =         DesdeESP32.MultiplicadorParabola == 0 ? 1.8f : (float)DesdeESP32.MultiplicadorParabola;
  float MultiplicadorAyudaPatas =       DesdeESP32.MultiplicadorAyudaPatas == 0 ? 0.5f : (float)DesdeESP32.MultiplicadorAyudaPatas;


    // DistanciaPata[i]
    for (int i = 1; i <= 6; i++)  {
      DistanciaPataDesdeCentro[i] = sqrt(pow((Postura()[i].x - Punta_Pata_RespectoCentro_GUARDADO[i].x),2) 
                                + pow((Postura()[i].y - Punta_Pata_RespectoCentro_GUARDADO[i].y),2));
    }
    
    // DistanciaJoistickDerecho && DistanciaJoistickIzquierdo
    { 
      DistanciaJoistickIzquierdo = map (sqrt(JixVal * JixVal + JiyVal * JiyVal), 0, 128, 0, 100);
      // DistanciaJoistickDerecho = map (sqrt(JdxVal * JdxVal + JdyVal * JdyVal), 0, 128, 0, 100);
    }
    
    // DirX -- DirY -- enZonaMuerta
    for (int i = 1; i <= 6; i++)  {
      const int ZonaMuerta = ZonaMuertaJoistick;
      enZonaMuerta = (millis() < He_Llegado_Al_Paso_2+500) || ((abs(JixVal) < ZonaMuerta) && (abs(JiyVal) < ZonaMuerta));

      float AnguloJoystickIzquierdo = atan2f((float)JiyVal, (float)JixVal);
      float DireccionHacia_J_I_X = cosf(AnguloJoystickIzquierdo);
      float DireccionHacia_J_I_Y = sinf(AnguloJoystickIzquierdo);

      float AnguloJoystickDerecho = atan2f((float)JdyVal, (float)JdxVal);
      float DireccionHacia_J_D_X = cosf(AnguloJoystickDerecho); (void) DireccionHacia_J_D_X;
      float DireccionHacia_J_D_Y = sinf(AnguloJoystickDerecho); (void) DireccionHacia_J_D_Y;

      // 1) Vector desde GUARDADO hasta POS_CIRCULAR
      float dx = Postura()[i].x - Punta_Pata_RespectoCentro_GUARDADO[i].x;
      float dy = Postura()[i].y - Punta_Pata_RespectoCentro_GUARDADO[i].y;

      // 2) Ángulo de esa dirección (igual que con el joystick)
      float AnguloCircular = atan2f(dy, dx);

      // 3) Dirección unitaria con cos/sin (igual que tu DirX/DirY)
      float DireccionHaciaPostura_X = cosf(AnguloCircular);
      float DireccionHaciaPostura_Y = sinf(AnguloCircular);

      DireccionHaciaPostura_X = ParabolicaActiva[i] ? DireccionHaciaPostura_X : -DireccionHaciaPostura_X;
      DireccionHaciaPostura_Y = ParabolicaActiva[i] ? DireccionHaciaPostura_Y : -DireccionHaciaPostura_Y;
      

      DirX[i] = (ParabolicaActiva[i] ? (HaciaPostura[i] ? DireccionHaciaPostura_X : DireccionHacia_J_I_X) : DireccionHacia_J_I_X);
      DirY[i] = (ParabolicaActiva[i] ? (HaciaPostura[i] ? DireccionHaciaPostura_Y : DireccionHacia_J_I_Y) : DireccionHacia_J_I_Y);


    }

    // MAX
    {
      MAX = L1 ? Paso1_mm : (R1 ? Paso3_mm : Paso2_mm) ;
    }

    // --- Distancia hasta el borde siguiendo la trayectoria actual ---
    for (int i = 1; i <= 6; i++)  {
      // r = P - C
      float rx = Punta_Pata_RespectoCentro_GUARDADO[i].x - Postura()[i].x;
      float ry = Punta_Pata_RespectoCentro_GUARDADO[i].y - Postura()[i].y;

      // Dirección efectiva según estado:
      //  - En el aire (ParabolicaActiva[i] == true): avanzas en +Dir
      //  - En el suelo (false): avanzas en -Dir (como en tu bloque de movimiento)
      float dxi = ParabolicaActiva[i] ? DirX[i] : -DirX[i];
      float dyi = ParabolicaActiva[i] ? DirY[i] : -DirY[i];

      // |r|^2 y r·d
      float rr  = rx*rx + ry*ry;
      float rd  = rx*dxi + ry*dyi;

      // s = -rd + sqrt( rd^2 - (|r|^2 - R^2) )
      float disc = rd*rd - (rr - MAX*MAX);
      if (disc < 0.0f) disc = 0.0f;        // robustez numérica
      float s = -rd + sqrtf(disc);
      if (!isfinite(s) || s < 0.0f) s = 0.0f;

      DistHastaBorde[i] = s;               // distancia válida en la *dirección real de avance*
    }

    // Calculo de multiplicador de avanze
    {
      if(GaitTripod){
        MultiplicadorAvanze[1] = fmin (DistHastaBorde[1] / DistHastaBorde[2], 3.0f);
        MultiplicadorAvanze[2] = fmin (DistHastaBorde[2] / DistHastaBorde[1], 3.0f);
        MultiplicadorAvanze[3] = fmin (DistHastaBorde[3] / DistHastaBorde[4], 3.0f);
        MultiplicadorAvanze[4] = fmin (DistHastaBorde[4] / DistHastaBorde[3], 3.0f);
        MultiplicadorAvanze[5] = fmin (DistHastaBorde[5] / DistHastaBorde[6], 3.0f);
        MultiplicadorAvanze[6] = fmin (DistHastaBorde[6] / DistHastaBorde[5], 3.0f);
      }

      if (GaitBipod) {
        MultiplicadorAvanze[1] = fmin( fmax( DistHastaBorde[1] / DistHastaBorde[2], DistHastaBorde[1] / DistHastaBorde[3] ), 3.0f ); 
        MultiplicadorAvanze[2] = fmin( fmax( DistHastaBorde[2] / DistHastaBorde[1], DistHastaBorde[2] / DistHastaBorde[4] ), 3.0f ); 
        MultiplicadorAvanze[3] = fmin( fmax( DistHastaBorde[3] / DistHastaBorde[1], DistHastaBorde[3] / DistHastaBorde[5] ), 3.0f ); 
        MultiplicadorAvanze[4] = fmin( fmax( DistHastaBorde[4] / DistHastaBorde[2], DistHastaBorde[4] / DistHastaBorde[6] ), 3.0f ); 
        MultiplicadorAvanze[5] = fmin( fmax( DistHastaBorde[5] / DistHastaBorde[3], DistHastaBorde[5] / DistHastaBorde[6] ), 3.0f ); 
        MultiplicadorAvanze[6] = fmin( fmax( DistHastaBorde[6] / DistHastaBorde[5], DistHastaBorde[6] / DistHastaBorde[4] ), 3.0f );
      }


      // if (GaitBipod) {
      //   const float LIM = 2.0f;
      //   const float EPS = 0.001f;

      //   // Referencia que tiene en cuenta TODAS las patas:
      //   //   - media de distancias del grupo en el aire (siempre >= EPS)
      //   float suma = 0.0f; int n = 0;
      //   for (int k = 1; k <= 6; k++) {
      //     if (ParabolicaActiva[k]) { suma += DistHastaBorde[k]; n++; }
      //   }
      //   if (n == 0) n = 1;
      //   float RefALL = fmaxf(suma / (float)n, EPS);

      //   // Si quieres ponderar también las que están en suelo, activa esto:
      //   // float suma = 0.0f; float wsum = 0.0f;
      //   // for (int k = 1; k <= 6; k++) {
      //   //   float w = ParabolicaActiva[k] ? 1.0f : 0.33f;   // el suelo también cuenta (poco)
      //   //   suma += w * DistHastaBorde[k];
      //   //   wsum += w;
      //   // }
      //   // float RefALL = fmaxf(suma / fmaxf(wsum,1.0f), EPS);

      //   // Multiplicadores usando esa referencia común (todas las patas influencian la división)
      //   MultiplicadorAvanze[1] = fminf(DistHastaBorde[1] / RefALL, LIM);
      //   MultiplicadorAvanze[2] = fminf(DistHastaBorde[2] / RefALL, LIM);
      //   MultiplicadorAvanze[3] = fminf(DistHastaBorde[3] / RefALL, LIM);
      //   MultiplicadorAvanze[4] = fminf(DistHastaBorde[4] / RefALL, LIM);
      //   MultiplicadorAvanze[5] = fminf(DistHastaBorde[5] / RefALL, LIM);
      //   MultiplicadorAvanze[6] = fminf(DistHastaBorde[6] / RefALL, LIM);
      // }
    }

    // PasoAvance
    {
      float Velocidad_Seleccionada_mm_x_ms = ((Velocidad_Max_mm_s * (DistanciaJoistickIzquierdo / 100)) / 1000) * MAX/Paso3_mm;  //Velocidad seleccionada teniendo en cuenta la distancia del Joistick Izquierdo

      static long ANTERIOR = 0;
      long AHORA = millis();

      float TiempoTranscurrido_ms = AHORA - ANTERIOR; // Calculo del tiempo transcurrido en ms respecto al ultimo ciclo
      ANTERIOR = AHORA;

      if(GaitTripod){
        for (int i = 1; i <= 6; i++)  {
          // calculo de mm necesarios para este ciclo para cumplir con Velocidad_Seleccionada
          PasoAvance[i] = enZonaMuerta? 0.0 : Velocidad_Seleccionada_mm_x_ms * TiempoTranscurrido_ms * (ParabolicaActiva[i] ? MultiplicadorAvanze[i] + MultiplicadorAyudaPatas : 1); 
        }
      }

      if(GaitBipod){
        for (int i = 1; i <= 6; i++)  {
          // calculo de mm necesarios para este ciclo para cumplir con Velocidad_Seleccionada
          PasoAvance[i] = (enZonaMuerta? 0.0 : Velocidad_Seleccionada_mm_x_ms * TiempoTranscurrido_ms * (ParabolicaActiva[i] ? (MultiplicadorAvanze[i]*3) + MultiplicadorAyudaPatas : 1)/2); 
        }
      }
      
    }

    // PataQuieta
    for (int i = 1; i <= 6; i++)  {
      if (enZonaMuerta) {PataQuieta[i] = true;} else {PataQuieta[i] = false;}
    }

    // Parabolica
    for (int i = 1; i <= 6; i++)  {
      
      float Parabola_Normal = (MAX - DistanciaPataDesdeCentro[i]) * MultiplicadorParabola;            // Parabolica normal si las patas estan em movimiento
      
      //Velocidad seleccionada teniendo en cuenta si necesita subir o bajar         
      float Velocidad_Seleccionada_mm_x_ms = (PataQuieta[i] ? Velocidad_Bajada_Patas_mm_s : Velocidad_Subida_Patas_mm_s) / 1000.0f;  

      static long ANTERIOR[7] = {};
      long AHORA = millis();

      float TiempoTranscurrido_ms = AHORA - ANTERIOR[i];
      ANTERIOR[i] = AHORA;

      // subir hacia Parabola_Normal
      if (!PataQuieta[i]) {Parabolica[i] = Parabolica[i] + Velocidad_Seleccionada_mm_x_ms * TiempoTranscurrido_ms;}
      if (PataQuieta[i]) {Parabolica[i] = Parabolica[i] - Velocidad_Seleccionada_mm_x_ms * TiempoTranscurrido_ms;}
      Parabolica[i] = constrain(Parabolica[i], 0.0f, Parabola_Normal);

    }
      
    // =================================================== MOVIMIENTO ===================================================
    for (int i = 1; i <= 6; i++)  {

    static int PatasNecesariasCambio[4];
    static bool PermisoCambioDeCompañeras;

    if(GaitTripod){
      if (i == 1) {PatasNecesariasCambio[1] = 2; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 6;}
      if (i == 2) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 4; PatasNecesariasCambio[3] = 5;}
      if (i == 3) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 4; PatasNecesariasCambio[3] = 5;}
      if (i == 4) {PatasNecesariasCambio[1] = 2; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 6;}
      if (i == 5) {PatasNecesariasCambio[1] = 2; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 6;}
      if (i == 6) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 4; PatasNecesariasCambio[3] = 5;}
      PermisoCambioDeCompañeras = 
        !ParabolicaActiva[PatasNecesariasCambio[1]] && 
        !ParabolicaActiva[PatasNecesariasCambio[2]] && 
        !ParabolicaActiva[PatasNecesariasCambio[3]];
    }


    if(GaitBipod){
      if (i == 1) {PatasNecesariasCambio[1] = 2; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 5; PatasNecesariasCambio[4] = 6;} // pareja 4
      if (i == 2) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 4; PatasNecesariasCambio[4] = 6;} // pareja 5
      if (i == 3) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 2; PatasNecesariasCambio[3] = 4; PatasNecesariasCambio[4] = 5;} // pareja 6
      if (i == 4) {PatasNecesariasCambio[1] = 2; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 5; PatasNecesariasCambio[4] = 6;} // pareja 1
      if (i == 5) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 3; PatasNecesariasCambio[3] = 4; PatasNecesariasCambio[4] = 6;} // pareja 2
      if (i == 6) {PatasNecesariasCambio[1] = 1; PatasNecesariasCambio[2] = 2; PatasNecesariasCambio[3] = 4; PatasNecesariasCambio[4] = 5;} // pareja 3
      PermisoCambioDeCompañeras =
      !ParabolicaActiva[PatasNecesariasCambio[1]] &&
      !ParabolicaActiva[PatasNecesariasCambio[2]] &&
      !ParabolicaActiva[PatasNecesariasCambio[3]] &&
      !ParabolicaActiva[PatasNecesariasCambio[4]];
    }

    


      if (ParabolicaActiva[i]) {
        Punta_Pata_RespectoCentro_GUARDADO[i].x += DirX[i] * PasoAvance[i];
        Punta_Pata_RespectoCentro_GUARDADO[i].y += DirY[i] * PasoAvance[i];
        Punta_Pata_RespectoCentro_GUARDADO[i].z  = Punta_Pata_RespectoCentro_circular[i].z - Parabolica[i];
      } else { 
        Punta_Pata_RespectoCentro_GUARDADO[i].x -= DirX[i] * PasoAvance[i];
        Punta_Pata_RespectoCentro_GUARDADO[i].y -= DirY[i] * PasoAvance[i];
        Punta_Pata_RespectoCentro_GUARDADO[i].z  = Punta_Pata_RespectoCentro_circular[i].z;
      }

      if (ParabolicaActiva[i] && DistanciaPataDesdeCentro[i] >= MAX && !CambioRealizado[i]) {ParabolicaActiva[i] = !ParabolicaActiva[i]; CambioRealizado[i] = true;}
      if (!ParabolicaActiva[i] && PermisoCambioDeCompañeras && DistanciaPataDesdeCentro[i] >= MAX && !CambioRealizado[i]) {ParabolicaActiva[i] = !ParabolicaActiva[i]; CambioRealizado[i] = true;}
      if (DistanciaPataDesdeCentro[i] <= (MAX - 0.2f) && CambioRealizado[i]) {CambioRealizado[i] = false;}

      if (CambioRealizado[i]) {HaciaPostura[i] = true;}
      if (HaciaPostura[i] && ParabolicaActiva[i] && DistanciaPataDesdeCentro[i] <= (MAX - (MAX/2))) {HaciaPostura[i] = false;} // MOVER A POSTURA LAS PATAS QUE ESTAN EN EL AIRE
      if (HaciaPostura[i] && !ParabolicaActiva[i] && DistanciaPataDesdeCentro[i] <= (MAX - (MAX/6))) {HaciaPostura[i] = false;}    // MOVER A POSTURA LAS PATAS QUE ESTAN EN EL SUELO

      Punta_Pata_RespectoCentro_INTERP[i]  = Punta_Pata_RespectoCentro_GUARDADO[i];
    }

    PrimerEstadoMoverseCangrejo = false;

}



}else {C.ESTADO_CANGREJO = false; BloqueoEstadoMoverseCANGREJO = false, EstadoMoverseCANGREJO = 0, TON[6].reset();}


// ============================================================================ ACCION MOVERSE NORMAL =============================================================================
if (A.ACCION_MOVERSE_NORMAL) {


  
}


// ============================================================================ ACCION POSICIONARSE =============================================================================
if (A.ACCION_POSICIONARSE > 0) {

  static bool PataDesviada[7] = {};
  static bool PataDiferenciaAltura [7] = {};
  static int ULTIMA_DEMANDA = 0;

  static XYZfloat POSICION_A_LA_QUE_POSICIONARSE[7];
  static XYZfloat Diferencia_Posicion[7];

  static const int orden[6] = {3, 4, 1, 6, 2, 5};
  static int idx = 0;
  static int pata1 = 0;
  static int pata2 = 0;
  static bool BLOQUEO[20] = {};

  // RESET DEMANDA POSICIONAMIENTO SI EL OBJETIVO ES DIFERENTE
  if (ULTIMA_DEMANDA != A.ACCION_POSICIONARSE && patasTerminadas[1] && patasTerminadas[2] && patasTerminadas[3] && patasTerminadas[4] && patasTerminadas[5] && patasTerminadas[6]) 
  { EstadoPOSICIONAMIENTO = 0; 
    TON[25].reset();
    for (int i = 1; i <= 6; i++) {
      PataDesviada[i] = false;
      PataDiferenciaAltura[i] = false;
      Diferencia_Posicion[i] = {0,0,0};
    }
    idx = 0;
    BLOQUEO[1] = false;
    BLOQUEO[2] = false;
    BLOQUEO[3] = false;

  }


  if (EstadoPOSICIONAMIENTO == 0 && patasTerminadas[1] && patasTerminadas[2] && patasTerminadas[3] && patasTerminadas[4] && patasTerminadas[5] && patasTerminadas[6]) // COMPROBAR DIFERENCIAS ENTRE POSICIONES Y GUARDAR LA DIFERENCIA SI ES QUE HAY
  {

    ULTIMA_DEMANDA = A.ACCION_POSICIONARSE;

    //SELECTOR DE POSICIONES A LA QUE IR
    if (A.ACCION_POSICIONARSE == 1) { //POSICIONARSE MODO LEVANTADO
      for (int i = 1; i <= 6; i++) {
      POSICION_A_LA_QUE_POSICIONARSE[i] = Punta_Pata_RespectoCentro_extendida[i];
    }
    }

    if (A.ACCION_POSICIONARSE == 2) { //POSICIONARSE MODO ACOSTADO
      for (int i = 1; i <= 6; i++) {
      POSICION_A_LA_QUE_POSICIONARSE[i] = Punta_Pata_RespectoCentro_REPOSO1[i];
    }
    }

    if (A.ACCION_POSICIONARSE == 3) { //POSICIONARSE MODO ESTATICO
      for (int i = 1; i <= 6; i++) {
      POSICION_A_LA_QUE_POSICIONARSE[i] = Punta_Pata_RespectoCentro_extendida[i];
    }
    }

    if (A.ACCION_POSICIONARSE == 4) { //POSICIONARSE MODO CANGREJO
      for (int i = 1; i <= 6; i++) {
      POSICION_A_LA_QUE_POSICIONARSE[i] = Punta_Pata_RespectoCentro_circular[i];
    }    
    }


  

    //FUNCION PARA BORRAR DATOS
    for (int i = 1; i <= 6; i++){Diferencia_Posicion[i] = {0};}
    for (int i = 1; i <= 6; i++){PataDesviada[i] = {0};}
    for (int i = 1; i <= 6; i++){PataDiferenciaAltura[i] = {0};}


    //COMPARAR X,Y,Z De la DEMANDA vs las ACTUALES y si son diferentes guardarlas en DIFERENCIA POSICION 
    for (int i = 1; i <= 6; i++){ 

    if(fabs(POSICION_A_LA_QUE_POSICIONARSE[i].x - Punta_Pata_RespectoCentro_ACTUAL[i].x) >= 0.5f)
    {Diferencia_Posicion[i].x = POSICION_A_LA_QUE_POSICIONARSE[i].x - Punta_Pata_RespectoCentro_ACTUAL[i].x; PataDesviada[i]=true;}

    if(fabs(POSICION_A_LA_QUE_POSICIONARSE[i].y - Punta_Pata_RespectoCentro_ACTUAL[i].y) >= 0.5f)
    {Diferencia_Posicion[i].y = POSICION_A_LA_QUE_POSICIONARSE[i].y - Punta_Pata_RespectoCentro_ACTUAL[i].y; PataDesviada[i]=true;}

    if(fabs(POSICION_A_LA_QUE_POSICIONARSE[i].z - Punta_Pata_RespectoCentro_ACTUAL[i].z) >= 0.5f)
    {Diferencia_Posicion[i].z = POSICION_A_LA_QUE_POSICIONARSE[i].z - Punta_Pata_RespectoCentro_ACTUAL[i].z; PataDiferenciaAltura[i]=true;}
      
    
    Punta_Pata_RespectoCentro_GUARDADO[i] = Punta_Pata_RespectoCentro_ACTUAL[i]; VelocidadMovimiento[i] = 50; PARABOLICO[i] = 0;

    }


    EstadoPOSICIONAMIENTO = 1;
  }
      
  if (EstadoPOSICIONAMIENTO == 1) { // ============== PRIMERO ALZAR SI LA POSICION DESEADA ES MAS ALTA QUE LA ACTUAL  ========================

    if ((PataDiferenciaAltura[1] || PataDiferenciaAltura[2] || PataDiferenciaAltura[3] || PataDiferenciaAltura[4] || PataDiferenciaAltura[5] || PataDiferenciaAltura[6]) && (
    Diferencia_Posicion[1].z > 0 || Diferencia_Posicion[2].z > 0  || Diferencia_Posicion[3].z > 0 || Diferencia_Posicion[4].z > 0 || Diferencia_Posicion[5].z > 0 || Diferencia_Posicion[6].z > 0))
    {

    for (int i = 1; i <= 6; i++){
    if ((Diferencia_Posicion[i].z > 0) && (BLOQUEO[2] == false)) {
    Punta_Pata_RespectoCentro_GUARDADO[i].x = Punta_Pata_RespectoCentro_GUARDADO[i].x;
    Punta_Pata_RespectoCentro_GUARDADO[i].y = Punta_Pata_RespectoCentro_GUARDADO[i].y;
    Punta_Pata_RespectoCentro_GUARDADO[i].z = Punta_Pata_RespectoCentro_GUARDADO[i].z + Diferencia_Posicion[i].z;
    VelocidadMovimiento[i] = 30; PARABOLICO[i] = 0; }
    }

    TON[25].set(100);
    BLOQUEO[2] = true;
    
    if (TON[25].done() && patasTerminadas[1] && patasTerminadas[2] && patasTerminadas[3] && patasTerminadas[4] && patasTerminadas[5] && patasTerminadas[6])
    {EstadoPOSICIONAMIENTO = 2; TON[25].reset(); BLOQUEO[2] = false;}

  } else {EstadoPOSICIONAMIENTO = 2;}
  }

  if (EstadoPOSICIONAMIENTO == 2) // ============== MOVER PATAS DE 2 EN 2 ========================
{
  pata1 = orden[idx];     // primera pata del par
  pata2 = orden[idx + 1]; // segunda pata del par

  bool algunaDesviada = false;

  // -------- Pata 1 --------
  if (PataDesviada[pata1]) {
      VelocidadMovimiento[pata1] = 100;
      PARABOLICO[pata1] = 1;
      Punta_Pata_RespectoCentro_GUARDADO[pata1].x = POSICION_A_LA_QUE_POSICIONARSE[pata1].x;
      Punta_Pata_RespectoCentro_GUARDADO[pata1].y = POSICION_A_LA_QUE_POSICIONARSE[pata1].y;      
      algunaDesviada = true;
  }

  // -------- Pata 2 --------
  if (PataDesviada[pata2]) {
      VelocidadMovimiento[pata2] = 100;
      PARABOLICO[pata2] = 1;
      Punta_Pata_RespectoCentro_GUARDADO[pata2].x = POSICION_A_LA_QUE_POSICIONARSE[pata2].x;
      Punta_Pata_RespectoCentro_GUARDADO[pata2].y = POSICION_A_LA_QUE_POSICIONARSE[pata2].y;  
      algunaDesviada = true;
  }

  // -------- Si alguna estaba desviada, esperar a que terminen --------
  if (algunaDesviada) {
      if (!BLOQUEO[1]) {TON[25].set(100); BLOQUEO[1] = true;}

      if (TON[25].done() && patasTerminadas[pata1] && patasTerminadas[pata2]) {
          BLOQUEO[1] = false;
          idx += 2;  // avanzar al siguiente par
          TON[25].reset();
          if (idx >= 6) {idx = 0; EstadoPOSICIONAMIENTO = 3;}
      }
  }
  else {
      // Ninguna de las dos patas desviada → saltar al siguiente par directamente
      idx += 2;
      if (idx >= 6) {idx = 0; EstadoPOSICIONAMIENTO = 3;}
  }
}
  
  if (EstadoPOSICIONAMIENTO == 3) { // ============== ULTIMO ACOSTAR SI LA POSICION DESEADA ES MAS BAJA QUE LA ACTUAL  ========================
    
    if ((PataDiferenciaAltura[1] || PataDiferenciaAltura[2] || PataDiferenciaAltura[3] || PataDiferenciaAltura[4] || PataDiferenciaAltura[5] || PataDiferenciaAltura[6]) && (
    Diferencia_Posicion[1].z < 0 || Diferencia_Posicion[2].z < 0  || Diferencia_Posicion[3].z < 0 || Diferencia_Posicion[4].z < 0 || Diferencia_Posicion[5].z < 0 || Diferencia_Posicion[6].z < 0))
    {
    for (int i = 1; i <= 6; i++){
    if ((Diferencia_Posicion[i].z < 0) && (BLOQUEO[3] == false)) {
    
    Punta_Pata_RespectoCentro_GUARDADO[i].x = Punta_Pata_RespectoCentro_GUARDADO[i].x;
    Punta_Pata_RespectoCentro_GUARDADO[i].y = Punta_Pata_RespectoCentro_GUARDADO[i].y;
    Punta_Pata_RespectoCentro_GUARDADO[i].z = Punta_Pata_RespectoCentro_GUARDADO[i].z + Diferencia_Posicion[i].z;
    VelocidadMovimiento[i] = 30; PARABOLICO[i] = 0; }
    }

    TON[25].set(100);
    BLOQUEO[3] = true;

    if (TON[25].done() && patasTerminadas[1] && patasTerminadas[2] && patasTerminadas[3] && patasTerminadas[4] && patasTerminadas[5] && patasTerminadas[6])
    {EstadoPOSICIONAMIENTO = 4; TON[25].reset(); BLOQUEO[3] = false;
}

  } else {EstadoPOSICIONAMIENTO = 4;}
  }

  if (EstadoPOSICIONAMIENTO == 4)  // ================================== FORZAR PATAS A POSICION  ==============================================
{
  A.CONTROL_POSICION = A.ACCION_POSICIONARSE; 
  A.ACCION_POSICIONARSE = 0;

}


} else {EstadoPOSICIONAMIENTO = 0;TON[25].reset();}






  

// ---------- PROCESADO y INTERPOLACION DE LA INCLINACION DEL ROBOT ----------
{
  // Helpers locales
  auto mapf = [](float v, float in_min, float in_max, float out_min, float out_max) {
    return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  };
  auto clampf = [](float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  };
  auto stepAxis = [](float cur, float target, float vmax_deg_s, float dt_s) {
    float d = target - cur;
    float s = vmax_deg_s * dt_s;
    if (d >  s) return cur + s;
    if (d < -s) return cur - s;
    return target;
  };

  // 1) Joystick normalizado a [-1,1] (stick derecho)
  float x = mapf((float)JdxVal, -128.0f, 127.0f, -1.0f, 1.0f);
  float y = mapf((float)JdyVal, -128.0f, 127.0f, -1.0f, 1.0f);

  // Deadzone pequeña
  const float DZ = 0.06f;
  auto dzProc = [DZ](float a) {
    if (fabsf(a) < DZ) return 0.0f;
    return (a > 0.0f) ? (a - DZ) / (1.0f - DZ) : (a + DZ) / (1.0f - DZ);
  };
  x = dzProc(x);
  y = dzProc(y);

  // 2) Escala a grados (yaw/pitch con stick; roll con L2/R2)
  const float YAW_MAX   = 20.0f;
  const float PITCH_MAX = 20.0f;
  const float ROLL_MAX  = 20.0f;

  const float yaw_target   = x * YAW_MAX;
  const float pitch_target = -y * PITCH_MAX; // invierte a +y si lo prefieres
  const float roll_target  = mapf((float)((int)R2Val - (int)L2Val), -255.0f, 255.0f, -ROLL_MAX, ROLL_MAX);

  // 3) Interpolación (rampa a velocidad constante)
  static bool init = false;
  static float yaw_f = 0.0f, pitch_f = 0.0f, roll_f = 0.0f;
  static unsigned long t_prev = 0;
  const float vmax = 30.0f; // deg/s

  const unsigned long now = millis();
  float dt = init ? (now - t_prev) / 1000.0f : 0.0f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f)  dt = 0.05f;
  t_prev = now;

  if (!init) { yaw_f = yaw_target; pitch_f = pitch_target; roll_f = roll_target; init = true; }

  yaw_f   = stepAxis(yaw_f,   yaw_target,   vmax, dt);
  pitch_f = stepAxis(pitch_f, pitch_target, vmax, dt);
  roll_f  = stepAxis(roll_f,  roll_target,  vmax, dt);

  // 4) Límites y salida
  yaw_f   = clampf(yaw_f,   -YAW_MAX,   YAW_MAX);
  pitch_f = clampf(pitch_f, -PITCH_MAX, PITCH_MAX);
  roll_f  = clampf(roll_f,  -ROLL_MAX,  ROLL_MAX);

  POS_CALCULADO_CENTRO_ROBOT.yaw   = yaw_f;
  POS_CALCULADO_CENTRO_ROBOT.pitch = pitch_f;
  POS_CALCULADO_CENTRO_ROBOT.roll  = roll_f;
}


// DETECCION + INTERPOLACION SIMPLE
if (EstadoMoverseCANGREJO != 3) {
//====================================== DETECCION DE DEMANDA DE MOVIMIENTO (por pata)=================================

for (int i = 1; i <= 6; i++) {
  // si el robot está acostado o calibrando, no hay demanda
  if (C.ESTADO_ACOSTADO || C.ESTADO_CALIBRANDO) {
    demandaMovimiento[i] = false;
    patasTerminadas[i]   = true;
    continue;
  }

  const float dx = (Punta_Pata_RespectoCentro_GUARDADO[i].x - Punta_Pata_RespectoCentro_ACTUAL[i].x);
  const float dy = (Punta_Pata_RespectoCentro_GUARDADO[i].y - Punta_Pata_RespectoCentro_ACTUAL[i].y);
  const float dz = (Punta_Pata_RespectoCentro_GUARDADO[i].z - Punta_Pata_RespectoCentro_ACTUAL[i].z);

  // hay demanda si alguna diferencia supera EPS
  const bool hayDemanda = (fabs(dx) > 0.5f) || (fabs(dy) > 0.5f) || (fabs(dz) > 0.5f);

  demandaMovimiento[i] = hayDemanda;
  patasTerminadas[i]   = !hayDemanda;   // “ha llegado” si NO hay demanda

  if (hayDemanda) {
  deltaX[i] = dx;
  deltaY[i] = dy;
  deltaZ[i] = dz;
} else {
  deltaX[i] = deltaY[i] = deltaZ[i] = 0.0f;
}
}







//====================================== DETECCION DE CAMBIO DE OBJETIVO (por pata)=================================

// Inicializamos con el objetivo actual la primera vez (evita falsos positivos)
if (!PrimerCICLO) {
  for (int i = 1; i <= 6; i++) {
    objPrevX[i] = Punta_Pata_RespectoCentro_GUARDADO[i].x;
    objPrevY[i] = Punta_Pata_RespectoCentro_GUARDADO[i].y;
    objPrevZ[i] = Punta_Pata_RespectoCentro_GUARDADO[i].z;
  }
  PrimerCICLO = true;
}

// Comparamos objetivo actual vs. objetivo anterior
for (int i = 1; i <= 6; i++) {
  float dx = Punta_Pata_RespectoCentro_GUARDADO[i].x - objPrevX[i];
  float dy = Punta_Pata_RespectoCentro_GUARDADO[i].y - objPrevY[i];
  float dz = Punta_Pata_RespectoCentro_GUARDADO[i].z - objPrevZ[i];

  // Hay cambio si cualquiera supera el umbral
  cambioObjetivo[i] = ((fabsf(dx) > 0.3f) || (fabsf(dy) > 0.3f) || (fabsf(dz) > 0.3f)) && (demandaMovimiento[i] == 1);

  // Actualizamos “previo” para el próximo ciclo
  objPrevX[i] = Punta_Pata_RespectoCentro_GUARDADO[i].x;
  objPrevY[i] = Punta_Pata_RespectoCentro_GUARDADO[i].y;
  objPrevZ[i] = Punta_Pata_RespectoCentro_GUARDADO[i].z;
}






//====================================== INTERPOLACION DE POSICIONES (por pata)  — VELOCIDAD CONSTANTE ===============================
{
  // NOTA: desde ahora, VelocidadMovimiento[i] = velocidad lineal en mm/s (no ms).
  // Puedes limitar aquí la velocidad para evitar burradas:
  const float V_MIN = 5.0f;    // mm/s
  const float V_MAX = 200.0f;  // mm/s

  static float  sParam[7]         = {};   // progreso 0..1 por pata
  static float  invDist[7]        = {};   // 1/distancia del segmento actual
  static unsigned long tPrev[7]   = {};   // timestamp por pata
  static bool   rampaActiva[7]    = {false};

  for (int i = 1; i <= 6; i++) {

    // si está acostado o calibrando, no rampas
    if (C.ESTADO_ACOSTADO || C.ESTADO_CALIBRANDO) {
      rampaActiva[i] = false;
      Punta_Pata_RespectoCentro_INTERP[i] = Punta_Pata_RespectoCentro_ACTUAL[i];
      patasTerminadas[i] = true;
      continue;
    }

    // (A) ¿hay que iniciar/reiniciar rampa (objetivo nuevo)?
    if (cambioObjetivo[i] || (demandaMovimiento[i] && !rampaActiva[i])) {
      // arranque de tramo
      p0x[i] = Punta_Pata_RespectoCentro_ACTUAL[i].x;
      p0y[i] = Punta_Pata_RespectoCentro_ACTUAL[i].y;
      p0z[i] = Punta_Pata_RespectoCentro_ACTUAL[i].z;

      p1x[i] = Punta_Pata_RespectoCentro_GUARDADO[i].x;
      p1y[i] = Punta_Pata_RespectoCentro_GUARDADO[i].y;
      p1z[i] = Punta_Pata_RespectoCentro_GUARDADO[i].z;

      const float dx = p1x[i] - p0x[i];
      const float dy = p1y[i] - p0y[i];
      const float dz = p1z[i] - p0z[i];
      const float dist = sqrtf(dx*dx + dy*dy + dz*dz);

      if (dist < 1e-3f) {
        // tramo despreciable
        rampaActiva[i] = false;
        sParam[i] = 1.0f;
        Punta_Pata_RespectoCentro_INTERP[i] = Punta_Pata_RespectoCentro_ACTUAL[i];
        patasTerminadas[i] = true;
        continue;
      }

      invDist[i]   = 1.0f / dist;
      sParam[i]    = 0.0f;
      tPrev[i]     = millis();
      rampaActiva[i] = true;
    }

    // (B) avanzar rampa a velocidad constante
    if (rampaActiva[i]) {
      unsigned long now = millis();
      float dt = (now - tPrev[i]) / 1000.0f;
      if (dt < 0.0005f) dt = 0.0005f;     // mínimo para estabilidad
      if (dt > 0.05f)   dt = 0.05f;       // máximo por seguridad
      tPrev[i] = now;

      // velocidad deseada (mm/s), limitada
      float v = VelocidadMovimiento[i];
      if (v < V_MIN) v = V_MIN;
      if (v > V_MAX) v = V_MAX;

      // avanzar s en proporción a v y distancia del tramo
      sParam[i] += (v * dt) * invDist[i];
      if (sParam[i] >= 1.0f) { sParam[i] = 1.0f; rampaActiva[i] = false; }

      // base lineal
      float x = p0x[i] + sParam[i] * (p1x[i] - p0x[i]);
      float y = p0y[i] + sParam[i] * (p1y[i] - p0y[i]);
      float z = p0z[i] + sParam[i] * (p1z[i] - p0z[i]);

      // joroba parabólica (swing)
      if (PARABOLICO[i]) {
        float dxy = hypotf(p1x[i] - p0x[i], p1y[i] - p0y[i]);
        float H   = fminf(40.0f, 0.8f * dxy);           // altura de paso
        float lift = H * (4.0f * sParam[i] * (1.0f - sParam[i]));  // 0→H→0
        z -= lift; // (mantengo tu convenio de signos)
      }

      Punta_Pata_RespectoCentro_INTERP[i].x = x;
      Punta_Pata_RespectoCentro_INTERP[i].y = y;
      Punta_Pata_RespectoCentro_INTERP[i].z = z;

      patasTerminadas[i] = !rampaActiva[i];
    }
    else {
      // sin rampa → ya llegó o no hay demanda
      Punta_Pata_RespectoCentro_INTERP[i] = demandaMovimiento[i]
        ? Punta_Pata_RespectoCentro_GUARDADO[i]
        : Punta_Pata_RespectoCentro_ACTUAL[i];
      patasTerminadas[i] = !demandaMovimiento[i];
    }
  }

  // primera inicialización (por seguridad)
  static bool firstInterpInit = true;
  if (firstInterpInit) {
    for (int i = 1; i <= 6; i++) {
      Punta_Pata_RespectoCentro_INTERP[i] = Punta_Pata_RespectoCentro_ACTUAL[i];
    }
    firstInterpInit = false;
  }
}

} // FIN DETECCION + INTERPOLACION SIMPLE



// ====================================== APLICAR INCLINACION AL POSICIONAMIENTO DE PATAS ===============================
if (true) { 

    // ============ CÁLCULO SENOS Y COSENOS (grados -> rad dentro de sinf/cosf) ===================
  float SEN_YAW   = sinf(POS_CALCULADO_CENTRO_ROBOT.yaw   * (M_PI / 180.0f));
  float COS_YAW   = cosf(POS_CALCULADO_CENTRO_ROBOT.yaw   * (M_PI / 180.0f));
  float SEN_PITCH = sinf(POS_CALCULADO_CENTRO_ROBOT.pitch * (M_PI / 180.0f));
  float COS_PITCH = cosf(POS_CALCULADO_CENTRO_ROBOT.pitch * (M_PI / 180.0f));
  float SEN_ROLL  = sinf(POS_CALCULADO_CENTRO_ROBOT.roll  * (M_PI / 180.0f));
  float COS_ROLL  = cosf(POS_CALCULADO_CENTRO_ROBOT.roll  * (M_PI / 180.0f));

  if (!C.ESTADO_ACOSTADO && !C.ESTADO_CALIBRANDO) {
    for (int i = 1; i <= 6; i++) {

      float dx1 =  Punta_Pata_RespectoCentro_INTERP[i].x * COS_ROLL + Punta_Pata_RespectoCentro_INTERP[i].z * SEN_ROLL;
      float dy1 =  Punta_Pata_RespectoCentro_INTERP[i].y;
      float dz1 = -Punta_Pata_RespectoCentro_INTERP[i].x * SEN_ROLL + Punta_Pata_RespectoCentro_INTERP[i].z * COS_ROLL;

      float dx2 =  dx1;
      float dy2 =  dy1 * COS_PITCH - dz1 * SEN_PITCH;
      float dz2 =  dy1 * SEN_PITCH + dz1 * COS_PITCH;

      float dx3 =  dx2 * COS_YAW - dy2 * SEN_YAW;
      float dy3 =  dx2 * SEN_YAW + dy2 * COS_YAW;
      float dz3 =  dz2;

      Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[i].x = dx3;
      Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[i].y = dy3;
      Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[i].z = dz3;

    }
  }
}



// ======================================================================CINEMATICA INVERSA Y ENVIO A SERVOS================================================================
{
  if (C.ESTADO_ACOSTADO == true) {
    if (!C.ESTADO_CALIBRANDO) {servos.disable_all();}
  } else {


   for (int NPATA = 1; NPATA <= 6; NPATA++) {

  //CALCULAR POSICION LOCAL DE CADA PATA A PARTIR DEL LA COORDENADA GLOBAL ACTUAL, LA INCLINACION Y EL CENTRO DEL ROBOT
  const float INVERTIDO = (NPATA==2 || NPATA==4 || NPATA==6) ? -1.0f : 1.0f;
  float x = INVERTIDO * (Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[NPATA].x - CONST_POSICNES_ORIGEN_PATA[NPATA].x);
  float y =             (Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[NPATA].y - CONST_POSICNES_ORIGEN_PATA[NPATA].y);
  float z =             (Punta_Pata_RespectoCentro_INTERP_MAS_INCLINACION[NPATA].z - CONST_POSICNES_ORIGEN_PATA[NPATA].z);

  XYZfloat Posicion0 = {0.0f, 0.0f, 0.0f};
  XYZfloat PuntaCoxa;
  PuntaCoxa.x = Posicion0.x + Lcoxa;
  PuntaCoxa.y = Posicion0.y;
  PuntaCoxa.z = Posicion0.z;

  // ======== TRIÁNGULO ALFA 1 (azul) ========
  ABC TrianguloAlfa1;
  TrianguloAlfa1.lonV_A_B = z;                          // AB (vertical)
  TrianguloAlfa1.lonV_B_C = (sqrt(x*x + y*y)) - PuntaCoxa.x;                  // BC (horizontal)
  TrianguloAlfa1.lonV_C_A = sqrt(pow(TrianguloAlfa1.lonV_A_B, 2) + pow(TrianguloAlfa1.lonV_B_C, 2));  // AC (hipotenusa)
  TrianguloAlfa1.ang_C = atan2(TrianguloAlfa1.lonV_A_B, TrianguloAlfa1.lonV_B_C) * 180.0f / M_PI;  // ángulo en A (coxa)
  TrianguloAlfa1.ang_B = 90.0f;   
  TrianguloAlfa1.ang_A = 90.0f - TrianguloAlfa1.ang_C;  // ✅ corregido

  // ======== TRIÁNGULO ALFA 2 (verde) =========
  ABC TrianguloAlfa2;
  TrianguloAlfa2.lonV_A_B = Lfemur;                         // lado b
  TrianguloAlfa2.lonV_B_C = Ltibia;                         // lado a
  TrianguloAlfa2.lonV_C_A = TrianguloAlfa1.lonV_C_A;        // lado c (AC)
  float a = TrianguloAlfa2.lonV_B_C;  // tibia
  float b = TrianguloAlfa2.lonV_A_B;  // fémur
  float c = TrianguloAlfa2.lonV_C_A;  // línea AC (objetivo)

  TrianguloAlfa2.ang_A = acos((b * b + c * c - a * a) / (2 * b * c)) * 180.0f / M_PI;
  TrianguloAlfa2.ang_C = acos((a * a + c * c - b * b) / (2 * a * c)) * 180.0f / M_PI;
  TrianguloAlfa2.ang_B = 180.0f - TrianguloAlfa2.ang_A - TrianguloAlfa2.ang_C;

  angCoxa[NPATA] = atan2(y, x) * 180.0f / M_PI; // ángulo de la coxa
  angFemur[NPATA] = 270.0f - (TrianguloAlfa1.ang_A + TrianguloAlfa2.ang_A) ;  // ángulo del femur
  angTibia[NPATA] = TrianguloAlfa2.ang_B;  // ángulo de la tibia

 }


  //=================================================================   PATA 1   =================================
  angCoxaCalibrado[1] = angCoxa[1] - 0.0f - (45.0f - CalServo[0]); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[1] = angFemur[1] - (65.986f - CalServo[6]); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> -80º
  angTibiaCalibrado[1] = angTibia[1] - (19.013f - CalServo[12]); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> -75º

  //=================================================================   PATA 2   =================================

  angCoxaCalibrado[2] = -(angCoxa[2] - 0.0f - (45.0f + CalServo[1])); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[2] = -(angFemur[2] - (65.986f + CalServo[7])); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> 80º
  angTibiaCalibrado[2] = -(angTibia[2] - (19.013f + CalServo[13])); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> 75º


  //=================================================================   PATA 3   =================================

  angCoxaCalibrado[3] = angCoxa[3] - 0.0f - (0.0f - CalServo[2]); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[3] = angFemur[3] - (65.986f - CalServo[8]); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> -80º
  angTibiaCalibrado[3] = angTibia[3] - (19.013f - CalServo[14]); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> -75º


  //=================================================================   PATA 4   =================================

  angCoxaCalibrado[4] = -(angCoxa[4] - 0.0f - (0.0f + CalServo[3])); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[4] = -(angFemur[4] - (65.986f + CalServo[9])); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> 80º
  angTibiaCalibrado[4] = -(angTibia[4] - (19.013f + CalServo[15])); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> 75º

 
  //=================================================================   PATA 5   =================================
  
  angCoxaCalibrado[5] = angCoxa[5] - 0.0f - (-45.0f - CalServo[4]); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[5] = -(angFemur[5] - (65.986f + CalServo[10])); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> 80º
  angTibiaCalibrado[5] = -(angTibia[5] - (19.013f + CalServo[16])); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> 75º
  

  //=================================================================   PATA 6   =================================

  angCoxaCalibrado[6] = - (angCoxa[6] - 0.0f - (-45.0f + CalServo[5])); // COXA CALIBRADO
  //en reposo pata[1].angCoxa = 0 --->
  angFemurCalibrado[6] = angFemur[6] - (65.986f - CalServo[11]); // FEMUR CALIBRADO
  //en reposo pata[2].angCoxa = 65,986º ---> -80º
  angTibiaCalibrado[6] = angTibia[6] - (19.013f - CalServo[17]); // TIBIA CALIBRADO
  //en reposo pata[3].angCoxa = 19,013º ---> -75º



  servos.value(0,angCoxaCalibrado[1]); // Movimiento servo COXA
  servos.value(6,angFemurCalibrado[1] <= CalServo[6] ? CalServo[6] : angFemurCalibrado[1]); // Movimiento servo FEMUR
  servos.value(12,angTibiaCalibrado[1] <= CalServo[12] ? CalServo[12] : angTibiaCalibrado[1]); // Movimiento servo TIBIA

  servos.value(1,angCoxaCalibrado[2]); // Movimiento servo COXA
  servos.value(7,angFemurCalibrado[2] >= CalServo[7] ? CalServo[7] : angFemurCalibrado[2]); // Movimiento servo FEMUR
  servos.value(13,angTibiaCalibrado[2] >= CalServo[13] ? CalServo[13] : angTibiaCalibrado[2]); // Movimiento servo TIBIA

  servos.value(2,angCoxaCalibrado[3]); // Movimiento servo COXA
  servos.value(8,angFemurCalibrado[3] <= CalServo[8] ? CalServo[8] : angFemurCalibrado[3]); // Movimiento servo FEMUR
  servos.value(14,angTibiaCalibrado[3] <= CalServo[14] ? CalServo[14] : angTibiaCalibrado[3]); // Movimiento servo TIBIA

  servos.value(3,angCoxaCalibrado[4]); // Movimiento servo COXA
  servos.value(9,angFemurCalibrado[4] >= CalServo[9] ? CalServo[9] : angFemurCalibrado[4]); // Movimiento servo FEMUR
  servos.value(15,angTibiaCalibrado[4] >= CalServo[15] ? CalServo[15] : angTibiaCalibrado[4]); // Movimiento servo TIBIA

  servos.value(4,angCoxaCalibrado[5]); // Movimiento servo COXA
  servos.value(10,angFemurCalibrado[5] >= CalServo[10] ? CalServo[10] : angFemurCalibrado[5]); // Movimiento servo FEMUR
  servos.value(16,angTibiaCalibrado[5] >= CalServo[16] ? CalServo[16] : angTibiaCalibrado[5]); // Movimiento servo TIBIA

  servos.value(5,angCoxaCalibrado[6]); // Movimiento servo COXA
  servos.value(11,angFemurCalibrado[6] <= CalServo[11] ? CalServo[11] : angFemurCalibrado[6]); // Movimiento servo FEMUR
  servos.value(17,angTibiaCalibrado[6] <= CalServo[17] ? CalServo[17] : angTibiaCalibrado[6]); // Movimiento servo TIBIA

 servos.load();
 // FINAL MOVIMIENTO
}
}



  for (int i = 1; i <= 6; ++i) {
    Punta_Pata_RespectoCentro_ACTUAL[i]  = Punta_Pata_RespectoCentro_INTERP[i];
  }

  BLOQUEAR = 1;


 // DEBUGGING  (opcional)
{
    // printf("\033[2J\033[H");  // Limpia pantalla y mueve el cursor arriba a la izquierda
    // printf("\n=== ESTADO DE COMANDOS ===\n");
    // Solicitudes
    // printf("SOLICITUD_NINGUNA: %d\n", C.SOLICITUD_NINGUNA);
    // printf("SOLICITUD_LEVANTAR: %d\n", C.SOLICITUD_LEVANTAR);
    // printf("SOLICITUD_CALIBRAR: %d\n", C.SOLICITUD_CALIBRAR);
    // printf("SOLICITUD_ACOSTAR: %d\n", C.SOLICITUD_ACOSTAR);
    // printf("SOLICITUD_PARARSE: %d\n", C.SOLICITUD_PARARSE);
    // printf("SOLICITUD_MOVERSE_ESTATICO: %d\n", C.SOLICITUD_MOVERSE_ESTATICO);
    // printf("SOLICITUD_MOVERSE_CANGREJO: %d\n", C.SOLICITUD_MOVERSE_CANGREJO);
    // printf("SOLICITUD_MOVERSE_NORMAL: %d\n", C.SOLICITUD_MOVERSE_NORMAL);
    // printf("SOLICITUD_DESCANSAR: %d\n", C.SOLICITUD_DESCANSAR);

    // // Estado
    // printf("ESTADO_LEVANTADO: %d\n", C.ESTADO_LEVANTADO);
    // printf("ESTADO_ACOSTADO: %d\n", C.ESTADO_ACOSTADO);
    // printf("ESTADO_DESCANSADO: %d\n", C.ESTADO_DESCANSADO);
    // printf("ESTADO_ESTATICO: %d\n", C.ESTADO_ESTATICO);
    // printf("ESTADO_CANGREJO: %d\n", C.ESTADO_CANGREJO);
    // printf("ESTADO_NORMAL: %d\n", C.ESTADO_NORMAL);
    // printf("ESTADO_PARADO: %d\n", C.ESTADO_PARADO);
    // printf("ESTADO_CALIBRANDO: %d\n", C.ESTADO_CALIBRANDO);
    // printf("ESTADO_NINGUNA: %d\n", C.ESTADO_NINGUNA);

    // // Acciones
    // printf("ACCION_NINGUNA: %d\n", A.ACCION_MOVERSE_NORMAL);
    // printf("ACCION_CALIBRAR: %d\n", A.ACCION_CALIBRAR);
    // printf("ACCION_LEVANTARSE: %d\n", A.ACCION_LEVANTARSE);
    // printf("ACCION_ACOSTARSE: %d\n", A.ACCION_ACOSTARSE);
    // printf("ACCION_DESCANSARSE: %d\n", A.ACCION_DESCANSARSE);
    // printf("ACCION_MOVERSE_ESTATICO: %d\n", A.ACCION_MOVERSE_ESTATICO);
    // printf("ACCION_MOVERSE_CANGREJO: %d\n", A.ACCION_MOVERSE_CANGREJO);
    // printf("ACCION_MOVERSE_NORMAL: %d\n", A.ACCION_MOVERSE_NORMAL);
    // printf("ACCION_POSICIONARSE: %d\n", A.ACCION_POSICIONARSE);
    // printf("CONTROL_POSICION: %d\n", A.CONTROL_POSICION);
    // printf("POSICION_PARAR: %d\n", A.ACCION_PARAR);

    // // Modo
    // printf("NORMAL: %d\n", C.NORMAL);
    // printf("CALIBRACION: %d\n", C.CALIBRACION);
    // printf("__\n");
    // printf("EstadoPOSICIONAMIENTO: %d\n", EstadoPOSICIONAMIENTO);
    // printf("VELOCIDAD: %f\n", VelocidadMovimiento[1]);


    // printf ("POS ACTUAL X: %f\n", Punta_Pata_Pos_Act[1].x);
    // printf ("POS ACTUAL Y: %f\n", Punta_Pata_Pos_Act[1].y);
    // printf ("POS ACTUAL Z: %f\n", Punta_Pata_Pos_Act[1].z);

    // printf ("POS ACTUAL DESDE CENTRO X: %f\n", Punta_Pata_RespectoCentro_ACTUAL[1].x);
    // printf ("POS ACTUAL DESDE CENTRO Y: %f\n", Punta_Pata_RespectoCentro_ACTUAL[1].y);
    // printf ("POS ACTUAL DESDE CENTRO Z: %f\n", Punta_Pata_RespectoCentro_ACTUAL[1].z);

    // printf ("POS GUARDADA DESDE CENTRO X: %f\n", Punta_Pata_RespectoCentro_GUARDADO[1].x);
    // printf ("POS GUARDADA DESDE CENTRO Y: %f\n", Punta_Pata_RespectoCentro_GUARDADO[1].y);
    // printf ("POS GUARDADA DESDE CENTRO Z: %f\n", Punta_Pata_RespectoCentro_GUARDADO[1].z);

    // printf ("POS GUARDADA DESDE CENTRO POS EXTENDIDA X: %f\n", Punta_Pata_RespectoCentro_extendida[1].x);
    // printf ("POS GUARDADA DESDE CENTRO POS EXTENDIDA Y: %f\n", Punta_Pata_RespectoCentro_extendida[1].y);
    // printf ("POS GUARDADA DESDE CENTRO POS EXTENDIDA Z: %f\n", Punta_Pata_RespectoCentro_extendida[1].z);

    // printf ("POS GUARDADA DESDE CENTRO POS CIRCULAR X: %f\n", Punta_Pata_RespectoCentro_circular[1].x);
    // printf ("POS GUARDADA DESDE CENTRO POS CIRCULAR Y: %f\n", Punta_Pata_RespectoCentro_circular[1].y);
    // printf ("POS GUARDADA DESDE CENTRO POS CIRCULAR Z: %f\n", Punta_Pata_RespectoCentro_circular[1].z);

    // printf ("PATA 1 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[1].x,Punta_Pata_RespectoCentro_INTERP[1].y,Punta_Pata_RespectoCentro_INTERP[1].z);
    // printf ("PATA 2 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[2].x,Punta_Pata_RespectoCentro_INTERP[2].y,Punta_Pata_RespectoCentro_INTERP[2].z);
    // printf ("PATA 3 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[3].x,Punta_Pata_RespectoCentro_INTERP[3].y,Punta_Pata_RespectoCentro_INTERP[3].z);
    // printf ("PATA 4 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[4].x,Punta_Pata_RespectoCentro_INTERP[4].y,Punta_Pata_RespectoCentro_INTERP[4].z);
    // printf ("PATA 5 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[5].x,Punta_Pata_RespectoCentro_INTERP[5].y,Punta_Pata_RespectoCentro_INTERP[5].z);
    // printf ("PATA 6 X: %f  Y: %f Z: %f\n", Punta_Pata_RespectoCentro_INTERP[6].x,Punta_Pata_RespectoCentro_INTERP[6].y,Punta_Pata_RespectoCentro_INTERP[6].z);

    // printf ("PATA 1 DISTANCIA: %f\n", DistanciaP1P2[1]);
    // printf ("PATA 2 DISTANCIA: %f\n", DistanciaP1P2[2]);
    // printf ("PATA 3 DISTANCIA: %f\n", DistanciaP1P2[3]);
    // printf ("PATA 4 DISTANCIA: %f\n", DistanciaP1P2[4]);
    // printf ("PATA 5 DISTANCIA: %f\n", DistanciaP1P2[5]);
    // printf ("PATA 6 DISTANCIA: %f\n", DistanciaP1P2[6]);    
    
    // printf("PATA HA TERMINADO 1 %d\n", patasTerminadas[1]);
    // printf("PATA HA TERMINADO 2 %d\n", patasTerminadas[2]);
    // printf("PATA HA TERMINADO 3 %d\n", patasTerminadas[3]);
    // printf("PATA HA TERMINADO 4 %d\n", patasTerminadas[4]);
    // printf("PATA HA TERMINADO 5 %d\n", patasTerminadas[5]);
    // printf("PATA HA TERMINADO 6 %d\n", patasTerminadas[6]);
  
    // printf ("PITCH MPU X: %f\n", pitchValMPU);
    // printf ("ROLL MPU Y: %f\n", rollValMPU);
  //  
 

}


}

// =================== RECIBIR DESDE UART ===================
void recibirYprocesarDesdeUART() {

  // Mantener entre llamadas
  static std::string buffer = "";
  static std::string RecivoBuffer = "";

  while (uart_is_readable(uart1)) {

    // PROCESADOR DE LÍNEAS
{
    char c = uart_getc(uart1);
    if (c == '\r') continue;

    if (c != '\n') {
      if (buffer.size() < 127) buffer.push_back(c);
      else buffer.clear();  // overflow -> descartamos línea corrupta
      continue;
    }

    // c == '\n' -> tenemos una línea completa
    RecivoBuffer = buffer;
    //printf("%s\n", RecivoBuffer.c_str());
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
         if (Comparar(RecivoBuffer, "JIX"))  { ExtraerINT(RecivoBuffer,   JixVal); }
    else if (Comparar(RecivoBuffer, "JIY"))  { ExtraerINT(RecivoBuffer,   JiyVal); }
    else if (Comparar(RecivoBuffer, "JDX"))  { ExtraerINT(RecivoBuffer,   JdxVal); }
    else if (Comparar(RecivoBuffer, "JDY"))  { ExtraerINT(RecivoBuffer,   JdyVal); }
    else if (Comparar(RecivoBuffer, "L2V"))  { ExtraerINT(RecivoBuffer,   L2Val); }
    else if (Comparar(RecivoBuffer, "R2V"))  { ExtraerINT(RecivoBuffer,   R2Val); }
    else if (Comparar(RecivoBuffer, "PIT"))  { ExtraerFLOAT(RecivoBuffer, pitchValMPU); }   // <-- FLOAT
    else if (Comparar(RecivoBuffer, "ROL"))  { ExtraerFLOAT(RecivoBuffer, rollValMPU); }     // <-- FLOAT
    else if (Comparar(RecivoBuffer, "ON"))   { ExtraerINT(RecivoBuffer,   mandoConectado); }
    else if (Comparar(RecivoBuffer, "UP"))   { ExtraerINT(RecivoBuffer,   UP); }
    else if (Comparar(RecivoBuffer, "DN"))   { ExtraerINT(RecivoBuffer,   DOWN); }
    else if (Comparar(RecivoBuffer, "LF"))   { ExtraerINT(RecivoBuffer,   LEFT); }
    else if (Comparar(RecivoBuffer, "RT"))   { ExtraerINT(RecivoBuffer,   RIGHT); }
    else if (Comparar(RecivoBuffer, "SQ"))   { ExtraerINT(RecivoBuffer,   SQUARE); }    
    else if (Comparar(RecivoBuffer, "CR"))   { ExtraerINT(RecivoBuffer,   CROSS); }
    else if (Comparar(RecivoBuffer, "CI"))   { ExtraerINT(RecivoBuffer,   CIRCLE); }
    else if (Comparar(RecivoBuffer, "TR"))   { ExtraerINT(RecivoBuffer,   TRIANGLE); }
    else if (Comparar(RecivoBuffer, "L1"))   { ExtraerINT(RecivoBuffer,   L1); }
    else if (Comparar(RecivoBuffer, "L2"))   { ExtraerINT(RecivoBuffer,   L2); }
    else if (Comparar(RecivoBuffer, "R1"))   { ExtraerINT(RecivoBuffer,   R1); }
    else if (Comparar(RecivoBuffer, "R2"))   { ExtraerINT(RecivoBuffer,   R2); }
    else if (Comparar(RecivoBuffer, "L3"))   { ExtraerINT(RecivoBuffer,   L3); }
    else if (Comparar(RecivoBuffer, "R3"))   { ExtraerINT(RecivoBuffer,   R3); }
    else if (Comparar(RecivoBuffer, "SH"))   { ExtraerINT(RecivoBuffer,   SHARE); }
    else if (Comparar(RecivoBuffer, "OP"))   { ExtraerINT(RecivoBuffer,   OPTIONS); }
    else if (Comparar(RecivoBuffer, "PS"))   { ExtraerINT(RecivoBuffer,   PSButton); }
    else if (Comparar(RecivoBuffer, "TP"))   { ExtraerINT(RecivoBuffer,   Touchpad); }
    else if (Comparar(RecivoBuffer, "CALI")) { ExtraerINT(RecivoBuffer,   CALIBRANDO); }

    // Calibraciones: usa FLOAT (aunque te lleguen enteros, es más seguro)
    else if (Comparar(RecivoBuffer, "EEPROM0"))  { ExtraerFLOAT(RecivoBuffer, CalServo[0]);  }
    else if (Comparar(RecivoBuffer, "EEPROM1"))  { ExtraerFLOAT(RecivoBuffer, CalServo[1]);  }
    else if (Comparar(RecivoBuffer, "EEPROM2"))  { ExtraerFLOAT(RecivoBuffer, CalServo[2]);  }
    else if (Comparar(RecivoBuffer, "EEPROM3"))  { ExtraerFLOAT(RecivoBuffer, CalServo[3]);  }
    else if (Comparar(RecivoBuffer, "EEPROM4"))  { ExtraerFLOAT(RecivoBuffer, CalServo[4]);  }
    else if (Comparar(RecivoBuffer, "EEPROM5"))  { ExtraerFLOAT(RecivoBuffer, CalServo[5]);  }
    else if (Comparar(RecivoBuffer, "EEPROM6"))  { ExtraerFLOAT(RecivoBuffer, CalServo[6]);  }
    else if (Comparar(RecivoBuffer, "EEPROM7"))  { ExtraerFLOAT(RecivoBuffer, CalServo[7]);  }
    else if (Comparar(RecivoBuffer, "EEPROM8"))  { ExtraerFLOAT(RecivoBuffer, CalServo[8]);  }
    else if (Comparar(RecivoBuffer, "EEPROM9"))  { ExtraerFLOAT(RecivoBuffer, CalServo[9]);  }
    else if (Comparar(RecivoBuffer, "EEPROM10")) { ExtraerFLOAT(RecivoBuffer, CalServo[10]); }
    else if (Comparar(RecivoBuffer, "EEPROM11")) { ExtraerFLOAT(RecivoBuffer, CalServo[11]); }
    else if (Comparar(RecivoBuffer, "EEPROM12")) { ExtraerFLOAT(RecivoBuffer, CalServo[12]); }
    else if (Comparar(RecivoBuffer, "EEPROM13")) { ExtraerFLOAT(RecivoBuffer, CalServo[13]); }
    else if (Comparar(RecivoBuffer, "EEPROM14")) { ExtraerFLOAT(RecivoBuffer, CalServo[14]); }
    else if (Comparar(RecivoBuffer, "EEPROM15")) { ExtraerFLOAT(RecivoBuffer, CalServo[15]); }
    else if (Comparar(RecivoBuffer, "EEPROM16")) { ExtraerFLOAT(RecivoBuffer, CalServo[16]); }
    else if (Comparar(RecivoBuffer, "EEPROM17")) { ExtraerFLOAT(RecivoBuffer, CalServo[17]); }

    // Parametros desde esp32 guardados en eeprom
    else if (Comparar(RecivoBuffer, "EEPROM18")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.ZonaMuertaJoistick); } 
    else if (Comparar(RecivoBuffer, "EEPROM19")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Velocidad_Max_mm_s); }  
    else if (Comparar(RecivoBuffer, "EEPROM20")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Paso1_mm); } 
    else if (Comparar(RecivoBuffer, "EEPROM21")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Paso2_mm); } 
    else if (Comparar(RecivoBuffer, "EEPROM22")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Paso3_mm); } 
    else if (Comparar(RecivoBuffer, "EEPROM23")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Velocidad_Bajada_Patas_mm_s); } 
    else if (Comparar(RecivoBuffer, "EEPROM24")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.Velocidad_Subida_Patas_mm_s); } 
    else if (Comparar(RecivoBuffer, "EEPROM25")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.MultiplicadorParabola); } 
    else if (Comparar(RecivoBuffer, "EEPROM26")) { ExtraerFLOAT(RecivoBuffer, DesdeESP32.MultiplicadorAyudaPatas); } 
  }
}

// =================== ENVIAR DATOS UART ===================
void enviarDatosUART() {
  const int TMA = 30;                   //Tamaño Maximo Array
  const int NumMaxMensajesXciclo = 2;   //Numero maximo de mensajes a enviar por ciclo

  static char mensajePendiente[TMA+2][64];
  static char mensajePendientePrev[NumMaxMensajesXciclo][64];
  static int Num = 0;

  #define GuardarDatoINT(patron, variable, prev)                                                     \
    do {                                                                                             \
    if (Num < TMA) {                                                                                 \
    snprintf(mensajePendiente[Num], sizeof(mensajePendiente[Num]),"%s,%d+\n", patron, variable);     \
    prev = variable;                                                                                 \
    Num++;                                                                                           \
    }                                                                                                \
  } while(0)  

  #define GuardarDatoCHAR(patron, variable, prev)                                                    \
    do {                                                                                             \
    if (Num < TMA) {                                                                                 \
    snprintf(mensajePendiente[Num], sizeof(mensajePendiente[Num]),"%s,%s+\n", patron, variable);     \
    strcpy(prev, variable);                                                                          \
    Num++;                                                                                           \
    }                                                                                                \
  } while(0)  


  if (HaciaESP32.LEVANTADO != HaciaESP32.LEVANTADOprev)     {GuardarDatoINT("LEVANTADO", HaciaESP32.LEVANTADO, HaciaESP32.LEVANTADOprev); }
  if (HaciaESP32.ACOSTADO != HaciaESP32.ACOSTADOprev)       {GuardarDatoINT("ACOSTADO", HaciaESP32.ACOSTADO, HaciaESP32.ACOSTADOprev); }
  if (HaciaESP32.CALIBRANDO != HaciaESP32.CALIBRANDOprev)   {GuardarDatoINT("CALIBRANDO", HaciaESP32.CALIBRANDO, HaciaESP32.CALIBRANDOprev); }
  if (HaciaESP32.Altura != HaciaESP32.AlturaPrev)           {GuardarDatoINT("ALTURA",HaciaESP32.Altura,HaciaESP32.AlturaPrev); }
  if (HaciaESP32.Yaw != HaciaESP32.YawPrev)                 {GuardarDatoINT("YAW",HaciaESP32.Yaw,HaciaESP32.YawPrev); }
  if (HaciaESP32.Pitch != HaciaESP32.PitchPrev)             {GuardarDatoINT("PITCH",HaciaESP32.Pitch,HaciaESP32.PitchPrev); }
  if (HaciaESP32.Roll != HaciaESP32.RollPrev)               {GuardarDatoINT("ROLL",HaciaESP32.Roll,HaciaESP32.RollPrev); }


  // evita duplicados respecto al último enviado
  for (int i = 0; i < NumMaxMensajesXciclo; ++i) {
    if (strcmp(mensajePendiente[i], mensajePendientePrev[i]) != 0) {
      uart_puts(uart1, mensajePendiente[i]); // envía por UART
      strcpy(mensajePendientePrev[i], mensajePendiente[i]);
    }
  }


  // desplaza [1..Num-1] -> [0..Num-2]
  for (int i = NumMaxMensajesXciclo; i < Num; ++i) {
    strcpy(mensajePendiente[i - NumMaxMensajesXciclo], mensajePendiente[i]);
  }

  
  if (Num > 0) Num--;
}

// ==================== MAIN =====================
int main() {
  (void)stepAxis; // evitar warning de variable no usada
  stdio_init_all();

  uart_init(uart1, 250000);
  gpio_set_function(20, GPIO_FUNC_UART); // TX
  gpio_set_function(21, GPIO_FUNC_UART); // RX

  while (true) {
    recibirYprocesarDesdeUART();
    setupServos();
    { static unsigned long ANTERIOR = 0, intervalo = 5;
      if (millis() > ANTERIOR + intervalo) { ANTERIOR = millis();
    enviarDatosUART();
    }
    }
    Servo2040_PrintPower_Simple();
    
    if (mandoConectado && servosIniciados) {
      MOVIMIENTOPATAS ();
    }

    sleep_ms(1);
  }
}
