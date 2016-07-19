/***************************************************************************************************************
Configurando bussola*/


const float LIMITEREAL0 = 70.8; 
const float LIMITEREAL90 = 126.4;
const float LIMITENEGATIVOREAL90 = -45.5;
const float LIMITEREAL180 = -173.9;    //-176; // Deve ser um valor um pouco maior que -180 da bussóla nao normalizada (Cerca de -178~-172)

// Para poder usar o código feito pelo Arthur
const float LIMITE0 = LIMITEREAL180 + 180;
const float LIMITE90 = LIMITENEGATIVOREAL90 + 180;
const float LIMITE180 = LIMITEREAL0 + 180;
const float LIMITE270 = LIMITEREAL90 + 180;


/***************************************************************************************************************
* Razor AHRS Firmware v1.4.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
*   * Add binary output of unfused sensor data for all 9 axes.
***************************************************************************************************************/

// Includes do GPS
/*****************************************************************/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

// Includes da IMU
/****************************************************************/
#include <Wire.h>

// Includes do Controle
/****************************************************************/
#include <VirtualWire.h>

// Includes do US
/****************************************************************/
#include <Ultrasonic.h>

// Definicoes da IMU
/************************************************************************************************************************************************************************************************/
// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 200// in milliseconds

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)


// Altymeter
#define ALT_SEA_LEVEL_PRESSURE 102133

/*
// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)
*/

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi



// Definicoes da GPS
/************************************************************************************************************************************************************************************************/
#define GPSBaud 4800

// Definicoes do Controle
/************************************************************************************************************************************************************************************************/
#define PINO_CONTROLE 46

// Definicoes do DRIVE
/************************************************************************************************************************************************************************************************/
#define IN1_E 4 
#define IN2_E 5
#define velEsquerda 3

#define IN1_D 6
#define IN2_D 7
#define velDireita 2

//#define IN4 3
#define Frente 'f'    //Frente
#define Tras 't'    //Parar
#define Esquerda 'e'
#define Direita 'd'
#define Parar 'p'


// Definicoes do US
/************************************************************************************************************************************************************************************************/
#define ECHO_PIN1    53
#define TRIGGER_PIN1 51
#define ECHO_PIN2    49
#define TRIGGER_PIN2 47
#define ECHO_PIN3    45
#define TRIGGER_PIN3 43
#define ECHO_PIN4    41
#define TRIGGER_PIN4 39
#define ECHO_PIN5    37
#define TRIGGER_PIN5 35
#define ECHO_PIN6    33
#define TRIGGER_PIN6 31

// Variáveis da IMU
/************************************************************************************************************************************************************************************************/

// OUTPUT OPTIONS
/*****************************************************************/

// RAW sensor data
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//float accel_min[3];
//float accel_max[3];

float magnetom[3];
//float magnetom_min[3];
//float magnetom_max[3];

float gyro[3];
//float gyro_average[3];
//int gyro_num_samples = 0;

float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3]= {0, 0, 0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw, pitch, roll;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

float angulo_normalizado;

// Variáveis do GPS
/************************************************************************************************************************************************************************************************/
//static const int RXPin = 4, TXPin = 3;
double lat,lng;
float speed;
boolean C1 = false, C2 = false, C3 = false;
const double CONE1_LAT =  -23.64732742;  // latitudes e longitudes dos cones  -23.64732742,-46.57261276
const double CONE1_LNG =  -46.57261276;
const double CONE2_LAT =  -15.762824;   // -15.762824,-47.859519
const double CONE2_LNG =  -47.859519;
const double CONE3_LAT =  -15.765286;
const double CONE3_LNG =  -47.871986;
const double INICIO_LAT = -23.64728546;      // latitude e longitude do ponto inicial    -23.64728546,-46.57264709
const double INICIO_LNG = -46.57264709;
const double DIRECAO_IMU_CONE1 = 96.6;   // angulo da IMU para os cone 
const double DIRECAO_IMU_CONE2 = 176.5;
const double DIRECAO_IMU_CONE3 = -96;
const double COURSE_CONE1 = atan2((CONE1_LNG-INICIO_LNG),(CONE1_LAT - INICIO_LAT));   // ângulos de trajetória iniciais para cada cone
const double COURSE_CONE2 = atan2((CONE2_LNG - CONE1_LNG),(CONE2_LAT - CONE1_LAT));
const double COURSE_CONE3 = atan2((CONE3_LNG - CONE2_LNG),(CONE3_LAT - CONE2_LAT));
double COURSE_ATUAL = 0;
double DELTA_COURSE = 0;  
double IMU_CORRIGIDA = DIRECAO_IMU_CONE1;
double DISTANCIA = 0;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

// Variáveis do Sensor de Luz
/************************************************************************************************************************************************************************************************/

int i, t, valor1, valor2, valor3;
int soma;
int limite1=250;
int limite2=250;
int limite3=250;
int cont=0;

// Variáveis do DRIVER
/************************************************************************************************************************************************************************************************/


// Variáveis do Controle
/************************************************************************************************************************************************************************************************/
byte message[VW_MAX_MESSAGE_LEN];    // Armazena as mensagens recebidas
byte msgLength = VW_MAX_MESSAGE_LEN; // Armazena o tamanho das mensagens
char comando ='L';

// Variáveis do US
/************************************************************************************************************************************************************************************************/
Ultrasonic us_frente_esquerda(TRIGGER_PIN6, ECHO_PIN6); 
Ultrasonic us_frente_direita(TRIGGER_PIN5, ECHO_PIN5);
Ultrasonic us_esquerda_frente(TRIGGER_PIN3, ECHO_PIN3);
Ultrasonic us_esquerda_tras(TRIGGER_PIN1, ECHO_PIN1);
Ultrasonic us_direita_frente(TRIGGER_PIN4, ECHO_PIN4);
Ultrasonic us_direita_tras(TRIGGER_PIN2, ECHO_PIN2);
float dist_obstaculo = 100;

//Variáveis globais
/************************************************************************************************************************************************************************************************/
int dandoRe = 0;
long tempoRe;
int start = 0;
int ajuste = 0;
int contAjusteFino = 0;
long latMed=0,lngMed=0;
double angulo_ajuste;
int ajustar_esquerda, ajustar_direita;
// Funcoes da IMU
/************************************************************************************************************************************************************************************************/

void ReadSensors() {
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();  
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  ReadSensors();
  
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
    // Magnetometer axis mapping
    Magn_Vector[1] = -magnetom[0];
    Magn_Vector[0] = -magnetom[1];
    Magn_Vector[2] = -magnetom[2];

    // Magnetometer values mapping
    Magn_Vector[0] -= MAGN_X_OFFSET;
    Magn_Vector[0] *= MAGN_X_SCALE;
    Magn_Vector[1] -= MAGN_Y_OFFSET;
    Magn_Vector[1] *= MAGN_Y_SCALE;
    Magn_Vector[2] -= MAGN_Z_OFFSET;
    Magn_Vector[2] *= MAGN_Z_SCALE;
  
    // Accelerometer axis mapping
    Accel_Vector[1] = accel[0];
    Accel_Vector[0] = accel[1];
    Accel_Vector[2] = accel[2];

    // Accelerometer values mapping
    Accel_Vector[0] -= ACCEL_X_OFFSET;
    Accel_Vector[0] *= ACCEL_X_SCALE;
    Accel_Vector[1] -= ACCEL_Y_OFFSET;
    Accel_Vector[1] *= ACCEL_Y_SCALE;
    Accel_Vector[2] -= ACCEL_Z_OFFSET;
    Accel_Vector[2] *= ACCEL_Z_SCALE;
    
    // Gyroscope axis mapping
    Gyro_Vector[1] = -gyro[0];
    Gyro_Vector[0] = -gyro[1];
    Gyro_Vector[2] = -gyro[2];

    // Gyroscope values mapping
    Gyro_Vector[0] -= GYRO_X_OFFSET;
    Gyro_Vector[0] *= GYRO_X_SCALE;
    Gyro_Vector[1] -= GYRO_Y_OFFSET;
    Gyro_Vector[1] *= GYRO_Y_SCALE;
    Gyro_Vector[2] -= GYRO_Z_OFFSET;
    Gyro_Vector[2] *= GYRO_Z_SCALE;
}

void calculate_IMU(){
  // Time to read the sensors again?
  if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    ReadSensors();
    
    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading

    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    
    float angle = TO_DEG(yaw)+180;
    
    if(angle >= LIMITE0 && angle < LIMITE90){
       angulo_normalizado = (angle-LIMITE0)*(90.0/(LIMITE90 - LIMITE0));
    }
      
    if(angle >= LIMITE90 && angle < LIMITE180){
      angulo_normalizado = (angle-LIMITE90)*(90.0/(LIMITE180 - LIMITE90)) + 90.0;
    }
    
    if(angle >= LIMITE180 && angle < LIMITE270){
      angulo_normalizado = (angle-LIMITE180)*(90.0/(LIMITE270 - LIMITE180)) + 180.0;
    }
    
    if((angle >= LIMITE270 && angle < 360.0) || (angle >=0 && angle < LIMITE0)){
      if(angle < LIMITE270){angle = angle + 360;}
      angulo_normalizado = (angle-LIMITE270)*(90.0/(360.0 - LIMITE270 + LIMITE0)) + 270; // Perfect não mexer
    }
    
    angulo_normalizado -= 180;
    
    Serial.println(angulo_normalizado);    Serial.print(";");
    delay(180);    Serial.print(TO_DEG(yaw));    Serial.print(";");
    Serial.print(TO_DEG(pitch));  Serial.print(";");
    Serial.print(TO_DEG(roll));   Serial.print(";");
    Serial.print(temperature);    Serial.print(";");
    Serial.print(pressure);       Serial.print(";");
    Serial.print(altitude);       Serial.println();
    /*Serial.println(LIMITE0);
    Serial.println(LIMITE90);
    Serial.println(LIMITE180);
    Serial.println(LIMITE270);*/
  }
}

void initialize_IMU(){
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
}

// Funcoes do GPS
/************************************************************************************************************************************************************************************************/

void initialize_GPS(){
  Serial1.begin(GPSBaud);
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    lat = gps.location.lat();
    lng = gps.location.lng();
    Serial.print(lat,8);
    Serial.print(F(","));
    Serial.print(lng,8);
    Serial.println();
  }
  else
  {
    Serial.print(F("INVALID"));
    lat = 0;
    lng = 0;
  }
  Serial.print(F("Speed: "));
  if (gps.speed.isValid())
  {
    speed = gps.speed.mps();
    Serial.print(speed);
    Serial.println();  
  }
  else
  {
    Serial.print(F("INVALID"));
    speed = 0;
  }
  
}

void calculate_GPS(){

    while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())) {
      Serial.println("AHAHAHAHAHAHAHHAHAHAA");
      displayInfo();
      CalculoDistancia();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    lat = 0;
    lng = 0;
  }
}

void CalculoDistancia ()
{ 
  
  if (lat!= 0 && lng!=0){
    
    if (!C1 &&  !C2 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE1_LAT, CONE1_LNG);
    else if (C1 &&  !C2 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE2_LAT, CONE2_LNG);
    else if (C1 &&  C2 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE3_LAT, CONE3_LNG);
  
    Serial.print("Distance (m) to cone: ");
    Serial.println(DISTANCIA);
  }
  
  else {
    Serial.print("Distance (m) to cone: ");
    Serial.println("Invalid");
    Serial.print("angulo: ");
    Serial.println("Invalid");
    Serial.print("Human directions: ");
    Serial.println("Invalid");  
  }
}

// Funcoes do Sensor de Luz
/************************************************************************************************************************************************************************************************/

void calculate_LUZ() {
  
  //Faz uma média dos valores lidos pelos sensores
  for (t=0; t<2; t++){

    valor1+=analogRead(A0);
  }
  for (t=0; t<2; t++){
 
    valor2+=analogRead(A1);
  }
  //for (t=0; t<10; t++){
 
   // valor3+=analogRead(A2);
  //}
  Serial.println("SENSORES DE LUZ");
  valor1=(valor1/2);
  valor2=(valor2/2);
  Serial.println(valor1);
  Serial.println(valor2);
  //valor3=(valor3/10);
 
  //Verifica a leitura dos sensores
  if((valor1>limite1)&&(valor2>limite2)){
    Serial.println("ta na Grama");
  }
  else{
    //Verifica se ja esta com todos os sensores em cima do marco
    if(valor1 <limite1){
     Serial.println("To subindo no marco");
    }
    if(valor2 <limite2){
      Serial.println("To no marco 1");
        frente(210, 200); //vai um pouco pra frente
        delay(100);
        parar();          
        
        if (!C1 && !C2 && !C3){
          C1 = true;
          while (true) {
            parar();
          }
        }else if (C1 && !C2 && !C3)
          C2 = true;
        else if (C1 && C2 && !C3){
          C3 = true;
          while (true) {
            parar();
          }
       }
       LigarSinal();
       AjustarPosicao(); 
    }
  }
  valor1=0;
  valor2=0;
  //valor3=0;
 
}

// Funcoes do DRIVER
/************************************************************************************************************************************************************************************************/


// potEsquerda e potDireita sao pwm do motor -> 0 a 255 (usar 180);

void initialize_DRIVER(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}

void frente (int potEsquerda, int potDireita) {
  Serial.println("To indo pra frente!!!");


}

void tras (int potEsquerda, int potDireita) {
  Serial.println("To dando re!!!");

}

void direita(int potEsquerda, int potDireita){
  Serial.println("To virando pra direita!!!");
  frente (potEsquerda, potDireita);
  

}

void direita_total(int potEsquerda, int potDireita){
  Serial.println("To virando TODO pra direita!!!"); 
  

}

void esquerda(int potEsquerda, int potDireita){
  Serial.println("To virando pra esquerda!!!");
  frente (potEsquerda, potDireita);

}

void esquerda_total(int potEsquerda, int potDireita){
  Serial.println("To virando TODO pra esquerda!!!");
}


void parar () {


}
// Funcoes Controle
/************************************************************************************************************************************************************************************************/
void initialize_CONTROLE(){
  
  vw_set_rx_pin(PINO_CONTROLE); // Define o pino 5 do Arduino como entrada 
  //de dados do receptor
  vw_setup(1000);             // Bits por segundo
  vw_rx_start();        

}

void calculate_CONTROLE(){    
  
  int potEsquerda = 150;
  int potDireita = 150;
 
  uint8_t message[VW_MAX_MESSAGE_LEN];    
  uint8_t msgLength = VW_MAX_MESSAGE_LEN; 
  
  if(vw_get_message(message, &msgLength)) { 
 
       for (int i = 0; i < msgLength; i++)
       {
          comando = message[i];
       
       }
    
    if(comando == Frente) {
      frente(potEsquerda, potDireita);  
    }
    else if(comando == Tras) {
      tras(potEsquerda, potDireita); 
    }
    else if(comando == Parar) {
      parar();
    }                               
    else if(comando == Esquerda) {
      potEsquerda = 0;
      potDireita = 150;
      esquerda(potEsquerda, potDireita);
    
      //esquerda_total(150, 140);
    }
    else if(comando == Direita) {
      potEsquerda = 150;
      potDireita = 0;
      direita(potEsquerda, potDireita);
    
      //direita_total(150, 140);
    }
  }
}

// Funcoes Sensor Ultra-som
/************************************************************************************************************************************************************************************************/
void calculate_US (){
  Serial.println("aqui");
  Serial.println(us_esquerda_frente.Ranging(CM));
  delay(200);  

}

// Funcoes Sinal Luminoso
/************************************************************************************************************************************************************************************************/
void initialize_SINAL(){
  
  pinMode(52, OUTPUT);
  digitalWrite(52, LOW);
}


void LigarSinal() {
 
   //Liga o sinal lumino
   digitalWrite(52, HIGH);
   delay(1000);
   digitalWrite(52, LOW);

}

// Funcoes Botao Start
/************************************************************************************************************************************************************************************************/

void initialize_START(){
  
    pinMode(50, INPUT);
}

void calculate_START(){
  
    if (digitalRead(50) == LOW)
      start = 1;
    Serial.println((digitalRead(50)));
}


// Funcoes de Cálculo do curso da trajetória, correcao trajetória
/************************************************************************************************************************************************************************************************/

// Alinha o robô de acordo com o ângulo definido pela IMU
void Alinhar (){
  
  double DIRECAO_CORRECAO = IMU_CORRIGIDA - angulo_normalizado;
  
  DIRECAO_CORRECAO = ArrumarAngulo(DIRECAO_CORRECAO);
  Serial.println("DIRECAO_CORRECAO");
  Serial.println(DIRECAO_CORRECAO);

  //digitalWrite(52, LOW);

  // resolver faixa de valores considerada certa com testes!!!

        if (DIRECAO_CORRECAO > 5 && DIRECAO_CORRECAO < 10) {
         // reduzir velocidade
        // virar o robô no sentido horário
            Serial.println("Corrigindo para direita");
            direita(200, 190);
        }  
        else if (DIRECAO_CORRECAO < -5 && DIRECAO_CORRECAO > -10) {
         // reduzir velocidade
        // virar o robô no sentido anti - horário
            Serial.println("Corrigindo para esquerda");
            esquerda(200, 190);
        }
        else if (DIRECAO_CORRECAO > 10) {
         // reduzir velocidade
        // virar o robô no sentido horário
            Serial.println("Corrigindo para direita");
            direita(200, 0);
        }  
        else if (DIRECAO_CORRECAO < -10) {
         // reduzir velocidade
        // virar o robô no sentido anti - horário
            Serial.println("Corrigindo para esquerda");
            esquerda(0, 190);
        }
        else {
            frente(200,190);
            digitalWrite(52, HIGH);
       }
      
}

void CalculoDirecao (){
  
  //MUDAR - VERIFICAR SE TEM QUE INVERTER O DELTA_COURSE
  //Nao achou nenhum dos cones ainda  
  if (!C1 && !C2 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE1_LNG-lng),(CONE1_LAT - lat)); 
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE1;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    DELTA_COURSE = ArrumarAngulo(DELTA_COURSE);   // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE1 + DELTA_COURSE;
    Serial.println("A outra IMU Corrigida");
    Serial.println(ArrumarAngulo(DIRECAO_IMU_CONE1 - DELTA_COURSE));
    
  }  
  // Ja encontrou o cone1
  if (C1 && !C2 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE2_LNG-lng),(CONE2_LAT - lat));  
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE2;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE2 + DELTA_COURSE;
       
  }  
  // Ja achou cones 1 e 2
   if (C1 && C2 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE3_LNG-lng),(CONE3_LAT - lat));  
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE3;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE3 + DELTA_COURSE;   
  }  
  Serial.println("DELTA COURSE");
  Serial.println(DELTA_COURSE);
  IMU_CORRIGIDA = ArrumarAngulo(IMU_CORRIGIDA);
  Serial.println("IMU CORRIGIDA");
  Serial.println(IMU_CORRIGIDA);
    
}

void CalculoDirecaoAjusteFino (int esq, int dir){
 
  if (esq){
      IMU_CORRIGIDA = angulo_ajuste - 45;
  }
  else if (dir){
  
      IMU_CORRIGIDA = angulo_ajuste + 45; 
  }
  
  IMU_CORRIGIDA = ArrumarAngulo(IMU_CORRIGIDA);
  Serial.println("IMU CORRIGIDA AJUSTE");
  Serial.println(IMU_CORRIGIDA);
 
}

double ArrumarAngulo (double Angulo) {
    
  if(Angulo > 180)
    Angulo = Angulo - 360;
    
  else if (Angulo < -180)
    Angulo = Angulo + 360;

  return Angulo;
}

void AjustarPosicao (){

  // Arrumar em uma posicao para ir pro proximo cone
  // inicialmente para e da uma ré
  
  parar();
  tras (150, 150);
  delay(1000);
  parar();
  // Arrumar para ir pro cone 2
  if (C1 && !C2 && !C3){
    IMU_CORRIGIDA = DIRECAO_IMU_CONE2;        
  }
  // Arrumar para ir pro cone 3
  if (C1 && C2 && !C3){
     IMU_CORRIGIDA = DIRECAO_IMU_CONE3;      
  }
  Alinhar();
}


void AjusteFino () {
  CalculoDirecaoAjusteFino (ajustar_esquerda,ajustar_direita); // angulo_ajuste;
  if (ajustar_esquerda && ArrumarAngulo(angulo_ajuste - angulo_normalizado) < 0) {
    ajustar_esquerda = 1;
  }else{
    ajustar_esquerda = 0;
    ajustar_direita = 1; 
  }  
  if (ajustar_direita && ArrumarAngulo(angulo_ajuste - angulo_normalizado) > 0) {
    ajustar_direita = 1;
  }else{
    ajustar_direita = 0;
    ajustar_esquerda = 1; 
  }  
  
  Alinhar();

  
}
/*
void DetectarObstaculo () {

  if (C1 && C2 && !C3){
    if (DISTANCIA < 10){
      ajuste = 1;
      
    
    }
 
  } 
}*/

// Main setup e Main Loop
/************************************************************************************************************************************************************************************************/

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT_BAUD_RATE);
  initialize_IMU();
  initialize_GPS();
  initialize_DRIVER();
  initialize_CONTROLE();
  initialize_START();
  initialize_SINAL();
}

void loop(){
  

  calculate_CONTROLE();
  calculate_START();
  if (comando == 'L' && start == 1){
    calculate_IMU();
    calculate_GPS();
    calculate_LUZ();
    //DetectarCone(); Separar a deteccao do cone pra outra funcao ?
    //if(DISTANCIA>8){
      if (lat!=0 && lng!=0){
        digitalWrite(52, LOW); //Desliga o Sinal Luminoso 
        CalculoDirecao();
        angulo_ajuste = IMU_CORRIGIDA;
        ajustar_esquerda = 1;
        ajustar_direita = 0;
      }
      Alinhar();
    //}else if(DISTANCIA < 8){
    //  digitalWrite(52, HIGH); 
    //  AjusteFino();          //Liga o Sinal Luminoso
    //}
    //DetectarObstaculo();  // Fazer a funcao de detectar obstaculo
    //Alinhar();
  }
  //calculate_US();
  
}
