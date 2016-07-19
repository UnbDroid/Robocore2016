//*********************Programa setup*************************************************************************//

//Pega os dados iniciais do GPS e da IMU


/***************************************************************************************************************
Includes e variaveis do GPS*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

static const uint32_t GPSBaud = 4800;
double lat,lng;
double speed;

TinyGPSPlus gps;


/***************************************************************************************************************
Configurando bussola*/


const float LIMITEREAL0 = -58.5;  
const float LIMITEREAL90 = 123;
const float LIMITENEGATIVOREAL90 = -122.63;
const float LIMITEREAL180 = -172;

// Para poder usar o código feito pelo Arthur
const float LIMITE0 = LIMITEREAL180 + 180;
const float LIMITE90 = LIMITENEGATIVOREAL90 + 180;
const float LIMITE180 = LIMITEREAL0 + 180;
const float LIMITE270 = LIMITEREAL90 + 180;
/*******************************************************************************************************************/

// Includes da IMU
/****************************************************************/
#include <Wire.h>

// Definicoes da IMU
/************************************************************************************************************************************************************************************************/
// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 200  // in milliseconds

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
    
    Serial.println("Angulo normalizado:");
    Serial.println(angulo_normalizado);
    delay(180);    Serial.print(TO_DEG(yaw));    Serial.print(";");
    Serial.print(TO_DEG(pitch));  Serial.print(";");
    Serial.print(TO_DEG(roll));   Serial.print(";");
    Serial.print(temperature);    Serial.print(";");
    Serial.print(pressure);       Serial.print(";");
    Serial.print(altitude);       Serial.println();
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
    Serial.print(lat, 6);
    Serial.print(F(","));
    Serial.print(lng, 6);
  }
  else
  {
    Serial.print(F("INVALID"));
    lat = 0;
    lng = 0;
  }

  Serial.println(F("Speed: "));
  if (gps.speed.isValid())
  {
    speed = gps.speed.mps();
    Serial.println(speed);  
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
      displayInfo();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    lat = 0;
    lng = 0;
  }
}

// Main setup e Main Loop
/************************************************************************************************************************************************************************************************/


void setup()
{
  Serial.begin(115200);
  initialize_IMU();
  initialize_GPS();
}

void loop(){
  calculate_IMU();
  calculate_GPS();
}
