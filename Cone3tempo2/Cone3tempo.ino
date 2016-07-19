/***************************************************************************************************************
Configurando bussola*/


const float LIMITEREAL0 = -41.5;//-26;  
const float LIMITEREAL90 = 7.9;//24;
const float LIMITENEGATIVOREAL90 = -91.7;//-78;
const float LIMITEREAL180 = -177; //-178;    //-176; // Deve ser um valor um pouco maior que -180 da bussóla nao normalizada (Cerca de -178~-172)

// Para poder usar o código feito pelo Arthur
const float LIMITE0 = LIMITEREAL180 + 180;
const float LIMITE90 = LIMITENEGATIVOREAL90 + 180;
const float LIMITE180 = LIMITEREAL0 + 180;
const float LIMITE270 = LIMITEREAL90 + 180;

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
#define address 0x1E
// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Definicoes da GPS
/************************************************************************************************************************************************************************************************/
#define GPSBaud 4800

// Definicoes do Controle
/************************************************************************************************************************************************************************************************/
#define PINO_CONTROLE 46


// Definicoes do START
/************************************************************************************************************************************************************************************************/
#define PINO_START 50


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


// Definicoes Sinal Luminoso
/************************************************************************************************************************************************************************************************/
#define PINO_SINAL 38


// Variáveis da IMU
/************************************************************************************************************************************************************************************************/

// DCM variables
float MAG_Heading;
float yaw;
float angulo_normalizado;

// Variáveis do GPS
/************************************************************************************************************************************************************************************************/
//static const int RXPin = 4, TXPin = 3;
double lat,lng;
float speed;
boolean C1 = false, C2 = false, C25 = false, C3 = false;
const double CONE1_LAT =  -23.64732360;  // latitudes e longitudes dos cones  Location: -23.64730644,-46.57260894  //,  -23.64730644,-46.57263183
const double CONE1_LNG =  -46.57263565 ; //-23.64732360,-46.57263565
const double CONE2_LAT =  -23.64722251;   // -23.64722251,-46.57280349
const double CONE2_LNG =  -46.57280349;   // -46.57278442
const double CONE25_LAT =  -23.64706611;   // Location: -23.64706611,-46.57263565 para ir direto: -23.64703750,-46.57268142 // -23.64706611,-46.57255554
const double CONE25_LNG =  -46.57255554;   
const double CONE3_LAT =  -23.64701271;      // -23.64700126,-46.57264709
const double CONE3_LNG =  -46.57264328;     //-23.64701271,-46.57264328
const double INICIO_LAT = -23.64698219;      // latitude e longitude do ponto inicial    Location: -23.64695739,-46.57279586
const double INICIO_LNG = -46.57277679;   ///-23.64698219,-46.57277679


const double DIRECAO_IMU_CONE1 = 39;   // angulo da IMU para os cone //-22 // 34 //40.8 //45 //48
const double DIRECAO_IMU_CONE2 = -109.5; //-97 //-95 //-103.6

const double DIRECAO_IMU_CONE25 = 109.9;  // direto pro cone 3: -176 //  
const double DIRECAO_IMU_CONE3 = -110;    // indo direto 125

const double COURSE_CONE1 = atan2((CONE1_LNG-INICIO_LNG),(CONE1_LAT - INICIO_LAT));   // ângulos de trajetória iniciais para cada cone
const double COURSE_CONE2 = atan2((CONE2_LNG - CONE1_LNG),(CONE2_LAT - CONE1_LAT));
const double COURSE_CONE25 = atan2((CONE25_LNG - CONE2_LNG),(CONE25_LAT - CONE2_LAT));
const double COURSE_CONE3 = atan2((CONE3_LNG - CONE25_LNG),(CONE3_LAT - CONE25_LAT));
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
int limite1=300;
int limite2=300;
int limite3=300;
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
char comandoCamera = 'K';
int flag_LUZ = 1;
unsigned long tempo;
// Funcoes da IMU
/************************************************************************************************************************************************************************************************/

void calculate_IMU(){
    int x,y,z; //triple axis data
  
    // Indica ao HMC5883 para iniciar a leitura
    Wire.beginTransmission(address);
    Wire.write(0x03); //select register 3, X MSB register
    Wire.endTransmission();
   
    // Le os dados de cada eixo, 2 registradores por eixo
    Wire.requestFrom(address, 6);
    if(6<=Wire.available())
    {
      x = Wire.read()<<8; //X msb
      x |= Wire.read(); //X lsb
      z = Wire.read()<<8; //Z msb
      z |= Wire.read(); //Z lsb
      y = Wire.read()<<8; //Y msb
      y |= Wire.read(); //Y lsb
    }
    
    // Imprime os vaores no serial monitor
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("  y: ");
    Serial.print(y);
    Serial.print("  z: ");
    Serial.println(z);
    
    MAG_Heading = atan2(-y, x);
    yaw = MAG_Heading;
    Serial.print("Heading: ");
    Serial.println(MAG_Heading*180/3.1415);
    delay(250);

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
    Serial.print(TO_DEG(yaw));    Serial.println(";");
    /*Serial.println(LIMITE0);
    Serial.println(LIMITE90);
    Serial.println(LIMITE180);
    Serial.println(LIMITE270);*/
}

void initialize_IMU(){
  Wire.begin();
  
  // Inicializa o HMC5883
  Wire.beginTransmission(address);
  // Seleciona o modo
  Wire.write(0x02); 
  // Modo de medicao continuo
  Wire.write(0x00); 
  Wire.endTransmission();
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
      displayInfo();
      CalculoDistancia();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    lat = 0;
    lng = 0;
    DISTANCIA = 100;
  }
}

void CalculoDistancia ()
{ 
  if (lat!= 0 && lng!=0){
    
    if (!C1 &&  !C2 &&  !C25 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE1_LAT, CONE1_LNG);
    else if (C1 &&  !C2 && !C25 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE2_LAT, CONE2_LNG);
    else if (C1 &&  C2 && !C25 && !C3)
      DISTANCIA = gps.distanceBetween(lat, lng, CONE25_LAT, CONE25_LNG);
    else if (C1 &&  C2 && C25 && !C3)
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
    if((valor1 <limite1) && flag_LUZ){
     Serial.println("To subindo no marco");
    }
    if((valor2 <limite2) && flag_LUZ){
      Serial.println("To no marco 1");
        //frente(210, 200); //vai um pouco pra frente
        //delay(100);
        parar();                  
        if (!C1 && !C2 && !C3){
          C1 = true;
          flag_LUZ = 0;
          tempo = millis();
          
        }else if (C1 && !C2 && !C25 && !C3 && flag_LUZ){
          C2 = true;
          flag_LUZ = 0;
          tempo = millis();        
        }else if (C1 && C2 && C25 && !C3 && flag_LUZ){
          C3 = true;
          LigarSinal();
         parar(); 
          while (true) {
            parar();
          }
       }
       LigarSinal(); 
       ComunicarCamera('N');
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
  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, HIGH);
  digitalWrite(IN2_E, LOW); 
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW); 

}

void tras (int potEsquerda, int potDireita) {
  Serial.println("To dando re!!!");
  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, LOW);
  digitalWrite(IN2_E, HIGH);  
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);

}

void direita(int potEsquerda, int potDireita){
  Serial.println("To virando pra direita!!!");
  frente (potEsquerda, potDireita);
  

}

void direita_total(int potEsquerda, int potDireita){
  Serial.println("To virando TODO pra direita!!!"); 
  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, HIGH);
  digitalWrite(IN2_E, LOW); 
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);
  

}

void esquerda(int potEsquerda, int potDireita){
  Serial.println("To virando pra esquerda!!!");
  frente (potEsquerda, potDireita);

}

void esquerda_total(int potEsquerda, int potDireita){
  Serial.println("To virando TODO pra esquerda!!!");
  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, LOW);
  digitalWrite(IN2_E, HIGH);
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW); 

}


void parar () {

  digitalWrite(IN1_E, LOW);
  digitalWrite(IN2_E, LOW);  
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, LOW); 

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

// Funcoes Camera
/************************************************************************************************************************************************************************************************/

void initialize_CAMERA(){

  Serial.begin(115200);
}

void calculate_CAMERA(){

  if(Serial.available() > 0){
  
    comandoCamera = Serial.read();
  
  }

}

// Ordem = Y conecta a camera, ordem = N desconecta a camera
void ComunicarCamera(char ordem){

  Serial.println(ordem);

}

// Funcoes Sensor Ultra-som
/************************************************************************************************************************************************************************************************/
void calculate_US (){
  Serial.println("aqui");
  Serial.println(us_esquerda_frente.Ranging(CM));  
}

// Funcoes Sinal Luminoso
/************************************************************************************************************************************************************************************************/
void initialize_SINAL(){
  
  pinMode(PINO_SINAL, OUTPUT);
  digitalWrite(PINO_SINAL, LOW);
}


void LigarSinal() {
 
   //Liga o sinal luminoso
   digitalWrite(PINO_SINAL, HIGH);
}


void DesligarSinal() {
 
   //Desligar o sinal luminoso
   digitalWrite(PINO_SINAL, LOW);
}

// Funcoes Botao Start
/************************************************************************************************************************************************************************************************/

void initialize_START(){
  
    pinMode(PINO_START, INPUT);
}

void calculate_START(){
  
    if (digitalRead(PINO_START) == LOW)
      start = 1;
    Serial.println((digitalRead(PINO_START)));
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
  if (DIRECAO_CORRECAO > 3.5 && DIRECAO_CORRECAO < 10) {
   // reduzir velocidade
  // virar o robô no sentido anti-horário
      Serial.println("Corrigindo para esquerda");
      esquerda(170, 210);
  }  
  else if (DIRECAO_CORRECAO < -3.5 && DIRECAO_CORRECAO > -10) {
   // reduzir velocidade
  // virar o robô no sentido horário
      Serial.println("Corrigindo para direita");
      direita(220, 170);
  }
  else if (DIRECAO_CORRECAO > 10) {
   // reduzir velocidade
  // virar o robô no sentido anti-horário
      Serial.println("Corrigindo para esquerda");
      esquerda(0, 210);
  }  
  else if (DIRECAO_CORRECAO < -10) {
   // reduzir velocidade
  // virar o robô no sentido horário
      Serial.println("Corrigindo para direita");
      direita(220, 0);
  }
  else {
    if(C1 && !C2 && !C3 && !C25)
      frente(175,145);
    else
      frente(220,190);
      //digitalWrite(52, HIGH);
 }

}

void CalculoDirecao (){
  
  //Nao achou nenhum dos cones ainda  
  if (!C1 && !C2 && !C25 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE1_LNG-lng),(CONE1_LAT - lat)); 
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE1;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    DELTA_COURSE = ArrumarAngulo(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE1 - DELTA_COURSE;  // Verificar se é menos mesmo!!!
    
  }  
  // Ja encontrou o cone 1
  if (C1 && !C2 && !C25 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE2_LNG-lng),(CONE2_LAT - lat));  
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE2;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    DELTA_COURSE = ArrumarAngulo(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE2 - DELTA_COURSE;
       
  }  
  // Ja achou cones 1 e 2
   if (C1 && C2 && !C25 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE25_LNG-lng),(CONE25_LAT - lat));  
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE25;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    DELTA_COURSE = ArrumarAngulo(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE25 - DELTA_COURSE;   
  }
  // Ja achou cones 1, 2 e 25
   if (C1 && C2 && C25 && !C3) {
        
    // verificando se o robô da seguindo o curso do gps;
    COURSE_ATUAL = atan2((CONE3_LNG-lng),(CONE3_LAT - lat));  
    DELTA_COURSE = COURSE_ATUAL - COURSE_CONE3;
    DELTA_COURSE = TO_DEG(DELTA_COURSE);
    DELTA_COURSE = ArrumarAngulo(DELTA_COURSE);
    // define a orientação da IMU corrigida do robô a partir da orientação do gps
    IMU_CORRIGIDA = DIRECAO_IMU_CONE3 - DELTA_COURSE;   
  }  
  Serial.println("DELTA COURSE");
  Serial.println(DELTA_COURSE);
  IMU_CORRIGIDA = ArrumarAngulo(IMU_CORRIGIDA);
  Serial.println("IMU CORRIGIDA");
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
  delay(500);
  DesligarSinal();
  //tras(200, 190);
  frente(200, 190);
  delay(500);
  
  // Arrumar para ir pro cone 2
  if (C1 && !C2 && !C25 && !C3){
    IMU_CORRIGIDA = DIRECAO_IMU_CONE2;        
  }
  // Arrumar para ir pro cone 25
  if (C1 && C2 && !C25 && !C3){
     IMU_CORRIGIDA = DIRECAO_IMU_CONE25;      
  }
  
  // Arrumar para ir pro cone 3
  if (C1 && C2 && C25 && !C3){
     IMU_CORRIGIDA = DIRECAO_IMU_CONE3;      
  }
  Alinhar();
}

// Funcoes Ajuste Camera
/************************************************************************************************************************************************************************************************/

void AjusteCamera(){
  
    if (comandoCamera == 'd') {
     // reduzir velocidade
    // virar o robô no sentido horário
        Serial.println("Corrigindo para direita");
        direita(200, 180);
    }  
    else if (comandoCamera == 'e') {
     // reduzir velocidade
    // virar o robô no sentido anti - horário
        Serial.println("Corrigindo para esquerda");
        esquerda(190, 200);
    }
    else if (comandoCamera == 'r') {
     // reduzir velocidade
    // virar o robô no sentido horário
        Serial.println("Corrigindo para direita");
        direita(200, 0);
    }  
    else if (comandoCamera == 'l') {
     // reduzir velocidade
    // virar o robô no sentido anti - horário
        Serial.println("Corrigindo para esquerda");
        esquerda(0, 190);
    }
    else if (comandoCamera == 'f'){
        frente(190,180);
        
   }
  
}


// Funcao inicio
/************************************************************************************************************************************************************************************************/

void inicio (){
  frente(200,190);
  delay(1000);  
}


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
  //initialize_CAMERA();
}

int comecar = 0;
int flag_afasta = 0;


void loop(){
  calculate_CONTROLE();
  calculate_START();
  if (comando == 'L' && start == 1){
    if(comecar == 0){
      inicio();
      comecar++;
    }
    calculate_IMU();
    calculate_GPS();
    calculate_LUZ();
    if (lat!= 0 && lng!=0){
      CalculoDirecao();
    }
    if(!flag_LUZ && ((millis() - tempo) >= 10000)){
      Serial.print("Passou o tempo");
      Serial.println((millis() - tempo));
      flag_LUZ  = 1;  
    }
    Serial.println("Flag");
    Serial.println(flag_LUZ);
    Serial.println("millis");
    Serial.println(millis());
    Serial.println("Tempo");
    Serial.println(tempo);


    if(C1 && C2 && !C25 && !C3 &&  ((millis() - tempo) >= 15000)  ){
       C25 = true;
       IMU_CORRIGIDA = DIRECAO_IMU_CONE3;
    }
    Alinhar();  
  }
}
  