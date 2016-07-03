  /*
Lilian - DROID 2015

// andar (15,0); -> trás com os do\\\qis lados
// andar (15,1); -> frente com a direita para trás com a esquerda
// andar (15,2); -> frente com a esquerda para trás com a direita
// andar (15,3); -> frente com a direita e esquerda
// andar (12,3); // ligar motores da esquerda

dir: 
  0 : liga os dois lados para trás
  1 : liga o lado direito para frente e esquerdo para trás
  2 : liga o lado esquerdo para frente e direito para trás
  3 : liga os dois lados para frente
  
tipo:
  15  todos os motores ligados
  14  totos da esquerda e direita traseiro
  13  todos da esquerda e direita dianteiro
  12  motores da esquerda ligados
  11  motores esquerda traseiro e todos da direita
  10  motores esquerda e direita traseiros
  9   motor esquerda traseiro e direita dianteiro
  8   motor esquerda traseiro
  7   motor esquerda dianteiro e motores direita
  6   motor esquerda dianteiro e direita traseiros
  5   motores esquerda traseiros
  4   motor esquerdo dianteiro
  3   motores da direita ligados
  2   motor direito traseiro
  1   motor direito dianteiro
  0   nenhum motor ligado

*/
#include <HMC.h>
#define LIMITE0 34.0
#define LIMITE90 102.0
#define LIMITE180 180.0
#define LIMITE270 295.0

#define FRENTE1  4
#define FRENTE2  356

#define CURVA_ESQUERDA1  15
#define CURVA_ESQUERDA2  5
#define CURVA_ESQUERDA3  5

#define CURVA_DIREITA1  15
#define CURVA_DIREITA2  5
#define CURVA_DIREITA3  5

//Delays "PWM"
#define DELAY2ON 200
#define DELAY2OFF 50
#define DELAY3ON 250
#define DELAY3OFF 50
#define DELAY4ON 300
#define DELAY4OFF 50

//Posição dos cones:
#define CONE_1 65
#define CONE_2 205
#define CONE_3 37

float grau;
float media_bu[10];
float bussula;

int Tstart=0;
int Tatual=0;
int anda=0;
int chegou=0;
int num_cone=0;

int num=0;

int cont_andar_reto=0;

void setup(){
Serial.begin(9600);
pinMode(2, OUTPUT);
pinMode(3, OUTPUT); //DIREITA DIANTEIRO

pinMode(4, OUTPUT);
pinMode(5, OUTPUT); //DIREITA TRASEIRO

pinMode(6, OUTPUT);
pinMode(7, OUTPUT); //ESUERDA DIANTEIRO

pinMode(8, OUTPUT);
pinMode(9, OUTPUT); //ESQUERDA TRASEIRO

pinMode(10, OUTPUT); //sinal luminoso

  delay(5); // The HMC5843 needs 5ms before it will communicate
  HMC.init(false);
 
  HMC.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  HMC.setMode(0);
  
}

void desligamotor() {
digitalWrite(2,0);
digitalWrite(3,0);
digitalWrite(4,0);
digitalWrite(5,0);
digitalWrite(6,0);
digitalWrite(7,0);
digitalWrite(8,0);
digitalWrite(9,0);
};

void andar(int tipo, int dir){

  int DD, DT, ED, ET;
  
  
  DD = tipo%2; 
  tipo = tipo/2;
  DT = tipo%2; 
  tipo = tipo/2;
  ED = tipo%2; 
  tipo = tipo/2;
  ET = tipo%2;
  
  int HD, HE;
  
  HD = dir%2; 
  dir = dir/2; //1 horario 0 antihorario
  HE = dir%2;
  
  desligamotor();
  if(HD==1){
    digitalWrite(2, DD);
    digitalWrite(4, DT);
  }else{
    digitalWrite(3, DD);
    digitalWrite(5, DT);
  }
  
  if(HE==1){
    digitalWrite(6, ED);
    digitalWrite(8, ET);
  }else{
    digitalWrite(7, ED);
    digitalWrite(9, ET);
  }
 
}

int receber(int n){
  if(Serial.available()!=0){
    n = Serial.read();
    Serial.write(n);
  } 
  return(n);
}

int escutar() {
  num = receber(num);
   
  while((chegou == 0)){ //Escutar o netbook até que mande ir para um cone (31, 32 ou 33)

  num = receber(num);
  
  switch(num) {
   case 23: //correção máxima para esquerda (para frente)
     andar(15,1); 
    break;
   case 1:
     andar(3,3);  
    break; 
   case 2:
     andar(7,3);   
    break;
   case 3:
     andar(11,3);  
    break;
   case 4:
     andar(15,3);  
    break; 
   case 5:
     andar(14,3);   
    break;
   case 6:
     andar(13,3);  
    break;
   case 7:
     andar(12,3);  
    break; 
   case 8:
     andar(15,2);   
    break;
   case 9:
     desligamotor(); 
    break;
   case 10:
     andar(15,2);  
    break; 
   case 11:
     andar(3,0);  
    break;
   case 12:
     andar(7,0);  
    break;
   case 13:
     andar(11,0);  
    break; 
   case 14:
     andar(15,0);   
    break;
   case 15:
     andar(14,0);  
    break;
   case 16:
     andar(13,0);  
    break; 
   case 17:
     andar(12,0);   
    break;
   case 18:
     andar(15,1);  
    break;
   case 19:
       
    break; 
   case 20:
     
    break;
   case 21:
    digitalWrite(10, 1);
    delay(5000);
    digitalWrite(10, 0);
    Serial.write(31);
    delay(1);
    num = 32;
    chegou=1;
    break;
    
   case 22:
    digitalWrite(10, 0);
    break;  
  }
 }
 
 
}

void serialFlush() {
  while(Serial.available()>0) {
   char t = Serial.read(); 
  }
}

void normalizar () {
  
  float angle; 
  float fx,fy,fz;
  
  //delay(100); // There will be new values every 100ms
  HMC.getValues(&fx,&fy,&fz);
  angle = ((atan2((double)fy,(double)fx))*180/M_PI + 180);
  //Serial.print(" angle:"); //angle eh o angulo do mag
  //Serial.println(angle);  
 //normalizando 
  if(angle >= LIMITE0 && angle < LIMITE90){
    grau = (angle-LIMITE0)*(90.0/(LIMITE90 - LIMITE0));
  }
  
  if(angle >= LIMITE90 && angle < LIMITE180){
    grau = (angle-LIMITE90)*(90.0/(LIMITE180 - LIMITE90)) + 90.0;
  }
  
  if(angle >= LIMITE180 && angle < LIMITE270){
    grau = (angle-LIMITE180)*(90.0/(LIMITE270 - LIMITE180)) + 180.0;
  }
  
  if((angle >= LIMITE270 && angle < 360.0) || (angle >=0 && angle < LIMITE0)){
    if(angle < LIMITE270){angle = angle + 360;}
    grau = (angle-LIMITE270)*(90.0/(360.0 - LIMITE270 + LIMITE0)) + 270; // Perfect não mexer
  }
  
while (grau < 0) grau += 360;
while (grau > 360) grau -= 360;
  
  
  
 //Serial.print("x:");
 //Serial.println(fx);
 //Serial.print(" y:");      
 //Serial.println(fy);
 //Serial.print(" z:");
 //Serial.println(fz);
  
 Serial.print(" GRAU:"); //angle eh o angulo do mag
 Serial.println(grau);    //grau eh o grau real
}

void maximo() {
  float maxx = media_bu[0];
  int k = 0;
  for ( int i=0; i<9; i++){
    if(media_bu[i+1] > maxx){
      maxx = media_bu[i+1];
      k = i + 1;
    }
  }
  media_bu[k] = 0;
}

void minimo() {
  float minn = media_bu[0];
  int k = 0;
  for ( int i=0; i<9; i++){
    if(media_bu[i+1] < minn){
      minn = media_bu[i+1];
      k = i + 1;
    }
  }
  media_bu[k] = 0;
}

void andar_reto (float dir) {
  for(int i=0; i<10; i++){
    normalizar();
    media_bu[i]=grau;
  }
  if(grau>10&&(grau<350)){
  minimo();
  maximo();
  grau = 0;
  for(int i = 0; i < 10; i++){
    grau += media_bu[i];
  }
  
  grau = grau/8.0;
  }
  Serial.print(" media:"); //angle eh o angulo do mag
  Serial.println(grau);    //grau eh o grau real
  bussula=grau-dir;
  if( bussula<0)
    bussula+=360;
  Serial.print(" Bussola:"); 
  Serial.println(bussula);  
 /*if(((millis()-Tstart)>6000)&&((millis()-Tstart)<9000)){ 
  desligamotor(); anda=0;
 }
 if((millis()-Tstart)<6000){ 
  anda=1;
 } 
  if((millis()-Tstart)>9000){ 
  anda=1; Tstart=millis();
 }
    
 if(anda==1){*/
                        if ((bussula <= FRENTE1) || (bussula >= FRENTE2)){
				andar(15,3);   
                        }            
                         if ((bussula <= FRENTE1+CURVA_ESQUERDA1) && (bussula > FRENTE1))
				andar(11,3);

			 if ((bussula <= FRENTE1+CURVA_ESQUERDA1+CURVA_ESQUERDA2) && (bussula > FRENTE1+CURVA_ESQUERDA1)) {
				andar(7,3); delay(DELAY2ON); desligamotor(); delay(DELAY2OFF);
                                }
			 if ((bussula <= FRENTE1+CURVA_ESQUERDA1+CURVA_ESQUERDA2+CURVA_ESQUERDA3) && (bussula > FRENTE1+CURVA_ESQUERDA1+CURVA_ESQUERDA2)) {
				andar(3,3); delay(DELAY3ON); desligamotor(); delay(DELAY3OFF);
                                }
			 if ((bussula > FRENTE1+CURVA_ESQUERDA1+CURVA_ESQUERDA2+CURVA_ESQUERDA3) && (bussula <= 180)) {
				andar(15,1); delay(DELAY4ON); desligamotor(); delay(DELAY4OFF);
                                }
			 if ((bussula >= FRENTE2-CURVA_DIREITA1) && (bussula < FRENTE2))
				andar(14,3);
			 if ((bussula >= FRENTE2-CURVA_DIREITA1-CURVA_DIREITA2) && (bussula < FRENTE2-CURVA_DIREITA1)) {
				andar(13,3); delay(DELAY2ON); desligamotor(); delay(DELAY2OFF);
                                }
			 if ((bussula >= FRENTE2-CURVA_DIREITA1-CURVA_DIREITA2-CURVA_DIREITA3) && (bussula < FRENTE2-CURVA_DIREITA1-CURVA_DIREITA2)) {
				andar(12,3); delay(DELAY3ON); desligamotor(); delay(DELAY3OFF);
                                }
			 if ((bussula < FRENTE2-CURVA_DIREITA1-CURVA_DIREITA2-CURVA_DIREITA3) && (bussula > 180)) {
				andar(15,2);delay(DELAY4ON); desligamotor(); delay(DELAY4OFF);
                                }
//  }
}

void sirene (){

  digitalWrite(10, HIGH);  
  delay(5000);
  digitalWrite(10, LOW);
  
  
}

void ir_cone() {
  num=0;
  if(num_cone==0){
  while(num != 30){
    num = receber(num);
    //Serial.println("entrou1"); 
    andar_reto(CONE_1); 
  }
  
  escutar();
  }
  if(num_cone==1){
    andar(15,2); 
    delay(2001); 
    desligamotor(); 
    delay(1000);

    while(num != 50){
    num = receber(num);
    //Serial.println("entrou1"); 
    delay(5);
    Serial.write(49);
    delay(5);
  }
  //Tstart=millis();
      
     serialFlush();
  
    while(cont_andar_reto < 107){
    num = receber(num);
    cont_andar_reto++;
    //Serial.println("entrou1"); 
   //if((millis()-Tstart) < 15000)
      andar_reto(CONE_2); 
   //else if(((millis()-Tstart)>15000)&&((millis()-Tstart)<18000)) desligamotor();
   // else Tstart=millis();
  }
  desligamotor();
  sirene();
  }
  desligamotor();
  //num=0;
  //escutar();
 
  //Serial.write(32);  
  num_cone++;
  //andar(15,0);
  //delay(1000);
  //desligamotor();
  chegou = 0;
  //else sirene();
}


void loop(){
  
int i, j; 
int time;
num=0;
if(num_cone==0){
desligamotor();
  while(digitalRead(A0)==0) { //Esperar botão de start
     delay(10);
    //normalizar();
     
    }
    Serial.write(20);
      
    
  Serial.write(20); 
    
  while(num != 31){            //Esperar comando para seguir cone 1    
    num = receber(num);
  }
  ir_cone();
//  Tstart=millis();   
  //Partiu cone 1!!!!! :D
}
if(num_cone==1){
 // Serial.write(32);
 
 // while(num != 32){            //Esperar comando para seguir cone 1    
   // num = receber(num);
  //} 

  andar(15,0); delay(1000); desligamotor();
  ir_cone(); //Segue a bussola, acha o cone e liga a sirene.

}
}
