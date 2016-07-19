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

char comando;

void setup(){
  
Serial.begin(9600);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

}

void frente (int potEsquerda, int potDireita) {

  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, HIGH);
  digitalWrite(IN2_E, LOW); 
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW); 

}

void tras (int potEsquerda, int potDireita) {

  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, LOW);
  digitalWrite(IN2_E, HIGH);  
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);

}

void direita(int potEsquerda, int potDireita){

  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, HIGH);
  digitalWrite(IN2_E, LOW); 
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);
  

}

void esquerda(int potEsquerda, int potDireita){

  analogWrite(velEsquerda, potEsquerda);
  analogWrite(velDireita, potDireita);
  digitalWrite(IN1_E, LOW);
  digitalWrite(IN2_E, HIGH);
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW); 

}

void parar () {

  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);  
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW); 

}


void loop(){    
  
  int potEsquerda;
  int potDireita;
 
  if(Serial.available()>0) { 
 
    comando = Serial.read();
    
    if(comando == Frente) {
      frente(potEsquerda, potDireita);  }
      else if(comando == Tras) {
        tras(potEsquerda, potDireita); }
        else if(comando == Parar) {
          parar();}                               
          else if(comando == Esquerda) {
            esquerda(potEsquerda, potDireita);}
            else if(comando == Direita) {
              direita(potEsquerda, potDireita);}
  }
}

