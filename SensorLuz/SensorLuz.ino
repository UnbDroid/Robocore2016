int i, t, valor1, valor2, valor3;
int soma;
int limite1=400;
int limite2=400;
int limite3=400;
int cont=0;

void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(9600);
  

}

void loop() {
  //Faz uma média dos valores lidos pelos sensores
  for (t=0; t<10; t++){
   // Serial.println(analogRead(A1));
    valor1+=analogRead(A1);
  }
  for (t=0; t<10; t++){
   // Serial.println(analogRead(A1));
    valor2+=analogRead(A2);
  }
  for (t=0; t<10; t++){
   // Serial.println(analogRead(A1));
    valor3+=analogRead(A3);
  }
  valor1=(valor1/10);
  //Verifica a leitura dos sensores
  if((valor1>limite1)&&(valor2>limite2)&&(valor3>limite3)){
    Serial.println("ta na Grama");
    delay(100);
  }
  else{
    //Verifica se ja esta com todos os sensores em cima do marco
    if(valor1<limite1){
      Serial.println("To subindo no marco");
    }
    if(valor2<limite2){
      Serial.println("To no marco");
      if(valor3<limite3){
        Serial.println("To no marco");
      }
    }
  }
    cont++;
   Serial.println(cont);
   Serial.println("To no marco");

  Serial.println("media");
  Serial.println(valor1);
    delay(1000);
    valor1=0;
    valor2=0;
    valor3=0;
 // }
}
