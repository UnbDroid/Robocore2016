#define GPSRATE 4800
//#define GPSRATE 38400
 
// GPS parser for 406a
#define BUFFSIZ 90 // plenty big
void setup() 
{ 

  Serial.begin(9600);
  Serial1.begin(GPSRATE);
   
  // prints title with ending line break 
  Serial.println("GPS parser"); 
  delay(100); 
} 
 
 
void loop() 
{ 
 char entrada=0;
 String resposta = "";
 while ((entrada = Serial1.read()) != 10)
 {
   if (entrada > 0)
   
   resposta += entrada;
 }  
 if(!resposta.equals("")) Serial.println(resposta);
 
}
