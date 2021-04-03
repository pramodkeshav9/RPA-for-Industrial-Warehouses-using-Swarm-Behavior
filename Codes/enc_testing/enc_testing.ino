// Rotary Encoder Inputs
#define inputA 13
#define inputB 19

//mtr ctrl
const int vcc1 = 22;
const int vcc2 = 23;
const int in_1 = 5 ;
const int in_2 = 4 ;
const int in_5 = 8 ;
const int in_6 = 9 ;

int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
int Old;
int New;
int Out; 

void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (inputA,INPUT);
   pinMode (inputB,INPUT);

   pinMode(in_1,OUTPUT) ;  //Logic pins are also set as output
   pinMode(in_2,OUTPUT) ;
   pinMode(in_5,OUTPUT) ;  //Logic pins are also set as output
   pinMode(in_6,OUTPUT) ;
   pinMode(vcc1, OUTPUT);
   pinMode(vcc2, OUTPUT);
  
   Serial.begin (115200);
   
  Old = digitalRead(inputA);
   
 
 } 
 
 void loop() { 
  Serial.print("Old:");
  Serial.println(Old);
  digitalWrite(in_1,HIGH) ;
  digitalWrite(in_2,LOW) ;
  digitalWrite(in_5,HIGH);
  digitalWrite(in_6,LOW);
  
    
    New = digitalRead (inputA) * 2 + digitalRead (inputB); // Convert binary input to decimal value
    Serial.println(Old,New);
    Serial.println(Old * 4 + New);
    Out = QEM [Old * 4 + New];
    
    Old = New;
    
    if(Out ==0)
    Serial.println("Not moving");
    else if(Out == 1)
    Serial.println("CW");
    else if(Out == -1)
    Serial.println("CCW");
    else
    Serial.println("Rubbish");
    
    delay(2000);
     digitalWrite(in_1,LOW);
     digitalWrite(in_2,LOW);
     digitalWrite(in_5,LOW);
     digitalWrite(in_6,LOW);
     
     
    
   // Update previousStateCLK with the current state
   //previousStateCLK = currentStateCLK; 
 }
