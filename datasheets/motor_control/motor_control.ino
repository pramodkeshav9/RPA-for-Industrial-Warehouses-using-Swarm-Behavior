//const int pwm1 = 2 ;
//const int pwm2 = 3;
//const int pwm3 = 3;
//const int pwm4 = 3;
const int vcc1 = 22;
const int vcc2 = 23;
const int in_1 = 7 ;
const int in_2 = 6 ;
const int in_3 = 5 ;
const int in_4 = 4 ;
const int in_5 = 8 ;
const int in_6 = 9 ;
const int in_7 = 10;
const int in_8 = 11;

//For providing logic to L298 IC to choose the direction of the DC motor 

void setup()
{
//pinMode(pwm1,OUTPUT) ;   //we have to set PWM pin as output
//pinMode(pwm2,OUTPUT) ; 
pinMode(in_1,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_2,OUTPUT) ;
pinMode(in_3,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_4,OUTPUT) ;
pinMode(in_5,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_6,OUTPUT) ;
pinMode(in_7,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_8,OUTPUT) ;
pinMode(vcc1, OUTPUT);
pinMode(vcc2, OUTPUT);
}

void loop()
{
//For Clock wise motion , in_1 = High , in_2 = Low

digitalWrite(in_1,HIGH) ;
digitalWrite(in_2,LOW) ;
digitalWrite(in_3,HIGH) ;
digitalWrite(in_4,LOW) ;
digitalWrite(in_5,HIGH);
digitalWrite(in_6,LOW);
digitalWrite(in_7,HIGH);
digitalWrite(in_8,LOW);
digitalWrite(vcc1,HIGH);
digitalWrite(vcc2,HIGH);
//Clockwise for 3 secs
delay(3000) ;     

//For brake
digitalWrite(in_1,LOW) ;
digitalWrite(in_2,LOW) ;
digitalWrite(in_3,LOW) ;
digitalWrite(in_4,LOW) ;
digitalWrite(in_5,LOW) ;
digitalWrite(in_6,LOW) ;
digitalWrite(in_7,LOW) ;
digitalWrite(in_8,LOW) ;
delay(1000) ;

//For Anti Clock-wise motion - IN_1 = LOW , IN_2 = HIGH
digitalWrite(in_1,LOW) ;
digitalWrite(in_2,HIGH) ;

digitalWrite(in_3,LOW) ;
digitalWrite(in_4,HIGH) ;

digitalWrite(in_5,LOW);
digitalWrite(in_6,HIGH);

digitalWrite(in_7,LOW);
digitalWrite(in_8,HIGH);
delay(1000) ;

//For brake
digitalWrite(in_1,LOW) ;
digitalWrite(in_2,LOW) ;
digitalWrite(in_3,LOW) ;
digitalWrite(in_4,LOW) ;
digitalWrite(in_5,LOW) ;
digitalWrite(in_6,LOW) ;
digitalWrite(in_7,LOW) ;
digitalWrite(in_8,LOW) ;
delay(1000) ;
 }
