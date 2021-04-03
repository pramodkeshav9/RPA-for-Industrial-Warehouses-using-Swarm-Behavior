
 
// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 280
 
// Encoder output to Arduino Interrupt pin
#define ENC_IN1 3
#define ENC_IN2 2 
 

 

// Analog pin for potentiometer
//int speedcontrol = 0;
const int in_1 = 5
const int in_2 = 4 ;
const int in_3 = 7;
const int in_4 = 6;
const int vcc1 = 22;
const int vcc2 = 23;
// Pulse count from encoder
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
 
// One-second interval for measurements
//int interval = 1000;
 
// Counters for milliseconds during interval
//long previousMillis = 0;
//long currentMillis = 0;
 
// Variable for RPM measuerment/
//int rpm = 0;
 
// Variable for PWM motor speed output
int motorPwm = 0;
 
void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600); 
   pinMode(vcc1, OUTPUT);
   pinMode(vcc2, OUTPUT);
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN1, INPUT_PULLUP); 
  pinMode(ENC_IN2, INPUT_PULLUP);
  // Set PWM and DIR connections as outputs
  pinMode(in_1, OUTPUT);
  pinMode(in_2,OUTPUT) ;
  pinMode(in_3,OUTPUT);
  pinMode(in_4,OUTPUT);
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN1), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), updateEncoder2, RISING);
  
  
  // Setup initial values for timer
  //previousMillis = millis();
}
 
void loop()
{
  
    // Control motor with potentiometer
    //motorPwm = map(analogRead(speedcontrol), 0, 1023, 0, 255);
    /*if(motorPwm < 255)
    {
      MotorPwm=MotorPwm+30;
    }
    else
    {
      MotorPwm=MotorPwm-30;
    }*/
    
    // Write PWM to controller
    digitalWrite(in_1, HIGH);
    digitalWrite(in_2,LOW);
    digitalWrite(in_3,HIGH);
    digitalWrite(in_4,LOW);
  
  // Update RPM value every second
  //currentMillis = millis();
  //if (currentMillis - previousMillis > interval) {
    //previousMillis = currentMillis;
 
 
    // Calculate RPM
    //rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
 
    // Only update display when there is a reading
     {
      //Serial.print("PWM VALUE: ");
      //Serial.print(motorPwm);
     
              
        Serial.print(" PULSES: ");
        Serial.print(encoderValue1);
        Serial.println("--------------");
        Serial.println(encoderValue2);
        Serial.println("--------------");
        
    
    
    
  }
}
 
void updateEncoder1()
{
  // Increment value for each pulse from encoder
  encoderValue1++;
}

void updateEncoder2()
{
  encoderValue2++;
}
