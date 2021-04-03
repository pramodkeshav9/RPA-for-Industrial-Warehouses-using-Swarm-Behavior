//#define MOTOR_EN_L    13
#define MOTOR_IN1_L   5   
#define MOTOR_IN2_L   4

//#define MOTOR_EN_R    19
#define MOTOR_IN1_R   8
#define MOTOR_IN2_R   9

#define ENCODER_L     13
#define ENCODER_R     19

#define ENCODEROUTPUT 280

//The sample code for driving one way motor encoder
volatile long encoderValue = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

int rpm = 0;
boolean measureRpm = false;
int motorPwm = 0;

void setup() {

  Serial.begin(115200);//Initialize the serial port
  EncoderInit();//Initialize the module
  
   //pinMode( MOTOR_EN_L , OUTPUT);
   pinMode( MOTOR_IN1_L , OUTPUT);
   pinMode( MOTOR_IN2_L , OUTPUT);
   pinMode(ENCODER_L, INPUT);

   //digitalWrite( MOTOR_EN_L , HIGH);
   digitalWrite(MOTOR_IN1_L,HIGH);
   digitalWrite(MOTOR_IN2_L,LOW);

  pinMode( MOTOR_IN1_R , OUTPUT);
   pinMode( MOTOR_IN2_R , OUTPUT);
   pinMode(ENCODER_R, INPUT);
   //digitalWrite( MOTOR_EN_R , HIGH);
   digitalWrite(MOTOR_IN1_R,HIGH);
   digitalWrite(MOTOR_IN2_R,LOW);

   encoderValue = 0;
   previousMillis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(2000);
 
  
  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    Serial.println(encoderValue);

    // Revolutions per minute (RPM) =
    // (total encoder pulse in 1s / motor encoder output) x 60s
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);
    Serial.println(rpm);
    // Only update display when there have readings
    if ( rpm > 0) {
      

      Serial.println(encoderValue);
      Serial.println(" pulse / ");
      Serial.println(ENCODEROUTPUT);
      Serial.println(" pulse per rotation x 60 seconds = ");
      Serial.println(rpm);
      Serial.println(" RPM");
    }
    
    encoderValue = 0;
  }

}

void EncoderInit()
{
 // Attach interrupt at hall sensor A on each rising signal
  Serial.println("interrupt");
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), updateEncoder, FALLING);
}


void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}
