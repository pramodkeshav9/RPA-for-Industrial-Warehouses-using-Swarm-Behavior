#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
//int pos2 = 0;
void setup() {
  myservo1.attach(5); // attaches the servo on pin 9 to the servo object
  myservo2.attach(9);
  
  //myservo.writeMicroseconds(1500);
}

void loop() {
/*myservo.write(150);
delay(15);
myservo.write(0);
*/

  /*for (pos1= 60 && pos2=60; pos1<= 180 && pos2<=180; pos1+=1 && pos2+=1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
     myservo2.write(pos2);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  */
/*
 for(pos1 = 60; pos1<=180; pos1+=1)
 {
  for(pos2 = 60; pos2<=180; pos2+=1)
  {
    myservo1.write(pos1);
    myservo2.write(pos2);
  }
 }
*/

 for (pos = 90; pos <= 180; pos += 1) { // goes from 180 degrees to 0 degrees        
    myservo1.write(pos);
    myservo2.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  for (pos = 180; pos >= 90; pos -=1){ // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);
    delay(15);
  }

}
