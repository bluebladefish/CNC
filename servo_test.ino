

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

//int pos = 0;    // variable to store the servo position
#define MAX_BUF (64) // What is the longest message Arduino can store?
char buffer[MAX_BUF]; // where we store the serial input message until we get a ';'
int sofar = 0; // how much is in the serial buffer
String serealINmessage = " " ; // current message being processed


void setup() {
  myservo.attach(11);  // attaches the servo on pin 9 to the servo object
  myservo.write(155);
    // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  Serial.println("set servo value 0 to 180");

}

void loop() {
  listenToSerial();
}

void LaserPower (int mypower){
  myservo.write(mypower);
  Serial.print("servo at : ");
  Serial.println(mypower);
}

void listenToSerial() {
  if( Serial.available() ) { // if something is available over USB
    char MyChar = Serial.read(); // get it
    //Serial.print(MyChar); // optional: repeat back what I got for debugging
    if(sofar < MAX_BUF) {
      buffer[sofar++]=MyChar;
    }
    // if we got a return character (\n) the message is done.
    if(MyChar=='\n') {
      //Serial.print(F("\r\n")); // optional: send back a return for debugging
      // strings must end with a \0.
      buffer[sofar]=0;
      char *ptr=buffer;
      int MyCueerntChar = 0;
      serealINmessage = "";
      while(MyCueerntChar <=sofar) {
        serealINmessage = (serealINmessage + ptr[MyCueerntChar]);
        MyCueerntChar++;
      }
          
     Serial.print("serealINmessage=");
     Serial.println(serealINmessage);
     LaserPower(serealINmessage.toInt());
     readyforSerial();
     }
     // processCommand(); // do something with the command from USB
      //readyforSerial(); 
 }
}

// ready for input from Pi
void readyforSerial() { 
  sofar=0; // clear input buffer
  Serial.print(F("> ")); // signal ready to receive input
  // Serial1.print(F("> ")); // signal ready to receive input
}
