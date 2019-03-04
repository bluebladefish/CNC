//------------------------------------------------------------------------------
// My 2 Axis CNC - BlueBlade
// 01-12-2019
//------------------------------------------------------------------------------
// v1.0 basic build, configured for servo powered pen.
// v1.1 re-configured for laser, step tuning and arc modification. renamed atan3 to BBatan, to reduce confusion with other atan3 math functions 
// v1.2 added extra buttons, "monitorbuttons" function, and converted buttons and endstops to digital read

//#include "Arduino.h"      //core arduino functions
//#include <Servo.h>          //if using servo write instead of laser
//Servo myservo;  // create servo object to control a servo

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#define VERSION        (1.2)  // firmware version
#define BAUD           (115200)  // How fast is the Arduino talking?
#define MAX_BUF        (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN (4076)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (1200.0) // in microsecands
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.1)
#define XstpMM   (110.6) // Steps per MM on Y axis (17000 = 156 mm = Max width)
#define YstpMM   (98) // steps per MM on X azis (22000 = 225 mm = Max length)
// for arc directions
#define ARC_CW          (-1)
#define ARC_CCW         (1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (315) // actually steps per segment (approx 104 stepps per mm) small numbers cause low angle accuracy (correction lines), large cause faceted curves

// Motor pin definitions
#define Pin0  3     // IN1 on the ULN2003 driver 1
#define Pin1  4     // IN2 on the ULN2003 driver 1
#define Pin2  5     // IN3 on the ULN2003 driver 1
#define Pin3  6     // IN4 on the ULN2003 driver 1
#define Pin0b  7     // IN1 on the ULN2003 driver 2
#define Pin1b  8     // IN2 on the ULN2003 driver 2
#define Pin2b  9     // IN3 on the ULN2003 driver 2
#define Pin3b  10     // IN4 on the ULN2003 driver 2
// servo/laser
#define writepin  11
// endstops (all analoge pins can be used as digital except A6 and A7 on the nano)
#define Xend  A4
#define Yend  A5
// buttons
#define but1  A0
#define but2  A1
#define but3  A2


// other settings
char  serialbuffer[MAX_BUF];  // where we store the message until we get a newline
int   sofar;            // how much is in the buffer
float px, py;      // location

// speeds
float fr = 0;  // human version
long  step_delay;  // machine version

// settings
char mode_abs=1;   // absolute mode?
int stepper1= 0;// track the current step for stepper 1
int stepper2= 0;// track the current step for stepper 2
int laseron= 105; //lowers servo/activates laser
int laseroff= 155; //raises servo/de-activates laser
int laserlevel= 155; // saves current laser state
//------------------------------------------------------------------------------
// Program
//------------------------------------------------------------------------------


// First thing this machine does on startup.  Runs only once.
void setup() {
  // stepper pins
  pinMode(Pin0, OUTPUT);
  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);
  pinMode(Pin3, OUTPUT);
  pinMode(Pin0b, OUTPUT);
  pinMode(Pin1b, OUTPUT);
  pinMode(Pin2b, OUTPUT);
  pinMode(Pin3b, OUTPUT);
  // button pins
  pinMode(but1, INPUT);
  pinMode(but2, INPUT);
  pinMode(but3, INPUT);
  // endstops
  pinMode(Xend, INPUT);
  pinMode(Yend, INPUT);
  // if using laser
  pinMode(writepin, OUTPUT); // for laser
  digitalWrite(writepin, LOW); // laser off at start
  // if using servo for write:
  //myservo.attach(11);  // attaches the servo on pin 9 to the servo object
  //myservo.write(laseroff);  // lift servo (not writing)
  //
  
  Serial.begin(BAUD);  // open coms
  feedrate(((MAX_FEEDRATE * 3) + MIN_FEEDRATE)/4);  // set default speed
  setup_controller();  //setup_controller (not yet defined) (move to endstops and set as 0,0)
  //myposition(0,0);  // set staring position
  help();  // say hello
  ready();
}


// After setup() this machine will repeat loop() forever.
void loop() {
  monitorserial(); // listen for serial commands
  monitorbuttons(); // check for button push
}


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

// listen to serial port for input:
void monitorserial(){
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) serialbuffer[sofar++]=c;  // store it
    if((c=='\n') || (c == '\r')) {
      // entire message received
      serialbuffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}

// prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F("ok"));  // signal ready to receive input
}

// Read the input buffer and find any recognized commands.  One G or M command per line.
void processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:{ // fast line move - no write
    feedrate(MAX_FEEDRATE);
    // need to add "S##" commands for spindle speed/laser power for now just turn on for 1 and off for 0.
    LaserPower(laseroff); //raise servo - do notwrite
    line( ((parsenumber('X',(mode_abs?(px/XstpMM):0))* XstpMM) + (mode_abs?0:px)),
          ((parsenumber('Y',(mode_abs?(py/YstpMM):0))* YstpMM) + (mode_abs?0:py)) ); 
    break;
    }
  case  1: { // line write
    feedrate(parsenumber('F',fr));
    // need to add "S##" commands for spindle speed/laser power for now just turn on for 1 and off for 0.
    LaserPower(laseron); //drop servo - write
    line( ((parsenumber('X',(mode_abs?(px/XstpMM):0))* XstpMM) + (mode_abs?0:px)),
          ((parsenumber('Y',(mode_abs?(py/YstpMM):0))* YstpMM) + (mode_abs?0:py)) ); 
    break;
    }
  case 2: {  // arc clockwise
      feedrate(parsenumber('F',fr));
      // need to add "S##" commands for spindle speed/laser power for now just turn on for 1 and off for 0.
      LaserPower(laseron); //drop servo - write
      arc(((parsenumber('I',0)* XstpMM) + px), // I and J should be relative offset always
          ((parsenumber('J',0)* YstpMM) + py), // I and J should be relative offset always
          ((parsenumber('X',(mode_abs?(px/XstpMM):0))* XstpMM) + (mode_abs?0:px)),
          ((parsenumber('Y',(mode_abs?(py/YstpMM):0))* YstpMM) + (mode_abs?0:py)),
          ARC_CW);
      break;
    }
  case 3: {  // arc Counter clockwise
      feedrate(parsenumber('F',fr));
      // need to add "S##" commands for spindle speed/laser power for now just turn on for 1 and off for 0.
      LaserPower(laseron); //drop servo - write
      arc(((parsenumber('I',0)* XstpMM) + px), // I and J should be relative offset always
          ((parsenumber('J',0)* YstpMM) + py), // I and J should be relative offset always
          ((parsenumber('X',(mode_abs?(px/XstpMM):0))* XstpMM) + (mode_abs?0:px)),
          ((parsenumber('Y',(mode_abs?(py/YstpMM):0))* YstpMM) + (mode_abs?0:py)),
          ARC_CCW);
      break;
    }
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    myposition( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  default:  break;
  }
  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}

// monitor and react to console buttons
void monitorbuttons (){
  int thisB1 = digitalRead(but1);
  int thisB2 = digitalRead(but2);
  int thisB3 = digitalRead(but3);
  if (thisB1 == HIGH){ // button one pressed (move deck forward to allow paper load)
    LaserPower(laseroff); //raise servo - do notwrite
    feedrate(MAX_FEEDRATE);
    line((175*XstpMM),(120*YstpMM)); 
  } else if (thisB2 == HIGH){ // button Two pressed (re-home - set 0,0)
    LaserPower(laseroff); //raise servo - do notwrite
    setup_controller();
  } else if (thisB3 == HIGH){ // button Three pressed (position laser to allow focusing, turn laser on)
    LaserPower(laseroff); //raise servo - do notwrite
    feedrate(MAX_FEEDRATE);
    line((90*XstpMM),(120*YstpMM));
    LaserPower(laseron); //drop servo - write
  }
}


// display helpful information
void help() {
  Serial.print(F("GcodeCNC Version"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}

// print the current position, feedrate, and absolute mode.
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=serialbuffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)serialbuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}

// returns angle of dy/dx as a value from 0...2PI
float BBatan(float thisdy,float thisdx) {
  float a = atan2(thisdy,thisdx);
  if(a<0) a = (PI*2.0)+a;
  return a;
}

// stepper code:
void m1step(int dir){// stepper motor 1 (X) activate
  //Serial.print("M1 move 1 step in dir=");
  //Serial.print(dir);
  //Serial.print(" delay=");
  //Serial.println(step_delay);
  if (dir > 0 ){ // std direction for stepper 1
    if (stepper1 < 7){
      stepper1++;
    } else {
      stepper1 = 0;
    }
  } else {
    if (stepper1 > 0){
      stepper1--;
    } else {
      stepper1 = 7;
    }
  }
 switch(stepper1){
 case 0:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, HIGH);
 break;
 case 1:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, HIGH);
 digitalWrite(Pin3, HIGH);
 break;
 case 2:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, HIGH);
 digitalWrite(Pin3, LOW);
 break;
 case 3:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, HIGH);
 digitalWrite(Pin2, HIGH);
 digitalWrite(Pin3, LOW);
 break;
 case 4:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, HIGH);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, LOW);
 break;
 case 5:
 digitalWrite(Pin0, HIGH);
 digitalWrite(Pin1, HIGH);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, LOW);
 break;
 case 6:
 digitalWrite(Pin0, HIGH);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, LOW);
 break;
 case 7:
 digitalWrite(Pin0, HIGH);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, HIGH);
 break;
 default:
 digitalWrite(Pin0, LOW);
 digitalWrite(Pin1, LOW);
 digitalWrite(Pin2, LOW);
 digitalWrite(Pin3, LOW);
 break;
 }
}

void m2step(int dir){// stepper motor 2 (y) activate
  //Serial.print("M1 move 1 step in dir=");
  //Serial.print(dir);
  //Serial.print(" delay=");
  //Serial.println(step_delay);
  if (dir > 0 ){ // std direction for stepper 2
    if (stepper2 < 7){
      stepper2++;
    } else {
      stepper2 = 0;
    }
  } else {
    if (stepper2 > 0){
      stepper2--;
    } else {
      stepper2 = 7;
    }
  }
 switch(stepper2){
 case 0:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, HIGH);
 break;
 case 1:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, HIGH);
 digitalWrite(Pin3b, HIGH);
 break;
 case 2:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, HIGH);
 digitalWrite(Pin3b, LOW);
 break;
 case 3:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, HIGH);
 digitalWrite(Pin2b, HIGH);
 digitalWrite(Pin3b, LOW);
 break;
 case 4:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, HIGH);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, LOW);
 break;
 case 5:
 digitalWrite(Pin0b, HIGH);
 digitalWrite(Pin1b, HIGH);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, LOW);
 break;
 case 6:
 digitalWrite(Pin0b, HIGH);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, LOW);
 break;
 case 7:
 digitalWrite(Pin0b, HIGH);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, HIGH);
 break;
 default:
 digitalWrite(Pin0b, LOW);
 digitalWrite(Pin1b, LOW);
 digitalWrite(Pin2b, LOW);
 digitalWrite(Pin3b, LOW);
 break;
 }
}


// delay for the appropriate number of microseconds * @input ms how many milliseconds to wait
void pause(long ms) {
  //delay(ms);
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}

// Set the feedrate (speed motors will move) * @input nfr the new speed in steps/second
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.
  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  if (step_delay < MIN_STEP_DELAY){
    step_delay = MIN_STEP_DELAY;
  }
  fr = nfr;
}

void LaserPower (int mypower){
  if (laserlevel != mypower){
    // for servo:
    //pause(2000); //allow the servo time to move
    //myservo.write(mypower); // if using servo write
    //pause(1200); //allow the servo time to move
    //laserlevel = mypower;
    // for laser:
    if (mypower == laseron){
      digitalWrite(writepin, HIGH); // turn laser on
      laserlevel = mypower;
    } else {
      digitalWrite(writepin, LOW); // turn laser off
      laserlevel = mypower;
    }
  //Serial.print("servo at : ");
  //Serial.println(mypower);
  }
}


//move to enstop on x and y and stop, set the position to x=0 y=0
void setup_controller(){
  int XsensorValue = digitalRead(Xend);
  int YsensorValue = digitalRead(Yend);
  while (XsensorValue != HIGH){
    m1step(-1);
    pause(step_delay);
    XsensorValue = digitalRead(Xend);
  }
  while (YsensorValue != HIGH){
    m2step(1);
    pause(step_delay);
    YsensorValue = digitalRead(Yend);
  }
  px = 0;
  py = 0;
}

// code to review and modify
//------------------------------------------------------------------------------  
// need work:

void disable(){}

/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  long i;
  long over= 0;
  long dx  = newx-px;
  long dy  = newy-py;
  int dirx = dx>0?1:-1;
  int diry = dy>0?-1:1;  // because the motors are mounted in opposite directions
  dx = abs(dx);
  dy = abs(dy);
  if(dx>dy) {
    over = dx/2;
    for(i=0; i<dx; ++i) {
      m1step(dirx);
      over += dy;
      if(over>=dx) {
        over -= dx;
        m2step(diry);
      }
      pause(step_delay);
    }
  } else {
    over = dy/2;
    for(i=0; i<dy; ++i) {
      m2step(diry);
      over += dx;
      if(over >= dy) {
        over -= dy;
        m1step(dirx);
      }
      pause(step_delay);
    }
  }
  px = newx;
  py = newy;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle (I, J)
// x/y - end position (X, Y)
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt((dx*dx)+(dy*dy));
  // find angle of arc (sweep)
  float angle1=BBatan(dy,dx);
  float angle2=BBatan(y-cy,x-cx);
  float theta=angle2-angle1;
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  theta=angle2-angle1;
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;
  int i, segments = ceil( len / MM_PER_SEGMENT );
  float nx, ny, angle3, scale;
  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
     line(nx,ny);
  }
  //Serial.println("drwing final line");
  line(x,y);
}




//------------- debugging --------------------
/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}

/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void myposition(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}
