

//Runs on AVR, STM32 etc.
//I2C communication on A4 = SDA  A5 = SCL  A3 = interrupt + pulse out to master
//A2 optional LED (active +)
//pwm LED on pin PD6
//PD0 PD1 Arduino serial programmer pins
//Keyboard drive and input pins : see arrays
//Two way i2c communication
//Slave on i2c address 10

#include <Wire.h>

  //================ Keyboard ===================

 byte port[]={A0,A1,2,3,4,5};     //Uno pin names for output

 byte pin[]={7,8,9,10,11,12,13};  //Input

 int code[]={0x08,0x10,0x20,0x40,0x80,0x100,0x200,0x09,0x11,0x21,0x41,0x81,0x101,
             0x0A,0x12,0x22,0x42,0x82,0x102,0x202,0x0B,0x13,0x23,0x43,0x83,0x103,
             0x0C,0x14,0x24,0x44,0x84,0x104,0x204,0x0D,0x15,0x25,0x45,0x85,0x105};

 char chars[]={'Q','W','E','R','T','Y','U','I','O','P','7','8','9',
               'A','S','D','F','G','H','J','K','L','#','4','5','6',
               'Z','X','C','V','B','N','M',' ','-','0','1','2','3'};
 char lastchar;

 byte slaveAdr = 10, bright;

// function that executes whenever data is requested by master

void requestEvent()  { 
  Wire.write(lastchar); 
 }

// function that executes whenever data is sent by master

 void receiveEvent(int i) {
  i=1;  int x;
  while(Wire.available()) bright = Wire.read();    // receive byte
  analogWrite(6,bright);                          //Master controls keyboard backlight
}

void setup() {

//Serial.begin(9600); Serial.println("Keyboard");

for (byte i=0; i<7; i++) pinMode (pin[i],INPUT_PULLUP);

for (byte i=0; i<6; i++) pinMode (port[i],OUTPUT);

pinMode(A2, OUTPUT); digitalWrite(A2, LOW); //interrupt pin,  active high
pinMode(6,OUTPUT);                          //PWM LED driver
bright = 127;
analogWrite(6,bright);

  // Start the I2C Bus as Slave on address

  Wire.begin(slaveAdr); Wire.onRequest(requestEvent);
                        Wire.onReceive(receiveEvent); // register event
}


void loop() {

int s = scan(),i;   //Serial.println(s,HEX);  //debug

if (s) {           //process key code, search array

  // end search if found in table, i shows position

  for ( i=0; i<39; i++ ) if (s == code[i]) break;

  lastchar=chars[i];  //Serial.print(lastchar);  //debug

  while (scan());  //wait for key release, no repeat

  digitalWrite(A2, HIGH );
  digitalWrite(A2, LOW);  //pulse interrupt pin

  delay(100);
}  // key processed

}


unsigned int scan(void) {

  unsigned int x, y=0;

  // Reset drive pins to High level

  for (byte j=0; j<6; j++) digitalWrite(port[j],HIGH);

  // Pull down drive pins one by one and check response

  for (byte j=0; j<6; j++) {

  digitalWrite(port[j],LOW);

  byte mult=1; x=0;

  // Sum up inputs in variable x

  for (byte k=0; k<7; k++) { x+= mult*digitalRead(pin[k]); mult=mult*2; }

  x = 0x7f - x ;   // Result is 0x7F if no key pressed

  x *= 8;  // Make room for key position (variable j) on low 3 bits

  digitalWrite(port[j],HIGH);

  if (x)  y = x + j ;    // key pressed if x > 0, save position in y

  }                              //all 6 key blocks scanned
            return(y);
  }

//============= Receiver code example ==================

/*

// Runs on Arduino Uno
// I2C communication on A4 = SDA  A5 = SCL  A2 = interrupt pin on master and slave
// Two way data transfer, master sends 1/sec random backlight LED brightness data
#include <Wire.h>

String text;
bool flag= false ;
byte x;

void setup() {
  Serial.begin(9600);

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(2, INPUT); attachInterrupt(0, handle, RISING);
  Wire.begin();
 }

void loop() {

  char data;

  if (flag) {   //interrupt flag set

  flag=false  ;  //reset interrupt request

  Wire.requestFrom(10, 1);

  // digitalWrite(13,HIGH); delay(50); digitalWrite(13, LOW); delay
(50); //pulse LED, char received

  while(Wire.available())     // slave may send less/more than requested

  { data = Wire.read();  if (data) Serial.print(data); }  //get
character from keyboard

  //Process special characters : #  Enter, - Delete ,  or add to string

  if (data =='-') {text = text.substring(0, text.length()-1);
Serial.println(); Serial.println(text); }

  else  if (data =='#') { Serial.println(); Serial.println(text);
text = ""; }  //string + enter

  else  text += data ;

  }  //interrupt processed
// Random backlight to test the communication
  if (millis() % 1000 == 0) {
    Wire.beginTransmission(10); // transmit to device #10
    Wire.write(random(255));    // sends one byte
    Wire.endTransmission();    // stop transmitting
    }

  }

void handle(void) {
  flag = true;
}*/
