// time set to 23:22:00 on 4/29/2019
// and then I programmed it a few times so reset that experiment
// 6-digit IN-17 nixie tube clock
/* clock operational features
 *  state variable tracks what to display
 *   time
 *   date
 *   menu value (setting mode)
 *  button functions
 *   implement short press and long press
 *   one button for select option
 *   one button for increment
 *   one button for decrement
 *   short press on select will go to next, with the final item in the menu being exit up 1 level
 *   long press on select will store current setting and exit entirely
 *  menu help text will print on serial
 *   everything may be set with serial as well
 *  serial display option
 *   may enter a mode where 6 numbers and blank may be displayed
 *  animations to keep cathodes from poisoning
 *   slide through physically adjacent digits
 *   set to blank and shift time in from left or right
 *   set to blank and increment each digit to its value in numerical sequence
 *   set to blank and increment each digit to its value in physical sequence
 *  get time from RTC
 *  set time to RTC
 *  set backlight LED colors (all PWM, all the same color)
 *  countdown to instant, countup from instant
 *  countdown from initial duration
 *  tube display (main loop, always running)
 *   based on mode, determine digits to display
 *   start with unknown digit setting, all digits off
 *   set digit with decoder
 *   enable digit (if not blank)
 *   wait specified on time
 *   disable digit
 *   wait specified off time
 *   repeat for each digit
 *  interrupts
 *   clock IRQ
 *   clock sqw output (possibly faster than 1 tick per second, to take advantage of interrupts for display
 *    increment time, does not require reading clock registers
 *    read all registers on a minute 00, and 1+ seconds into the minute to be correct
 *    perform animation on a minute 00 (or other user set value perhaps)
 *    set LEDs to their time-dependent value (ex blinking on seconds, blinking or colors in menu)
 *  list of user settable values
 *   red LED pwm value
 *   green LED pwm value
 *   blue LED pwm value
 *   diaplay brightness (pwm-like on-time, auto off-time)
 *   animation interval in hh:mm:ss
 *   colons enabled or disabled
 *   alarm enabled or disabled (RTC interrupt or external?)
 *   enable or disable the insanity second tick
 */

/* List of RTC Registers: M41T62
 *  http://www.st.com/content/ccc/resource/technical/document/datasheet/c8/66/38/71/c4/ac/49/9e/CD00019860.pdf/files/CD00019860.pdf/jcr:content/translations/en.CD00019860.pdf
 *  Time values are BCD
 *  addr    bit7  bit6  bit5  bit4  bit3  bit2  bit1  bit0    Name      Format    Notes
 *  0x00    0.1 seconds #     #     0.01 seconds#     #       ths       0-99      BCD
 *  0x01    ST    10 seconds  #     Seconds     #     #       Seconds   0-59      BCD, ST = stop bit
 *  0x02    OFIE  10 minutes  #     minutes     #     #       Minutes   0-59      BCD
 *  0x03    0     0     10 hours    hours(24-hour)    #       Hours     0-23      BCD
 *  0x04    RS3   RS2   RS1   RS0   0     Day of week #       day       1-7       BCD, RS=sqw frequency bits
 *  0x05    0     0     10 date     date: day of month#       date      1-31      BCD
 *  0x06    CB1   CB0   0   10M     month #     #     #       month     1-12      BCD, CB=century 0-3
 *  0x07    10 years    #     #     year  #     #     #       year      0-99      BCD
 *  0x08    OUT   0     S     Calibration #     #     #       calibration         OUT=output level, S=sign bit
 *  0x09    RB2   BMB4  BMB3  BMB2  BMB1  BMB0  RB1   RB0     watchdog  RB=resolution, BMB=multiplier
 *  0x0A    AFE   SQWE  0     10M   Alarm Month #     #       Al Month  1-12      BCD, AFE=al flag enable
 *  0x0B    RPT4  RPT5  AL 10 date  Alarm Date  #     #       Al Date   1-31      BCD, RPT1-5=repeat mode
 *  0x0C    RPT3  0     Al 10 hour  Alarm hour  #     #       Al Hour   0-23      BCD
 *  0x0D    RPT2  Al 10 mins  #     Alarm Mins  #     #       Al Mins   0-59      BCD
 *  0x0E    RPT1  Al 10 secs  #     Alarm Secs  #     #       Al Secs   0-59      BCD
 *  0x0F    WDF   AF    0     0     0     OF    0     0       Flags     WDF=watchdog (ROM), AF=Alarm, OF=osc fail
 *  
 *  enable the RTC squarewave output and measure TP19 to assist in calibration.  it should be 512Hz
 *  or compare the clock every month to a known reference (GPS time for example)
 *  drift is correctable within +5.5 to -2.75 minutes per month, or +10.7 to -5.35 seconds per day
 *  the number of times I re-start the oscillator may have more of an effect on the accuracy of the clock than the drift of the oscillator.  
 *  Maybe only kick-start the oscillator if the OF flag is set, but otherwise not.  
 */

// Uncomment this define to enable debug printing
// #define __DEBUG__

//LEDs on digital pins
#define PWM_GRN 11
#define PWM_RED 6
#define PWM_BLU 5
#define RX_LED 17
#define TX_LED 30
#define USER_LED 13

//buttons on digital pins
#define BUTTON_1_PIN 7
#define BUTTON_2_PIN 15
#define BUTTON_3_PIN 9

//tube anode control on digital pins
#define AnoA A4
#define AnoB A2
#define AnoC A3
#define AnoD A5
#define AnoE A1
#define AnoF A0

//tube cathode bits on digital pins
#define cbit0 12
#define cbit1 16
#define cbit2 14
#define cbit3 8

//RTC interrupts on digital pins
#define IRQ 0
#define SQW 1

//high voltage feedback on analog pin
#define FB 6

#define RTC_ADDR 0xD0

#include <Wire.h>
#include <EEPROM.h>

// like #define for arrays
const byte cbits[] = {cbit0,cbit1,cbit2,cbit3};
const byte AnoPins[] = {AnoA,AnoB,AnoC,AnoD,AnoE,AnoF};
const byte cathodeConvert[] = {6, 8, 5, 9, 4, 3, 7, 1, 2, 0, 15};
const byte dowKey[] = {1, 4, 4, 0, 2, 5, 0, 3, 6, 1, 4, 6};   // day of week key
const byte BTN[] = {BUTTON_1_PIN, BUTTON_2_PIN, BUTTON_3_PIN};

// globals
bool divchar = false; // used in serial parse function, flag for divider character
bool OF = false;  // oscillator fault flag, read on RTC check; reset after resetting time
bool checkFlag = false; // flag to check the rtc; set on an SQW interrupt in the toggle function
bool frameFlag = false; // frame flag used to progress animations by frame count
byte frameCount = 0;  // frame counter used to slow down animations
bool startup = true; // flag used to determine if the device was just reset
bool override_display = false; // allows user to inhibit the rtc from updating the digits
String tempstring = " "; // make this pass by reference and local to the serial functions
unsigned long dispVal = 000000;  // the value to show on the display, determined by state
byte dispBlank = 0x00;  // mask indicating blanks when paired with dispval; 1 is off; BCD, 6-bits
byte dispDigs[] = {0, 0, 0, 0, 0, 0}; // anything > 9 is off; position 0 is leftmost; position 2 is rightmost
byte rtcRegs[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //local copy of the RTC registers, to minimize I2C traffic time
unsigned int long_count = 1000; // milliseconds for long press detection

// non-volatile variables
// use EEPROM.update(address,value) to write only if the value changed, to maximize endurance
bool leadingZeroOff = false;
byte redVal = 0;
byte grnVal = 0;
byte bluVal = 0;
unsigned char onTime = 50; // tube on time, in microseconds
unsigned char period = 100; // maximum duration of a digit illumination, in microseconds
unsigned int deadtime = 1; // time off between digits, in microseconds

// state variables
bool pressed[] = {false, false, false};
bool short_press[] = {false, false, false};
bool long_press[] = {false, false, false};
int press_count[] = {0, 0, 0};
byte state = 0;   // used to determine button actions


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // this is fast to avoid slowing down the display when interrupted
  Wire.begin();
  Wire.setClock(400000); // this is fast to avoid slowing down the display when interrupted
  
  // Enable the RGB LED
  digitalWrite(PWM_GRN, LOW);
  digitalWrite(PWM_RED, LOW);
  digitalWrite(PWM_BLU, LOW);
  pinMode(PWM_GRN, OUTPUT);
  pinMode(PWM_RED, OUTPUT);
  pinMode(PWM_BLU, OUTPUT);

  // Enable the other LEDs
  digitalWrite(USER_LED, LOW);  //USER LED
  digitalWrite(RX_LED, HIGH); //RXLED
  digitalWrite(TX_LED, HIGH); //TXLED
  pinMode(USER_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);
  pinMode(TX_LED, OUTPUT);

  // Enable the three buttons on the side as inputs
  pinMode(BUTTON_1_PIN, INPUT);
  pinMode(BUTTON_2_PIN, INPUT);
  pinMode(BUTTON_3_PIN, INPUT);

  // Configure all Nixie Tube Annode Pins
  digitalWrite(AnoA, LOW);
  digitalWrite(AnoB, LOW);
  digitalWrite(AnoC, LOW);
  digitalWrite(AnoD, LOW);
  digitalWrite(AnoE, LOW);
  digitalWrite(AnoF, LOW);
  pinMode(AnoA, OUTPUT);
  pinMode(AnoB, OUTPUT);
  pinMode(AnoC, OUTPUT);
  pinMode(AnoD, OUTPUT);
  pinMode(AnoE, OUTPUT);
  pinMode(AnoF, OUTPUT);

  // Configure all Nixie Tube Cathode Decoder Input Pins
  digitalWrite(cbit0, LOW);
  digitalWrite(cbit1, LOW);
  digitalWrite(cbit2, LOW);
  digitalWrite(cbit3, LOW);
  pinMode(cbit0, OUTPUT);
  pinMode(cbit1, OUTPUT);
  pinMode(cbit2, OUTPUT);
  pinMode(cbit3, OUTPUT);

  //RTCinit();
  //setTime(13,25,36);
  //setDate(9,12,19);

  digitalWrite(IRQ, HIGH);  // open drain pin missing its pullup resistor...
  pinMode(IRQ, INPUT);
  // check that the RTC is in a good state before attaching the interrupt.  
  attachInterrupt(digitalPinToInterrupt(SQW), toggle1, RISING);
}

void loop(){
  if (Serial.available()>0) serialMenu();
  if (checkFlag) checkrtc();
  if (frameFlag){
    if (startup) animate_startup();
    /*// LED breathing
    if (((rtcRegs[1]%2) > 0) && bluVal < 255) bluVal++;
    else if (bluVal > 0) bluVal--;
    setLed(redVal, grnVal, bluVal);
    //*/
    frameFlag = false;
  }
  
  buttonFuncs();
  //state_machine();
  // if buttons trigger a menu, override the normal time display with setDisplay()
  
  disp_main();
  frameFlag = true;
}

void state_machine(){
  // given inputs, determine what to display, actions on button presses, and 
  
  if (long_press[1] && !pressed[1]){
    if (state == 0) state = 1;
    else if (state == 1) state = 2;
    else state = 0;
    long_press[1] = false;
  }
  if (short_press[2]){
    if (state == 0) redVal+=8;
    else if (state == 1) bluVal+=8;
    else if (state == 2) grnVal+=8;
    short_press[2] = 0;
    setLed(redVal, grnVal, bluVal);
  }
  if (short_press[0]){
    if (state == 0) redVal-=8;
    else if (state == 1) bluVal-=8;
    else if (state == 2) grnVal-=8;
    short_press[0] = 0;
    setLed(redVal, grnVal, bluVal);
  }
}

void disp_main(){
  // display
  // loop through tubes
  for (int i=0; i<6; i++){
    //set cathode bits
    for (int j=0; j<4; j++){ 
      if ((cathodeConvert[dispDigs[i]] & 0x01<<j) > 0)
        digitalWrite(cbits[j], HIGH);
      else
        digitalWrite(cbits[j], LOW);
    }
    //enable anode if needed
    if(dispDigs[i]<10) digitalWrite(AnoPins[i], HIGH);
    else digitalWrite(AnoPins[i], LOW);
    //wait on-time
    delayMicroseconds(onTime+1); //never 0
    //disable anode
    digitalWrite(AnoPins[i], 0);
    //wait off-time
    if (period > onTime) delayMicroseconds(period-onTime); //never 0
    else delayMicroseconds(20);
    delayMicroseconds(deadtime);
  }
}

void animate_startup(){
  frameCount++;
  if(frameCount >= 200){
    frameCount = 0;
    switch (dispDigs[0]){
      case 0: setDisplay(111111,0); break;
      case 1: setDisplay(222222,0); break;
      case 2: setDisplay(333333,0); break;
      case 3: setDisplay(444444,0); break;
      case 4: setDisplay(555555,0); break;
      case 5: setDisplay(666666,0); break;
      case 6: setDisplay(777777,0); break;
      case 7: setDisplay(888888,0); break;
      case 8: setDisplay(999999,0); break;
      case 9: setDisplay(000000,63); startup = false; break;
      default: setDisplay(111111,12); break;
    }
  }
}

void buttonFuncs(){  
  // check all buttons for state updates
  for(int key=0; key<3; key++){
    // initialize press count to 0 if there is a new press detected
    if(!pressed[key] && !digitalRead(BTN[key])){
      pressed[key] = true;
      short_press[key] = false;
      long_press[key] = false;
      press_count[key] = 0;
    }
    if(pressed[key] && !long_press[key]){
      press_count[key]++;
      // if we reach the count, it's a long press
      if (press_count[key] >= long_count){
        long_press[key] = true;
        short_press[key] = false;
        if (Serial) {
          Serial.print(F("long press "));
          Serial.println(key);
        }
        digitalWrite(17, LOW); //RXLED on
      }
      // but if we release without reaching the count, it's a short press
      else if (digitalRead(BTN[key])){
        short_press[key] = true;
        long_press[key] = false;
        pressed[key] = false;
        if (Serial) {
          Serial.print(F("short press "));
          Serial.println(key);
        }
      }
    }
    // check if it has been released from a long press
    else if(long_press[key] && digitalRead(BTN[key]) && pressed[key]){
      pressed[key] = false;
      if (Serial){
        Serial.print(F("long release "));
        Serial.println(key);
      }
      digitalWrite(17, HIGH); //RXLED off
    }
  }


  
}

void toggle1(){  // ISR to swap the state of LEDs
  digitalWrite(13, !digitalRead(13));  // pin 13 LED
  if(!startup) checkFlag = true;
  
  /*// do things on specific multiples of time
   *if (rtcRegs[1]|0x7F == 0){ // seconds
   *  if (rtcRegs[2]|0x7F == 0){  // minutes
   *    if (rtcRegs[3]|3F == 0){  // hours
   *    }
   *  }
   *
  //*/
}

void checkrtc(){  // ISR to get new RTC info, and update the display as needed
  readRTC(0x01,7);
  // the following is just to give electrodes time as different numerals.
  if (!override_display){
    dispDigs[5] = ( rtcRegs[1]     &0x0F);
    dispDigs[4] = ((rtcRegs[1]>>4) &0x07);
    dispDigs[3] = ( rtcRegs[2]     &0x0F);
    dispDigs[2] = ((rtcRegs[2]>>4) &0x07);
    dispDigs[1] = ( rtcRegs[3]     &0x0F);
    dispDigs[0] = ((rtcRegs[3]>>4) &0x03);
  }
  if ((((rtcRegs[3]>>4) &0x03) == 0) && (( rtcRegs[3]     &0x0F) == 0) && (((rtcRegs[2]>>4) &0x07) == 0)){
    // every 10 minutes, on the minute, run through the startup sequence, to keep from poisoning the cathodes
    startup = true;
  }
  checkFlag = false;
}

////////////////////////////////////////////////////////////////////////////////
// setDisplay
//
// This function updates all digits of the display using the least significant
// base 10 digits of inputValue, and turns the digits on or off using blankMask
//
// Arguments
// inputValue - an integer with at most 6 base 10 digits [between 0 and 999999]
//              if a larger number is presented the most significant digits will
//              be ignored.
// blankMask - a byte mask for which digits should be on and displaying a number
//             or should be off and not displaying any number
//             eg:
//             +-binary----+-hex--+-dec-+-Result----------------------------+
//             | b00000000 | 0x00 |   0 | Display all digits                |
//             | b00000011 | 0x03 |   3 | Display everything but seconds    |
//             | b00001100 | 0x0C |  12 | Display everything except minutes |
//             | b00110000 | 0x30 |  48 | Display everything except hours   |
//             | b00001111 | 0x0F |  15 | Only Display hours                |
//             | b00110011 | 0x33 |  51 | Only Display minutes              |
//             | b00111100 | 0x3C |  60 | Only Display Seconds              |
//             | b00111111 | 0x3F |  63 | Don't Display anything            |
//             +-----------+------+-----+-----------------------------------+
////////////////////////////////////////////////////////////////////////////////
void setDisplay(unsigned long inputValue, byte blankMask){
  for (int i=0; i<6; i++){
    // If the bit of the blankMask for this digit it 1 then turn this digit off
    if ((blankMask & (0x01<<i)) > 0){
      dispDigs[5-i] = 10;
    }
    // Otherwise set this digit to the value from the input
    else {
      dispDigs[5-i] = (inputValue%10);
    }
    // Chop off the digit that was just added for the next cycle
    inputValue /= 10;
  }
}


////////////////////////////////////////////////////////////////////////////////
// setDisplay
//
// A low level interface for setting the digits on the display.
// Digits should be passed in the order they are displayed in, [1,2,3,4,5,6]
// will display 123456 on the display
//
// inputDigits - an array of bytes representing the digits. Any value 10 or
//               above will cause the digit to be off instead of displaying
//               a number
////////////////////////////////////////////////////////////////////////////////
void setDisplay(byte inputDigits[6]){
  for (int i=0; i<6; i++){
    dispDigs[i] = (inputDigits[i]);
  }
}

void serialMenu(){
  unsigned int tempint = 0;
  unsigned char tempchar = 16;
  unsigned char tempSec = 0;
  unsigned char tempMin = 0;
  unsigned char tempHr = 0;
  tempstring = serialGetWord();
  Serial.print(F("$"));
  Serial.println(tempstring);
  
  if (tempstring == "help"){
    Serial.println(F("Command Options (prefix with '$'):"));
    Serial.println(F("set_led,redpwm,greenpwm,bluepwm // set the backlight led PWM values"));
    Serial.println(F("i2c_scan // searches for I2C devices and prints their addresses and registers"));
    Serial.println(F("display,decimal_value,BCD blank mask // sets the value to display, up to 999999; mask 63 is all off"));
    Serial.println(F("read_rtc,start_register,num bytes // reads rtc registers and prints results;"));
    Serial.println(F("\tstart at 0 if unspecified; read all if unspecified"));
    Serial.println(F("write_rtc,register,value // writes rtc registers and prints results;"));
    Serial.println(F("init_rtc // runs the RTCinit function"));
    Serial.println(F("set_time,hr,min,sec // sets the time, in decimal"));
    Serial.println(F("set_date,date,month,year // sets the date, in decimal"));
    Serial.println(F("ot,fraction/63 // sets the on duration, in microseconds/63"));
    Serial.println(F("check_rtc // runs the checkrtc function"));
    Serial.println(F("override,bool // sets the override modifier"));
    Serial.println(F("dt,int // set deadtime"));
    Serial.println(F("pd,char // set period"));
  }
  else if(tempstring == "dt"){
    if (Serial.available()>0){
      tempint = serialGetWord().toInt();
      if (tempint < 0) return;  // keep the user from doing silly things
      deadtime = tempint;
    }
    Serial.print(F("deadtime set to "));
    Serial.print(deadtime);
    Serial.println(F("us"));
  }
  else if(tempstring == "pd"){
    if (Serial.available()>0){
      tempint = serialGetWord().toInt();
      if (tempint < 0) return;  // keep the user from doing silly things
      if (tempint > 255) tempint = 255;  // keep the user from doing silly things
      period = tempint;
    }
    Serial.print(F("period set to "));
    Serial.print(period);
    Serial.println(F("us"));
  }
  else if(tempstring == "ot"){    
    if (Serial.available()>0){
      tempint = serialGetWord().toInt();
      if (tempint < 0) return;  // keep the user from doing silly things
      if (tempint > 255) tempint = 255;  // keep the user from doing silly things
      onTime = tempint;
    }
    Serial.print(F("ontime set to "));
    Serial.print(onTime);
    Serial.println(F("us"));
  }
  else if(tempstring == "i2c_scan"){
    i2cScan();
  }
  else if(tempstring == "check_rtc"){
    checkrtc();
  }
  else if (tempstring == "read_rtc"){
    if (Serial.available()>0) tempint = serialGetWord().toInt();
    if (tempint > 255) tempint = 255;  // keep the user from doing silly things
    if (Serial.available()>0) tempchar = serialGetWord().toInt();
    if (tempchar > 16) tempchar = 16;  // keep the user from doing silly things
    readRTC(tempint,tempchar);
  }
  else if (tempstring == "write_rtc"){
    if (Serial.available()>0) tempint = serialGetWord().toInt();
    if (tempint > 255) tempint = 255;  // keep the user from doing silly things
    if (Serial.available()>0) tempchar = serialGetWord().toInt();
    if (tempchar > 255) tempchar = 255;  // keep the user from doing silly things
    writeRTC(tempint,tempchar);
    readRTC(tempint,1);
  }
  else if (tempstring == "init_rtc"){
    RTCinit();
  }
  else if (tempstring == "set_time"){
    tempHr = serialGetWord().toInt();
    if (tempHr > 23) tempHr = 0;
    tempMin = serialGetWord().toInt();
    if (tempMin > 59) tempMin = 0;
    tempSec = serialGetWord().toInt();
    if (tempSec > 59) tempSec = 0;
    setTime(tempHr,tempMin,tempSec);
  }
  else if (tempstring == "set_date"){
    // re-using variables here:
    // tempHr is date, tempMin is month, tempSec is year
    tempHr = serialGetWord().toInt();
    if (tempHr > 31) tempHr = 0;
    tempMin = serialGetWord().toInt();
    if (tempMin > 12) tempMin = 0;
    tempSec = serialGetWord().toInt();
    if (tempSec > 99) tempSec = 0;
    setDate(tempHr,tempMin,tempSec);
  }
  else if (tempstring == "set_led"){
    Serial.print(F("setting LEDs\tR:"));
    redVal = serialGetWord().toInt();
    Serial.print(redVal);
    Serial.print(F("\tG:"));
    grnVal = serialGetWord().toInt();
    Serial.print(grnVal);
    Serial.print(F("\tB:"));
    bluVal = serialGetWord().toInt();
    Serial.println(bluVal);
    setLed(redVal, grnVal, bluVal);
  }
  else if (tempstring == "display"){
    dispVal = serialGetWord().toInt();
    if (Serial.available()>0) dispBlank = serialGetWord().toInt();
    else dispBlank = 0; // all digits active if no mask provided
      Serial.print(F("setting display value to: "));
      Serial.println(dispVal);
      setDisplay(dispVal, dispBlank);
  }
  else if (tempstring == "override"){
    switch (serialGetWord().toInt()){
      case 0:
        Serial.println(F("disabling override"));
        override_display = false;
        break;
      case 1:
        Serial.println(F("enabling override"));
        override_display = true;
        break;
      default:
        break;
    }
  }
  else{
    Serial.println(F("unrecognized command; type '$help' for options"));
  }
}

void setLed(unsigned char setred, unsigned char setgrn, unsigned char setblu){  // sets the static rear illumination LED PWM values to a new value
  redVal = setred;
  grnVal = setgrn;
  bluVal = setblu;
  analogWrite(PWM_RED, redVal);
  analogWrite(PWM_GRN, grnVal);
  analogWrite(PWM_BLU, bluVal);
}

String serialGetWord(){   // return information from the serial port input
  char inchar;
  String inword = "";
  bool startchar = false;
  bool endchar = false;
  
  if (divchar == true){
    divchar = false;
    startchar = true;
  }
  
  while (Serial.available()>0){
    inchar = Serial.read();
    if(inchar == '\n') endchar = true;
    if(inchar == '\r') endchar = true;
    if(inchar == ',') divchar = true;
    if(endchar == true){
      while (Serial.available()>0) Serial.read();
      if (startchar == false) Serial.println(F("incorrect format; type '$help' for options"));
      break;
    }
    if(divchar == true) break;
    if(startchar == true){
      inword += inchar;
    }
    if(inchar == '$') startchar = true;
  }
  return inword;
}

void i2cScan(){   // scan all possible i2c address for devices, and report any and their addresses
  byte error = 0x00;
  byte error2 = 0x00;
  byte address = 0x00;
  int nDevices = 0;
  Serial.println(F("I2C Devices at 8-bit addresses (even)"));

  for(address=1; address<127; address++){
    Wire.endTransmission();
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if(error==0){
      int i=0;
      Serial.print(F("Device at 0x"));
      if((address<<1)<16) Serial.print(F("0"));
      Serial.println(address<<1, HEX);
      Serial.print(F("Reg\tvalue\n"));
      Wire.requestFrom(address,255);
      for (i=0; i<Wire.available(); i++){
        byte temp = Wire.read();
        Serial.print(i);
        Serial.print(F("\t0x"));
        if(temp<16) Serial.print(F("0"));
        Serial.println(temp, HEX);
      }
      nDevices++;
      Wire.endTransmission();
    }
    else if (error==4){
      Serial.print(F("Unknown error at address 0x"));
      if((address<<1)<16) Serial.print(F("0"));
      Serial.println(address<<1, HEX);
    }
  }
  if(nDevices == 0) 
    Serial.println(F("No I2C Devices found"));
  else 
    Serial.println(F("done\n"));
}

void RTCinit(){   // initializes the RTC; run at startup
  unsigned char temp;
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x04);     // select the dow register
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDR>>1, 1);
  temp = Wire.read();   // get the sqw register values, so we don't overwrite other settings
  temp |= 0xF0;         // set to 1Hz output 1111xxxx
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x04);     // select the sqw register
  Wire.write(temp);     // send the new value
  Wire.endTransmission();

  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x0f);     // select the flags register
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDR>>1, 1);
  temp = Wire.read();
  Wire.endTransmission();
  if ((temp & 0x04)>0){ // check if the oscillator has faulted, aka lost time
    #ifdef __DEBUG__
      if (Serial) {
        Serial.println(F("Oscillator fault detected; please set the time"));
      }
    #endif
    OF = true;
  }

  if (OF){            // kickstart the oscillator if it stopped
    Wire.beginTransmission(RTC_ADDR>>1);
    Wire.write(0x01);     // select the seconds register
    Wire.endTransmission();
    Wire.requestFrom(RTC_ADDR>>1,1);
    temp = Wire.read();
    temp |= 0x80;
    Wire.endTransmission();
    Wire.beginTransmission(RTC_ADDR>>1);
    Wire.write(0x01);     // select the seconds register
    Wire.write(temp);     // send the new value
    Wire.endTransmission();

    Wire.beginTransmission(RTC_ADDR>>1);
    Wire.write(0x01);     // select the seconds register
    Wire.endTransmission();
    Wire.requestFrom(RTC_ADDR>>1,1);
    temp = Wire.read();
    temp &= !0x80;
    Wire.endTransmission();
    Wire.beginTransmission(RTC_ADDR>>1);
    Wire.write(0x01);     // select the seconds register
    Wire.write(temp);     // send the new value
    Wire.endTransmission();
  }
}

void readRTC(){   // reads the RTC registers starting from the beginning
  readRTC(0x00,16);
}

byte readRTC(byte reg, unsigned char count){
  // reads the RTC registers starting from the specified register
  String suffix[] = { F("\thundredths\n"),F("\tseconds\n"),F("\tminutes\n"),F("\thours\n"),F("\tdow\n"),
                      F("\tdate\n"),F("\tmonth\n"),F("\tyear\n"),F("\tcal\n"),F("\twatchdog\n"),
                      F("\talarm month\n"),F("\talarm date\n"),F("\talarm hour\n"),F("\talarm minute\n"),
                      F("\talarm second\n"),F("\tflags\n") };
  String output;
  output = "Reg\tvalue\n";
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(reg); //set the register at which to start reading
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDR>>1, count); //get all register values
  for (unsigned char i=reg; i<(reg+count); i++){
    rtcRegs[i] = Wire.read();
    #ifdef __DEBUG__
      output += i;
      output += "\t0x";
      if(rtcRegs[i]<16) output += "0";
      output += String(rtcRegs[i], HEX);
      output += suffix[i];
    #endif
  }
  #ifdef __DEBUG__
    if(Serial){
      Serial.println(output);
    }
  #endif
  Wire.endTransmission();
  return rtcRegs[reg+count];
}

void writeRTC(byte address, byte data){
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void setTime(unsigned char hours,unsigned char minutes, unsigned char seconds){
  byte tempByte = 0x00;
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(F("byte\tdec in\tmsn\tlsn\thex out"));
      Serial.print(F("secs \t"));
      Serial.print(seconds);
      Serial.print(F("\t0x"));
    }
  #endif
  // convert seconds to BCD
  seconds = ((seconds/10) << 4) | (seconds%10) & 0x7f;
  // ensure 0xxx xxxx, stop bit should not be affected by this and defaults to 0
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(seconds, HEX);
    }
  #endif
  
  // convert minutes to BCD
  tempByte = readRTC(0x02,1);  // get the current minutes register; need to know what OFIE is
  tempByte &= 0x80;       // get the OFIE bit
  #ifdef __DEBUG__
    if (Serial){
      Serial.print(F("mins \t"));
      Serial.print(minutes);
      Serial.print(F("\t0x"));
    }
  #endif
  minutes = ((minutes/10) << 4) | (minutes%10) | tempByte;
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(minutes, HEX);
      Serial.print(F("hrs \t"));
      Serial.print(hours);
      Serial.print(F("\t0x"));
    }
  #endif
  // convert hours to BCD
  hours = ((hours/10) << 4) | (hours%10);
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(hours, HEX);
    }
  #endif
  
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x01);       // start at seconds; tenths and hundredths will automatically set to 0
  Wire.write(seconds);    // set the seconds and stop bit
  Wire.write(minutes);    // set the minutes and OFIE bit
  Wire.write(hours);      // set the hours
  Wire.endTransmission();
}

void setDate(unsigned char date, unsigned char month,unsigned char year){
  byte tempByte = 0;
  byte dow = 0;
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(F("byte\tdec in\thex out"));
    }
  #endif

  // determine day of week
  // see http://mathforum.org/dr.math/faq/faq.calendar.html
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x04); //select the dow register
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDR>>1, 1);
  tempByte = Wire.read();
  tempByte &= 0xF0;   // keep only the RS setting bits
  
  dow = year/4 + date + dowKey[month-1] + year;
  dow += 6;  // needs adjustment for centuries after 2099, but not supported here
  if ((month == 1 or month == 2) and (year%4 == 0)) dow -= 1;
  dow = dow%7;
  #ifdef __DEBUG__
    if (Serial) Serial.println(dow);
  #endif
  // 0 is saturday, 1 is sunday, 2 is monday...
  tempByte |= dow;
  #ifdef __DEBUG__
    if (Serial){
      Serial.print(F("dow \t"));
      Serial.print(dow);
      Serial.print(F("\t0x"));
      Serial.println(tempByte, HEX);
    }
  #endif
  
  #ifdef __DEBUG__
    if (Serial){
      Serial.print(F("date \t"));
      Serial.print(date);
      Serial.print(F("\t0x"));
    }
  #endif
  // convert date to BCD
  date = ((date/10) << 4) | (date%10) & 0x3f;
  // ensure 00xx xxxx, for valid date
  
  #ifdef __DEBUG__
    if (Serial) Serial.println(date, HEX);
  #endif

  // convert month to BCD
  #ifdef __DEBUG__
    if (Serial){
      Serial.print(F("month \t"));
      Serial.print(month);
      Serial.print(F("\t0x"));
    }
  #endif
  month = ((month/10) << 4) | (month%10) & 0x3f;
  // room here for century bits at 0x80 and 0x40, but not supporting now: 000x xxxx
  #ifdef __DEBUG__
    if (Serial){
      Serial.println(month, HEX);
      Serial.print(F("year \t"));
      Serial.print(year);
      Serial.print(F("\t0x"));
    }
  #endif
  // convert year to BCD
  year = ((year/10) << 4) | (year%10);

  #ifdef __DEBUG__
    if (Serial) Serial.println(year, HEX);
  #endif

  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR>>1);
  Wire.write(0x04);       // start at dow
  Wire.write(tempByte);   // set the dow
  Wire.write(date);       // set the day
  Wire.write(month);      // set the month
  Wire.write(year);       // set the year
  Wire.endTransmission();
}

/*
void setAlarm(hours,minutes,seconds){
  
}
//*/
