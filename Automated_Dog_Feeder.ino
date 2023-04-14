/**
 * Prototype software for Automated Multi-Dog Feeder
 * Version 0.5
 */
 // Library Inclusions ===========================================================================================================
 #include <Servo.h> // Arduino Servo motor library
 #include "Wire.h" // For I2C
 #include "LCD.h" // For LCD
 #include "LiquidCrystal_I2C.h" // for I2C LCD backpack
 //#include <SPI.h> // for SD card
 #include <SD.h> // for SD card
 #include <HX711.h> // for load cell ADC
 #include <ThreeWire.h> // software SPI for RTC
 #include <RtcDS1302.h> // for RTC
 #include <TMRpcm.h>   // .wav library
 #include "PN532_I2C.h" //RFID I2C
 #include "PN532.h" //RFID
 #include "NfcAdapter.h" //RFID Read Data
 #include <EEPROM.h> // Arduino standard EEPROM library
 // end library Inclusions =======================================================================================================
 
 // I/O pin Designations =========================================================================================================
 #define SD_ChipSelectPin 53
 #define SLScontrol 9 //control pin for SLS servo motor
 #define menu 3 // menu button with interrupt
 #define up 4 // up button
 #define down 5 // down button
 #define back 6 // back button
 #define enter 7 // enter button

 #define SCALE_LEFT_DOUT 23 //left load cell data
 #define SCALE_LEFT_SCK 25 //left load cell clock
 #define SCALE_RIGHT_DOUT 22 //right load cell data
 #define SCALE_RIGHT_SCK 24 //right load cell clock

 #define RTC_CLK 47
 #define RTC_DAT 48
 #define RTC_RST 49

 #define SOUND 46 // Audio output

 #define MOTOR_LEFT_OPEN 31 //green
 #define MOTOR_LEFT_CLOSE 29 // yellow
 #define MOTOR_RIGHT_OPEN 30 // purple
 #define MOTOR_RIGHT_CLOSE 28 // blue

 #define RIGHT_CLOSED 37 // red
 #define RIGHT_OPEND 39 //yellow
 #define LEFT_CLOSED 41 // white
 #define LEFT_OPEND 35 // green
 

 // end I/O pins =================================================================================================================
 

 // Constants ====================================================================================================================
 #define SLS_idle_position 80 // Idle position for SLS storage(degrees)
 #define SLS_left_fill 0 // fill position to left hopper(degrees)
 #define SLS_left_dump 45 // dump positon to left bowl(degrees)
 #define SLS_right_fill 160 // fill position to right hopper(degrees)
 #define SLS_right_dump 110 // dump positon to right bowl(degrees)
 #define SLS_fill_delay 2000 // delay for SLS to fill from hopper(mS)
 #define SLS_dump_delay 2000 // delay for SLS to dump into bowl(mS)
 #define buttondelay 500 // delay for button debounce(mS)
// end constants============================================================================================================================

 const int chipSelect = 53;
 volatile boolean userInterface = false;
 volatile boolean allowSimultaneousFeeding;
 volatile boolean recentBowlMove = false;
 String tagId1 = "EC E7 19 49";
 String tagId2= "50 19 8F 1E"; // with constants
 String tagId = "None";
 byte nuidPICC[4]; //Not sure where this goes


//Structs===============================================================================================================================

  typedef struct FakeDateTime{
    uint16_t year;
    uint8_t month;
    uint8_t dayOfMonth;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
  };
  typedef struct timeString{
    char hours[2];
    char mins[2];
    
    
  };
  
  typedef struct feeding{
    
    uint8_t quartercups; // byte expecting values 1-16 correspondint to the number of quarter cups
    uint8_t hour; 
    uint8_t mins;
    char noise[9]; // needs to be a constant length char array
  };
  feeding leftMorning; // left dog morning feeding schedule to be kept at EEPROM Add. 0
  feeding leftEvening; // left dog evening feeding schedule to be kept at EEPROM Add. 1000
  feeding rightMorning; // right dog morning feeding schedule to be kept at EEPROM Add. 2000
  feeding rightEvening; // right dog evening feeding schedule to be kept at EEPROM Add. 3000
// instantiation of objects
 PN532_I2C pn532_i2c(Wire);
 NfcAdapter nfc = NfcAdapter(pn532_i2c);

 Servo SLS; // instantiate SLS servo motor
 HX711 leftScale; // Left Load Cell ADC
 HX711 rightScale; // Right Load Cell ADC
 LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); //ADDR,EN,R/W,RS,D4,D5,D6,D7 0x27 is the default I2C address for LCD
 ThreeWire RTC_Serial(RTC_DAT, RTC_CLK, RTC_RST);
 RtcDS1302<ThreeWire> Clock(RTC_Serial);
 TMRpcm speak; // audio output object
 
// Right arrow for menu
byte ARROW[8] = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};

//functions ######################################################################################################################
void displayTime(){
  
}

FakeDateTime Real2FakeDate(RtcDateTime) {
  FakeDateTime fakedate;
    uint16_t year;
    uint8_t month;
    uint8_t dayOfMonth;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
  }

String readNFC() {
  Serial.println("Reading NFC");
  if (nfc.tagPresent()){
    speak.play("beep.wav");
    NfcTag tag = nfc.read();
    tag.print();
    tagId = tag.getUidString();
    Serial.println("Tag ID:");
    Serial.println(tagId);
  }
  else {
  return(F("NoDog"));
  };

  return tagId;
}

void RFID() {
  Serial.println("Reading Rfid");
  tagId = readNFC();
  Serial.println(tagId);
  if(tagId==tagId2){
    Serial.println("Dog 2 Present");
      open_right();
      close_left();
    };
    if(tagId==tagId1){
      //close_right();
      open_left();
      close_right();
    }
  if(tagId==(F("NoDog"))){
    close_right();
    close_left();
  }
}

int open_left(){
  if(!recentBowlMove){
    digitalWrite(MOTOR_LEFT_OPEN, HIGH);
    while (digitalRead(LEFT_OPEND) != LOW){
    }
    digitalWrite(MOTOR_LEFT_OPEN, LOW);
  }
  bowlMoved();
  return 1;
 }
 int open_right(){
  if(!recentBowlMove){
    digitalWrite(MOTOR_RIGHT_OPEN, HIGH);
    while (digitalRead(RIGHT_OPEND) != LOW){
    }
    digitalWrite(MOTOR_RIGHT_OPEN, LOW);
  }
  bowlMoved();
  return 1;
 }
 int close_left(){
  if(!recentBowlMove){
    digitalWrite(MOTOR_LEFT_CLOSE, HIGH);
    while(digitalRead(LEFT_CLOSED) != LOW){
    }
    digitalWrite(MOTOR_LEFT_CLOSE, LOW);
  }
  bowlMoved();
  return 1;
 }
 int close_right(){
  if(!recentBowlMove){
    digitalWrite(MOTOR_RIGHT_CLOSE, HIGH);
    while(digitalRead(RIGHT_CLOSED) != LOW){
    }
    digitalWrite(MOTOR_RIGHT_CLOSE, LOW);
  }
  bowlMoved();
  return 1;
 }
 
 void bowlMoved(){
  //recentBowlMove = true;
  
 }
 
 void SLS_Idle(){
  //Actuates SLS to idle position
  SLS.write(SLS_idle_position);
  delay(150);
  Serial.println("SLS is Idle");
 }

 
 void SLS_Left(int quarterCups){
  //fills left bowl with "quarterCups" of left food
  for(int i=0; i<quarterCups; i++){
    SLS.write(SLS_left_fill);
    delay(SLS_fill_delay);
    SLS.write(SLS_left_dump);
    delay(SLS_dump_delay);
    Serial.print("dump #");
    Serial.println(i);
  }
  SLS_Idle();
 }
 
  void SLS_Right(int quarterCups){
    // fills right bowl with "quarterCups" of right food
  for(int i=0; i<quarterCups; i++){
    SLS.write(SLS_right_fill);
    delay(SLS_fill_delay);
    SLS.write(SLS_right_dump);
    delay(SLS_dump_delay);
    Serial.print("dump #");
    Serial.println(i);
  }
  SLS_Idle();
 }

void printFeeding(feeding input){
  Serial.println("printing feed time");
  float cups = (float(input.quartercups) * 0.25);
  String noise = input.noise;
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(cups);
  lcd.print("c ");
  lcd.print(input.hour);
  lcd.print(":");
  lcd.print(input.mins);
  lcd.print(" ");
  lcd.println(input.noise);

  Serial.print(cups);
  Serial.print("c ");
  Serial.print(input.hour);
  Serial.print(":");
  Serial.print(input.mins);
  Serial.print(" ");
  Serial.println(input.noise);
  Serial.print("in struct:");
  Serial.println(input.noise);
  Serial.print("string:");
  Serial.println(noise);
}

// User Interface #######################################################################################################################################
 void ISR_UI(){
  userInterface = true;
 }

 void UI(){
  String Set_Clock = "Set Clock";
  String Left_Dog = "Left Dog";
  String Right_Dog = "Right Dog";
  String Option = "Option";
  RtcDateTime timeset;
  feeding feedset = {1, 10, 30, "test.wav\0"};
  boolean isDog; // true for left dog false for right
  boolean isMorning; // true for morning false for evening
  
   main_clock:
   lcd.clear();
   lcd.write(9);
   lcd.print(Set_Clock);
   lcd.setCursor(1,1);
   lcd.print(Left_Dog);
   lcd.setCursor(1,2);
   lcd.print(Right_Dog);
   lcd.setCursor(1,3);
   lcd.print(Option);
   delay(buttondelay);
   while(true){
    if(!digitalRead(up))goto main_option;
    if(!digitalRead(down)) goto main_left;
    if(!digitalRead(back)) goto UI_exit;
    if(!digitalRead(enter)){
      timeset = Clock.GetDateTime();
      goto Clock_Set;
    }
   }

   main_left:
   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print(Set_Clock);
   lcd.setCursor(0,1);
   lcd.write(9);
   lcd.print(Left_Dog);
   lcd.setCursor(1,2);
   lcd.print(Right_Dog);
   lcd.setCursor(1,3);
   lcd.print(Option);
   delay(buttondelay);
   while(true){
    if(!digitalRead(up)) goto main_clock;
    if(!digitalRead(down)) goto main_right;
    if(!digitalRead(back)) goto UI_exit;
    if(!digitalRead(enter)){
      isDog = true;
      goto morning;
    }
   }

   main_right:
   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print(Set_Clock);
   lcd.setCursor(1,1);
   lcd.print(Left_Dog);
   lcd.setCursor(0,2);
   lcd.write(9);
   lcd.print(Right_Dog);
   lcd.setCursor(1,3);
   lcd.print(Option);
   delay(buttondelay);
   while(true){
    if(!digitalRead(up))goto main_left;
    if(!digitalRead(down)) goto main_option;
    if(!digitalRead(back)) goto UI_exit;
    if(!digitalRead(enter)){
      isDog = false;
      goto morning;
    }
   }

   morning:
   lcd.clear();
   lcd.write(9);
   lcd.print("Morning");
   lcd.setCursor(1,1);
   lcd.print("Evening");
   delay(buttondelay);
   while(true){
    if(!digitalRead(up))goto evening;
    if(!digitalRead(down)) goto evening;
    if(!digitalRead(back)){
      if(isDog){
        goto main_left;
      }
      else goto main_right;
    }
    if(!digitalRead(enter)){
      isMorning = true;
      goto set_feeding;
    }
   }

   evening:
   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print("Morning");
   lcd.setCursor(0,1);
   lcd.write(9);
   lcd.print("Evening");
   delay(buttondelay);
   while(true){
    if(!digitalRead(up))goto evening;
    if(!digitalRead(down)) goto evening;
    if(!digitalRead(back)){
      if(isDog){
        goto main_left;
      }
      else goto main_right;
     }
    if(!digitalRead(enter)){
      isMorning = false;
      goto set_feeding;
    }
   }
  
  set_feeding:
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.print("In Set Feeding");
  printFeeding(feedset);
  delay(buttondelay);
    while(true){
    if(!digitalRead(up)){
      if(feedset.quartercups<16){
        feedset.quartercups++;
      }
       goto set_feeding;
    }
    if(!digitalRead(down)){
      if(feedset.quartercups>1){
        feedset.quartercups--;
      }
      goto set_feeding;
    }
    if(!digitalRead(back)){
      if(isMorning)goto morning;
      else goto evening;
    }
    if(!digitalRead(enter)) goto set_feeding_hrs;
   }

 set_feeding_hrs:
   printFeeding(feedset);
   delay(buttondelay);
    while(true){
    if(!digitalRead(up)){
      if(feedset.hour<23) feedset.hour++;
      goto set_feeding_hrs;
    }
    if(!digitalRead(down)){
      if(feedset.hour>0) feedset.hour--;
      goto set_feeding_hrs;
    }
    if(!digitalRead(back)) goto set_feeding;
    if(!digitalRead(enter)) goto set_feeding_mins;
   }

   set_feeding_mins:
   printFeeding(feedset);
   delay(buttondelay);
    while(true){
    if(!digitalRead(up)){
      if(feedset.mins<59) feedset.mins++;// = feedset.hour + 1;
      goto set_feeding_mins;
    }
    if(!digitalRead(down)){
      if(feedset.hour>0) feedset.mins--;// = feedset.hour - 1;
      goto set_feeding_mins;
    }
    if(!digitalRead(back)) goto set_feeding_hrs;
    //if(!digitalRead(enter)) goto set_feeding_mins;
   }

   
    
   

  main_option:
   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print(Set_Clock);
   lcd.setCursor(1,1);
   lcd.print(Left_Dog);
   lcd.setCursor(1,2);
   lcd.print(Right_Dog);
   lcd.setCursor(0,3);
   lcd.write(9);
   lcd.print(Option);
   delay(buttondelay);
   while(true){
    if(!digitalRead(up))goto main_right;
    if(!digitalRead(down)) goto main_clock;
    if(!digitalRead(back)) goto UI_exit;
    if(!digitalRead(enter)) goto OptionA;
   }

   

  
  Clock_Set:
  lcd.clear();
  //printDateTime(timeset);
  delay(buttondelay);
  while(true){
    if(!digitalRead(up)){
     //timeset.Month() = timeset.Month()+1;
     goto Clock_Set;
    }
    //if(!digitalRead(down)) goto main_clock;
    if(!digitalRead(back)) goto main_clock;
    //if(!digitalRead(enter)) goto
   }

  OptionA:
  lcd.clear();
  lcd.print("Feed simultaneously?");
  lcd.setCursor(0,1);
  lcd.write(9);
  lcd.print("Yes");
  lcd.setCursor(1,2);
  lcd.print("No");
  delay(buttondelay);
  while(true){
    if(!digitalRead(up))goto OptionB;
    if(!digitalRead(down)) goto OptionB;
    if(!digitalRead(back)) goto main_option;
    if(!digitalRead(enter)){SetOption(true); goto main_option;}
   }
   
   OptionB:
  lcd.clear();
  lcd.print("Feed simultaneously?");
  lcd.setCursor(1,1);
  lcd.print("Yes");
  lcd.setCursor(0,2);
  lcd.write(9);
  lcd.print("No");
  delay(buttondelay);
  while(true){
    if(!digitalRead(up))goto OptionA;
    if(!digitalRead(down)) goto OptionA;
    if(!digitalRead(back)) goto main_option;
    if(!digitalRead(enter)){
      SetOption(false);
      goto main_option;
     }
    }
   
  
  UI_exit:
  userInterface = false;
  lcd.clear();
  return;
 } // End User Interface*********************************************************************************************************************************

 void SetOption(boolean toSet){
  if(toSet=true){allowSimultaneousFeeding = true; digitalWrite(13, LOW);}
  
  else if(toSet=false){allowSimultaneousFeeding = false; digitalWrite(13, LOW);}
  Serial.println(allowSimultaneousFeeding);
  return;
 }


void setup() {
  Serial.begin(9600);
  nfc.begin();
  pinMode(13, OUTPUT);

  pinMode(menu, INPUT_PULLUP);
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  pinMode(enter, INPUT_PULLUP);
  attachInterrupt(1, ISR_UI, LOW);

  pinMode(RIGHT_CLOSED, INPUT_PULLUP);
  pinMode(RIGHT_OPEND, INPUT_PULLUP);
  pinMode(LEFT_CLOSED, INPUT_PULLUP);
  pinMode(LEFT_OPEND, INPUT_PULLUP);
  
  pinMode(MOTOR_LEFT_OPEN, OUTPUT);
  pinMode(MOTOR_RIGHT_CLOSE, OUTPUT);
  pinMode(MOTOR_LEFT_CLOSE, OUTPUT);
  pinMode(MOTOR_RIGHT_OPEN, OUTPUT);
  digitalWrite(MOTOR_LEFT_OPEN, LOW);
  digitalWrite(MOTOR_RIGHT_CLOSE, LOW);
  digitalWrite(MOTOR_LEFT_CLOSE, LOW);
  digitalWrite(MOTOR_RIGHT_OPEN, LOW);

  leftScale.begin(SCALE_LEFT_DOUT, SCALE_LEFT_SCK);
  leftScale.set_scale();
  leftScale.tare();

  rightScale.begin(SCALE_RIGHT_DOUT, SCALE_RIGHT_SCK);
  rightScale.set_scale();
  rightScale.tare();
  
  // activate SLS
  SLS.attach(SLScontrol);
  
  lcd.begin (20,4); // 20 x 4 LCD module
  lcd.setBacklightPin(3,POSITIVE); // BL, BL_POL
  lcd.setBacklight(HIGH);
  lcd.createChar(9, ARROW);
  lcd.clear();

  Clock.Begin();
  
  pinMode(SOUND, OUTPUT);
  digitalWrite(SOUND, LOW);
  speak.speakerPin = SOUND;
  speak.volume(7);

    if (!SD.begin(SD_ChipSelectPin)) {  // see if the card is present and can be initialized:
    Serial.println("SD fail");  
    return;   // don't do anything more if not
  }  

 // Test code below here in setup ***************************************************************************************************
//  RtcDateTime CurrentTime = RtcDateTime(__DATE__,__TIME__);
//  Clock.SetDateTime(CurrentTime);

// print the contents of EEPROM  to serial output
//  Serial.println(" contents of EEProm");
//
//  for(int addr = 0; addr < 4096; addr=addr+8){
//    Serial.print(addr);
//    for(int a=0; a<8; a++){
//      Serial.print("\t");
//      Serial.print(EEPROM.read(addr + a));
//    }
//   Serial.println();
//  }

} // End of Setup******************************************************************************************************************

void loop() {
  if(userInterface){
    UI();
  }

  
  RtcDateTime now = Clock.GetDateTime();
  String timestamp = date2string(now);
  Serial.println(timestamp);
  lcd.clear();
  lcd.print(timestamp);
  printDateTime(now);
  
//  Serial.println("playing");
//  speak.play("test2.wav");
//  while(speak.isPlaying()){
    //RFID();
    if(!speak.isPlaying()){
      digitalWrite(SOUND, LOW);
    }
  delay(2000);
//  RFID();


//  RFID();
//  delay(2000);
  
  //speak.play("a.wav");
  
  // Test code in loop below here *************************************************************************************************


}

// more functions



//String date2string(const RtcDateTime& dateTime){
//  char datestring[25];
//
//  snprintf_P(datestring, (sizeof(datestring) / sizeof(datestring[0])),
//  PSTR("%02u/%02u/%04u %02u:%02u"),
//      //("%02u/%02u/%04u %02u:%02u:%02u")
//      dateTime.Month(),
//      dateTime.Day(),
//      dateTime.Year(),
//      dateTime.Hour(),
//      dateTime.Minute()//,
//      //dateTime.Second()
//      );
//  return datestring; // I do not know if returning a Char[] instead of a string works, maybe i should null terminate it? ah we're leaving this code anyway
//}



String date2string(const RtcDateTime& dateTime){
  char datestring[25];

  snprintf_P(datestring, (sizeof(datestring) / sizeof(datestring[0])),
  PSTR("%04u/%02u/%02u %02u:%02u"),
      //("%04u/%02u/%02u %02u:%02u:%02u")
      dateTime.Year(),
      dateTime.Month(),
      dateTime.Day(),
      dateTime.Hour(),
      dateTime.Minute()//,
      //dateTime.Second()
      );
  return datestring; // I do not know if returning a Char[] instead of a string works, maybe i should null terminate it? ah we're leaving this code anyway
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
  String datetime = date2string(dt);
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print(datetime);
  Serial.println(datetime);
    
}
