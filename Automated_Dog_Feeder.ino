/**
   Prototype software for Automated Multi-Dog Feeder
   Version 0.5
*/
// Library Inclusions ===========================================================================================================
#include <Servo.h> // Arduino Servo motor library
#include "Wire.h" // For I2C
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h" // for I2C LCD backpack
#include <SPI.h> // for SD card
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
#define up 5 // up button
#define down 4 // down button
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

#define UI_DELAY 300000


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
String tagId2 = "50 19 8F 1E"; // with constants
String tagId = "None";
byte nuidPICC[4]; //Not sure where this goes


//Structs===============================================================================================================================

typedef struct fakeDateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};


typedef struct feeding { // schedule for a feeding time
  uint8_t quartercups; // byte expecting values 1-16 corresponding to the number of quarter cups
  uint8_t hour;
  uint8_t mins;
  char noise[25]; // needs to be a constant length char array
  boolean isLeftDog; // true for left false for right
  uint8_t choice; // 0 for always, 1 for fill, 2 for wait
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
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); //ADDR,EN,R/W,RS,D4,D5,D6,D7 0x27 is the default I2C address for LCD
ThreeWire RTC_Serial(RTC_DAT, RTC_CLK, RTC_RST);
RtcDS1302<ThreeWire> Clock(RTC_Serial);
TMRpcm speak; // audio output object
File root; // SD card file

//Sd2Card card;
//SdVolume volume;
//SdFile root;

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
void updateTime() {
  RtcDateTime now = Clock.GetDateTime();
  printDateTime(now);
}


void updateEEPROM() {
  leftMorning.isLeftDog = true;
  leftEvening.isLeftDog = true;
  rightMorning.isLeftDog = false;
  rightEvening.isLeftDog = false;
  EEPROM.put(0, leftMorning);
  EEPROM.put(1000, leftEvening);
  EEPROM.put(2000, rightMorning);
  EEPROM.put(3000, rightEvening);
}

void updateRAM() {
  EEPROM.get(0, leftMorning);
  EEPROM.get(1000, leftEvening);
  EEPROM.get(2000, rightMorning);
  EEPROM.get(3000, rightEvening);
  allowSimultaneousFeeding = EEPROM.read(4000);
}

fakeDateTime Real2FakeDate(RtcDateTime& realDateTime) {
  fakeDateTime fakedate = {realDateTime.Year(), realDateTime.Month(), realDateTime.Day(), realDateTime.Hour(), realDateTime.Minute()};
  return fakedate;
}

String readNFC() {
  Serial.println("Reading NFC");
  if (nfc.tagPresent()) {
    speak.play("beep.wav");
    NfcTag tag = nfc.read();
    tag.print();
    tagId = tag.getUidString();
    Serial.println("Tag ID:");
    Serial.println(tagId);
  }
  else {
    return (F("NoDog"));
  };

  return tagId;
}

void RFID() {
  Serial.println("Reading Rfid");
  tagId = readNFC();
  Serial.println(tagId);
  if (tagId == tagId2) {
    Serial.println("Dog 2 Present");
    open_right();
    close_left();
  };
  if (tagId == tagId1) {
    //close_right();
    open_left();
    close_right();
  }
  if (tagId == (F("NoDog"))) {
    close_right();
    close_left();
  }
}

int open_left() {
  if (!recentBowlMove) {
    digitalWrite(MOTOR_LEFT_OPEN, HIGH);
    while (digitalRead(LEFT_OPEND) != LOW) {
    }
    digitalWrite(MOTOR_LEFT_OPEN, LOW);
  }
  bowlMoved();
  return 1;
}
int open_right() {
  if (!recentBowlMove) {
    digitalWrite(MOTOR_RIGHT_OPEN, HIGH);
    while (digitalRead(RIGHT_OPEND) != LOW) {
    }
    digitalWrite(MOTOR_RIGHT_OPEN, LOW);
  }
  bowlMoved();
  return 1;
}
int close_left() {
  if (!recentBowlMove) {
    digitalWrite(MOTOR_LEFT_CLOSE, HIGH);
    while (digitalRead(LEFT_CLOSED) != LOW) {
    }
    digitalWrite(MOTOR_LEFT_CLOSE, LOW);
  }
  bowlMoved();
  return 1;
}
int close_right() {
  if (!recentBowlMove) {
    digitalWrite(MOTOR_RIGHT_CLOSE, HIGH);
    while (digitalRead(RIGHT_CLOSED) != LOW) {
    }
    digitalWrite(MOTOR_RIGHT_CLOSE, LOW);
  }
  bowlMoved();
  return 1;
}

void bowlMoved() {
  //recentBowlMove = true;

}

void SLS_Idle() {
  //Actuates SLS to idle position
  SLS.write(SLS_idle_position);
  delay(150);
  Serial.println("SLS is Idle");
}


void SLS_Left(int quarterCups) {
  //fills left bowl with "quarterCups" of left food
  for (int i = 0; i < quarterCups; i++) {
    SLS.write(SLS_left_fill);
    delay(SLS_fill_delay);
    SLS.write(SLS_left_dump);
    delay(SLS_dump_delay);
    Serial.print("dump #");
    Serial.println(i);
  }
  SLS_Idle();
}

void SLS_Right(int quarterCups) {
  // fills right bowl with "quarterCups" of right food
  for (int i = 0; i < quarterCups; i++) {
    SLS.write(SLS_right_fill);
    delay(SLS_fill_delay);
    SLS.write(SLS_right_dump);
    delay(SLS_dump_delay);
    Serial.print("dump #");
    Serial.println(i);
  }
  SLS_Idle();
}




void printFeeding(feeding input) {
  Serial.println("printing feed time");
  float cups = (float(input.quartercups) * 0.25);
  String setTime = time2string(input.hour, input.mins);
  String noise = input.noise;
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(cups);
  lcd.print("c ");
  lcd.print(setTime);
  //lcd.print(":");
  //lcd.print(input.mins);
  lcd.print(" ");
  switch (input.choice) {
    case 0: lcd.print("always"); break;
    case 1: lcd.print("fill"); break;
    case 2: lcd.print("wait"); break;
  }
  lcd.setCursor(4, 3);
  lcd.print(noise);
}
void serialPrintFeed(feeding input) {
  Serial.println("printing feed time");
  float cups = (float(input.quartercups) * 0.25);
  String noise = input.noise;
  String setTime = time2string(input.hour, input.mins);
  Serial.print(cups);
  Serial.print("c ");
  Serial.print(setTime);
  //Serial.print(":");
  //Serial.print(input.mins);
  Serial.print(" ");
  Serial.println(input.noise);
  Serial.print("in struct:");
  Serial.println(input.noise);
  Serial.print("string:");
  Serial.println(noise);
}
String time2string(uint8_t hours, uint8_t mins) {
  char timestring[8];
  snprintf_P(timestring, (sizeof(timestring) / sizeof(timestring[0])), PSTR("%02u:%02u"), hours, mins);
  return timestring;
}



String date2string(const RtcDateTime& dateTime) {
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

String date2string(const fakeDateTime& dateTime) {
  char datestring[25];

  snprintf_P(datestring, (sizeof(datestring) / sizeof(datestring[0])),
             PSTR("%04u/%02u/%02u %02u:%02u"),
             //("%04u/%02u/%02u %02u:%02u:%02u")
             dateTime.year,
             dateTime.month,
             dateTime.day,
             dateTime.hour,
             dateTime.minute//,
             //dateTime.second()
            );
  return datestring; // I do not know if returning a Char[] instead of a string works, maybe i should null terminate it? ah we're leaving this code anyway
}

void printDateTime(const fakeDateTime& dt) {
  String datetime = date2string(dt);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print(datetime);
  //Serial.println(datetime);
}

void printDateTime(const RtcDateTime& dt) {
  String datetime = date2string(dt);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print(datetime);
  //Serial.println(datetime);
}

// User Interface #######################################################################################################################################
void ISR_UI() {
  userInterface = true;
}

void UI() {
  String Set_Clock = "Set Clock";
  String Left_Dog = "Left Dog";
  String Right_Dog = "Right Dog";
  String Option = "Option";
  String noise;
  fakeDateTime timeset;
  uint8_t maxday;
  uint32_t endtime = millis() + UI_DELAY;
  File file;
  feeding feedset;
  boolean isDog; // true for left dog false for right
  boolean isMorning; // true for morning false for evening

main_clock:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.write(9);
  lcd.print(Set_Clock);
  lcd.setCursor(1, 1);
  lcd.print(Left_Dog);
  lcd.setCursor(1, 2);
  lcd.print(Right_Dog);
  lcd.setCursor(1, 3);
  lcd.print(Option);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto main_option;
    if (!digitalRead(down)) goto main_left;
    if (!digitalRead(back)) goto UI_exit;
    if (!digitalRead(enter)) {
      RtcDateTime now = Clock.GetDateTime();
      timeset = Real2FakeDate(now);
      goto Clock_Set;
    }
    if (millis() > endtime) goto UI_exit;
  }

main_left:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(Set_Clock);
  lcd.setCursor(0, 1);
  lcd.write(9);
  lcd.print(Left_Dog);
  lcd.setCursor(1, 2);
  lcd.print(Right_Dog);
  lcd.setCursor(1, 3);
  lcd.print(Option);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) goto main_clock;
    if (!digitalRead(down)) goto main_right;
    if (!digitalRead(back)) goto UI_exit;
    if (!digitalRead(enter)) {
      isDog = true;
      goto morning;
    }
    if (millis() > endtime) goto UI_exit;
  }

main_right:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(Set_Clock);
  lcd.setCursor(1, 1);
  lcd.print(Left_Dog);
  lcd.setCursor(0, 2);
  lcd.write(9);
  lcd.print(Right_Dog);
  lcd.setCursor(1, 3);
  lcd.print(Option);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto main_left;
    if (!digitalRead(down)) goto main_option;
    if (!digitalRead(back)) goto UI_exit;
    if (!digitalRead(enter)) {
      isDog = false;
      goto morning;
    }
    if (millis() > endtime) goto UI_exit;
  }

main_option:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(Set_Clock);
  lcd.setCursor(1, 1);
  lcd.print(Left_Dog);
  lcd.setCursor(1, 2);
  lcd.print(Right_Dog);
  lcd.setCursor(0, 3);
  lcd.write(9);
  lcd.print(Option);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto main_right;
    if (!digitalRead(down)) goto main_clock;
    if (!digitalRead(back)) goto UI_exit;
    if (!digitalRead(enter)) {
      if (allowSimultaneousFeeding) goto OptionA;
      else goto OptionB;
    }
    if (millis() > endtime) goto UI_exit;
  }

morning:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.write(9);
  lcd.print("Morning");
  lcd.setCursor(1, 1);
  lcd.print("Evening");
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto evening;
    if (!digitalRead(down)) goto evening;
    if (!digitalRead(back)) {
      if (isDog) {
        goto main_left;
      }
      else goto main_right;
    }
    if (!digitalRead(enter)) {
      isMorning = true;
      goto feeding_setup;
    }
    if (millis() > endtime) goto UI_exit;
  }

evening:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Morning");
  lcd.setCursor(0, 1);
  lcd.write(9);
  lcd.print("Evening");
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto morning;
    if (!digitalRead(down)) goto morning;
    if (!digitalRead(back)) {
      if (isDog) {
        goto main_left;
      }
      else goto main_right;
    }
    if (!digitalRead(enter)) {
      isMorning = false;
      goto feeding_setup;
    }
    if (millis() > endtime) goto UI_exit;
  }

feeding_setup:
  if (isDog) {
    if (isMorning) feedset = leftMorning;
    else feedset = leftEvening;
  }
  if (!isDog) {
    if (isMorning) feedset = rightMorning;
    else feedset = rightEvening;
  }
  goto set_feeding;


set_feeding:
  endtime = millis() + UI_DELAY;
  printFeeding(feedset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (feedset.quartercups < 16) {
        feedset.quartercups++;
      }
      goto set_feeding;
    }
    if (!digitalRead(down)) {
      if (feedset.quartercups > 1) {
        feedset.quartercups--;
      }
      goto set_feeding;
    }
    if (!digitalRead(back)) {
      if (isMorning)goto morning;
      else goto evening;
    }
    if (!digitalRead(enter)) goto set_feeding_hrs;
    if (millis() > endtime) goto UI_exit;
  }

set_feeding_hrs:
  endtime = millis() + UI_DELAY;
  printFeeding(feedset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (feedset.hour < 23) feedset.hour++;
      goto set_feeding_hrs;
    }
    if (!digitalRead(down)) {
      if (feedset.hour > 0) feedset.hour--;
      goto set_feeding_hrs;
    }
    if (!digitalRead(back)) goto set_feeding;
    if (!digitalRead(enter)) goto set_feeding_mins;
    if (millis() > endtime) goto UI_exit;
  }

set_feeding_mins:
  endtime = millis() + UI_DELAY;
  printFeeding(feedset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (feedset.mins < 59) feedset.mins++;
      goto set_feeding_mins;
    }
    if (!digitalRead(down)) {
      if (feedset.mins > 0) feedset.mins--;
      goto set_feeding_mins;
    }
    if (!digitalRead(back)) goto set_feeding_hrs;
    if (!digitalRead(enter)) goto set_feeding_choice;
    if (millis() > endtime) goto UI_exit;
  }

set_feeding_choice:
  endtime = millis() + UI_DELAY;
  printFeeding(feedset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (feedset.choice < 2) feedset.choice++;
      else feedset.choice = 0;
      goto set_feeding_choice;
    }
    if (!digitalRead(down)) {
      if (feedset.choice > 0) feedset.choice--;
      else feedset.choice = 2;
      goto set_feeding_choice;
    }
    if (!digitalRead(back)) goto set_feeding_mins;
    if (!digitalRead(enter)) goto set_feeding_noise;
    if (millis() > endtime) goto UI_exit;
  }

set_feeding_noise:
  endtime = millis() + UI_DELAY;
  printFeeding(feedset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      root.rewindDirectory();
      file = root.openNextFile();
      file = root.openNextFile();
      Serial.println(file.name());
      strcpy(feedset.noise, file.name());
      goto set_feeding_noise;
    }
    if (!digitalRead(down)) {
      file = root.openNextFile();
      Serial.println(file.name());
      //feedset.noise =
      strcpy(feedset.noise, file.name());
      goto set_feeding_noise;
    }
    if (!digitalRead(menu)) {
      speak.play(feedset.noise);
      goto set_feeding_noise;
    }
    if (!digitalRead(back)) goto set_feeding_choice;
    if (!digitalRead(enter)) {
      if (isDog) {
        if (isMorning) leftMorning = feedset;
        else leftEvening = feedset;
      }
      if (!isDog) {
        if (isMorning) rightMorning = feedset;
        else rightEvening = feedset;
      }
      updateEEPROM();
      if (isDog) goto main_left;
      else goto main_right;
    }
    if (millis() > endtime) goto UI_exit;
  }


Clock_Set:
  endtime = millis() + UI_DELAY;
  printDateTime(timeset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      timeset.year++;
      goto Clock_Set;
    }
    if (!digitalRead(down)) {
      timeset.year--;
      goto Clock_Set;
    }
    if (!digitalRead(back)) goto main_clock;
    if (!digitalRead(enter)) goto Clock_Month;
    if (millis() > endtime) goto UI_exit;
  }

Clock_Month:
  endtime = millis() + UI_DELAY;
  printDateTime(timeset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (timeset.month < 12) timeset.month++;
      goto Clock_Month;
    }
    if (!digitalRead(down)) {
      if (timeset.month > 1) timeset.month--;
      goto Clock_Month;
    }
    if (!digitalRead(back)) goto Clock_Set;
    if (!digitalRead(enter)) {
      switch (timeset.month) {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12: maxday = 31; break;
        case 4:
        case 6:
        case 9:
        case 11: maxday = 30; break;
        case 2:
          if (timeset.year % 400 == 0) maxday = 29;
          else if (timeset.year % 100 == 0) maxday = 28;
          else if (timeset.year % 4 == 0) maxday = 29;
          else maxday = 28;
          break;
      }
      goto Clock_Day;
    }
    if (millis() > endtime) goto UI_exit;
  }

Clock_Day:
  endtime = millis() + UI_DELAY;
  printDateTime(timeset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (timeset.day < maxday) timeset.day++;
      goto Clock_Day;
    }
    if (!digitalRead(down)) {
      if (timeset.day > 1) timeset.day--;
      goto Clock_Day;
    }
    if (!digitalRead(back)) goto Clock_Month;
    if (!digitalRead(enter)) goto Clock_hours;
    if (millis() > endtime) goto UI_exit;
  }

Clock_hours:
  endtime = millis() + UI_DELAY;
  printDateTime(timeset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (timeset.hour < 23) timeset.hour++;
      goto Clock_hours;
    }
    if (!digitalRead(down)) {
      if (timeset.hour > 0) timeset.hour--;
      goto Clock_hours;
    }
    if (!digitalRead(back)) goto Clock_Day;
    if (!digitalRead(enter)) goto Clock_minutes;
    if (millis() > endtime) goto UI_exit;
  }

Clock_minutes:
  endtime = millis() + UI_DELAY;
  printDateTime(timeset);
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up)) {
      if (timeset.minute < 59) timeset.minute++;
      goto Clock_minutes;
    }
    if (!digitalRead(down)) {
      if (timeset.minute > 0) timeset.minute--;
      goto Clock_minutes;
    }
    if (!digitalRead(back)) goto Clock_hours;
    if (!digitalRead(enter)) {
      RtcDateTime clockSet = RtcDateTime(timeset.year, timeset.month, timeset.day, timeset.hour, timeset.minute, 0);
      Clock.SetDateTime(clockSet);
      goto main_clock;
    }
    if (millis() > endtime) goto UI_exit;
  }


OptionA:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.print("Feed simultaneously?");
  lcd.setCursor(0, 1);
  lcd.write(9);
  lcd.print("Yes");
  lcd.setCursor(1, 2);
  lcd.print("No");
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto OptionB;
    if (!digitalRead(down)) goto OptionB;
    if (!digitalRead(back)) goto main_option;
    if (!digitalRead(enter)) {
      SetOption(true);
      goto main_option;
    }
    if (millis() > endtime) goto UI_exit;
  }

OptionB:
  endtime = millis() + UI_DELAY;
  lcd.clear();
  lcd.print("Feed simultaneously?");
  lcd.setCursor(1, 1);
  lcd.print("Yes");
  lcd.setCursor(0, 2);
  lcd.write(9);
  lcd.print("No");
  delay(buttondelay);
  while (true) {
    if (!digitalRead(up))goto OptionA;
    if (!digitalRead(down)) goto OptionA;
    if (!digitalRead(back)) goto main_option;
    if (!digitalRead(enter)) {
      SetOption(false);
      goto main_option;
    }
    if (millis() > endtime) goto UI_exit;
  }


UI_exit:
  userInterface = false;
  lcd.clear();
  return;
} // End User Interface*********************************************************************************************************************************

void SetOption(boolean toSet) {
  allowSimultaneousFeeding = toSet;
  EEPROM.update(4000, allowSimultaneousFeeding);
  updateRAM();
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

  lcd.begin (20, 4); // 20 x 4 LCD module
  lcd.setBacklightPin(3, POSITIVE); // BL, BL_POL
  lcd.setBacklight(HIGH);
  lcd.createChar(9, ARROW);
  lcd.clear();

  Clock.Begin();

  pinMode(SOUND, OUTPUT);
  digitalWrite(SOUND, LOW);
  speak.speakerPin = SOUND;
  speak.volume(7);

  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println("SD fail");
  }
  else Serial.println("SD Succeed");

  root = SD.open("/");
  for (int i = 0; i < 3; i++) {
    File file = root.openNextFile();
    Serial.println(file.name());
  }
  //feeding feedset = {1, 10, 30, "test2.wav\0", true, 0};
  //leftMorning = feedset;
  //leftEvening = feedset;
  //rightMorning = feedset;
  //rightEvening = feedset;
  //updateEEPROM();



  //printDirectory(root, 0);

  //  root.openRoot(volume);
  //  root.ls(LS_R | LS_DATE | LS_SIZE);





  updateRAM();


  // Test code below here in setup ***************************************************************************************************
  //  RtcDateTime CurrentTime = RtcDateTime(__DATE__,__TIME__);
  //  Clock.SetDateTime(CurrentTime);

  // print the contents of EEPROM  to serial output
  Serial.println(" contents of EEProm");

  for (int addr = 0; addr < 4096; addr = addr + 8) {
    Serial.print(addr);
    for (int a = 0; a < 8; a++) {
      Serial.print("\t");
      Serial.print(EEPROM.read(addr + a));
    }
    Serial.println();
  }

} // End of Setup******************************************************************************************************************

void loop() {
  if (userInterface) UI();
  updateTime();
  RFID();
  if (!speak.isPlaying()) digitalWrite(SOUND, LOW);




  //  Serial.println("playing");
  //  speak.play("test2.wav");
  //  while (speak.isPlaying()) {
  //    RFID();
  //  }
  //  if (!speak.isPlaying()) {
  //    digitalWrite(SOUND, LOW);
  //  }
  delay(2000);
  //RFID();


  //  RFID();
  //  delay(2000);

  //speak.play("a.wav");

  // Test code in loop below here *************************************************************************************************
}
