

/**
 * Prototype software for Automated Multi-Dog Feeder
 * 
 */
 //Library Inclusions
 #include <Servo.h>
 #include "Wire.h" // For I2C
 #include "LCD.h" // For LCD
 #include "LiquidCrystal_I2C.h" // Added library*
 #include <SPI.h> // for SD card
 #include <SD.h> // for SD card
 #include <HX711.h> // for load cell ADC
 


 //pins
 #define SLScontrol 9 //control pin for SLS servo motor
 #define menu 3 // menu button with interrupt
 #define up 4 // up button
 #define down 5 // down button
 #define back 6 // back button
 #define enter 7 // enter button

 #define SCALE_LEFT_DOUT 23 //
 #define SCALE_LEFT_SCK 25 //
 #define SCALE_RIGHT_DOUT 22 //
 #define SCALE_RIGHT_SCK 24 //
 

 //values
 #define SLS_idle_position 90 // Idle position for SLS storage(degrees)
 #define SLS_left_fill 0 // fill position to left hopper(degrees)
 #define SLS_left_dump 68 // dump positon to left bowl(degrees)
 #define SLS_right_fill 180 // fill position to right hopper(degrees)
 #define SLS_right_dump 135 // dump positon to right bowl(degrees)
 #define SLS_fill_delay 2000 // delay for SLS to fill from hopper(mS)
 #define SLS_dump_delay 2000 // delay for SLS to dump into bowl(mS)

 const int chipSelect = 53;
 volatile boolean userInterface = false;

 // set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
 

 Servo SLS;
 HX711 leftScale;
 HX711 rightScale;

 //Set the pins on the I2C chip used for LCD connections
 //ADDR,EN,R/W,RS,D4,D5,D6,D7
 LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the default I2C address for LCD

// Right arrow
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


 //functions
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

 void ISR_UI(){
  userInterface = true;
 }

 void UI(){
  while(digitalRead(back) != LOW){
   lcd.clear();
   lcd.write(9);
   lcd.print("Set Clock");
   lcd.setCursor(1,1);
   lcd.print("Left Dog");
   lcd.setCursor(1,2);
   lcd.print("Right Dog");
   lcd.setCursor(1,3);
   lcd.print("Option");
   delay(1000);

   
   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print("Set Clock");
   lcd.setCursor(0,1);
   lcd.write(9);
   lcd.print("Left Dog");
   lcd.setCursor(1,2);
   lcd.print("Right Dog");
   lcd.setCursor(1,3);
   lcd.print("Option");
   delay(1000);

   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print("Set Clock");
   lcd.setCursor(1,1);
   lcd.print("Left Dog");
   lcd.setCursor(0,2);
   lcd.write(9);
   lcd.print("Right Dog");
   lcd.setCursor(1,3);
   lcd.print("Option");
   delay(1000);

   lcd.clear();
   lcd.setCursor(1,0);
   lcd.print("Set Clock");
   lcd.setCursor(1,1);
   lcd.print("Left Dog");
   lcd.setCursor(1,2);
   lcd.print("Right Dog");
   lcd.setCursor(0,3);
   lcd.write(9);
   lcd.print("Option");
   delay(1000);
 }
  userInterface = false;
  lcd.clear();
  return;
 }

void setup() {
  Serial.begin(9600);

  pinMode(menu, INPUT_PULLUP);
  attachInterrupt(1, ISR_UI, LOW);
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  pinMode(enter, INPUT_PULLUP);

  pinMode(36, OUTPUT);
  pinMode(38, OUTPUT);

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
  

 // Test code below here in setup ***************************************************************************************************
  // Set off LCD module

//   lcd.write(9);
//   lcd.print("Set Clock");
//   lcd.setCursor(1,1);
//   lcd.print("Left Dog");
//   lcd.setCursor(1,2);
//   lcd.print("Right Dog");
//   lcd.setCursor(1,3);
//   lcd.print("Option");
//   delay(1000);

   
//   lcd.clear();
//   lcd.setCursor(1,0);
//   lcd.print("Set Clock");
//   lcd.setCursor(0,1);
//   lcd.write(9);
//   lcd.print("Left Dog");
//   lcd.setCursor(1,2);
//   lcd.print("Right Dog");
//   lcd.setCursor(1,3);
//   lcd.print("Option");

//   lcd.clear();
//   lcd.setCursor(1,0);
//   lcd.print("Set Clock");
//   lcd.setCursor(1,1);
//   lcd.print("Left Dog");
//   lcd.setCursor(0,2);
//   lcd.write(9);
//   lcd.print("Right Dog");
//   lcd.setCursor(1,3);
//   lcd.print("Option");

//   lcd.clear();
//   lcd.setCursor(1,0);
//   lcd.print("Set Clock");
//   lcd.setCursor(1,1);
//   lcd.print("Left Dog");
//   lcd.setCursor(1,2);
//   lcd.print("Right Dog");
//   lcd.setCursor(0,3);
//   lcd.write(9);
//   lcd.print("Option");

  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);




} // End of Setup******************************************************************************************************************

void loop() {
  if(userInterface){
    UI();
  }

  Serial.println(rightScale.get_units());
  Serial.println(leftScale.get_units());
  delay(10);



  SLS.write(-50);
  delay(5000);
  SLS.write(360);
  delay(5000);
  SLS_Left(10);
  delay(5000);
  

   // Test code in loop below here *************************************************************************************************


   


}
