#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
PN532_I2C pn532_i2c(Wire);
int motorpin2=28;
NfcAdapter nfc = NfcAdapter(pn532_i2c);
String tagId1 = "EC E7 19 49";
String tagId2= "50 19 8F 1E";
String tagId = "None";
byte nuidPICC[4];
 
void setup(void) {
  Serial.begin(115200);
  pinMode(motorpin1,OUTPUT);
  pinMode(motorpin2,OUTPUT);
 
  nfc.begin();
   digitalWrite(motorpin1, LOW);
   digitalWrite(motorpin2, LOW);
}

void loop() {
  readNFC();
  if(tagId==tagId2)
  {
    Serial.println("Dog 2 Present");
    if(digitalRead(motorpin2) == 1)
    {
      digitalWrite(motorpin2, LOW);
      tagId = "";
      delay(1000);
    }

     else if(digitalRead(motorpin2) ==  0)
    {
      digitalWrite(motorpin2, HIGH);
      tagId = "";
      delay(1000);
    }

    //   if(digitalRead(motorpin2) == 1)
    // {
    //   digitalWrite(motorpin2, LOW);
    //   tagId = "";
    //   delay(1000);
    // }
    // return 1;
  }

  if(tagId==tagId1)
  {
    Serial.println("Dog 1 Present");
    if( digitalRead(motorpin2) == 1)
    {
      digitalWrite(motorpin2, LOW);
      tagId = "";
      delay(1000);
    }

    // else if(digitalRead(motorpin2) ==  0)
    // {
    //   digitalWrite(motorpin2, HIGH);
    //   tagId = "";
    //   delay(1000);

    // if( digitalRead(motorpin1) == 1)
    // {
    //   digitalWrite(motorpin1, LOW);
    //   tagId = "";
    //   delay(1000);
    // } 
    return 1;
  }
}

 void readNFC() {
  if (nfc.tagPresent())
  {
    NfcTag tag = nfc.read();
    tag.print();
    tagId = tag.getUidString();
    Serial.println("Tag ID:");
    Serial.println(tagId);
  }

  else
  {
    return 1;
  }

  delay(1000);
}