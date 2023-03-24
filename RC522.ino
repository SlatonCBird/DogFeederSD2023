#include <SPI.h>
#include <MFRC522.h>
int motorpin1=29;
#define SS_PIN 9
#define RST_PIN 8
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
 
void setup() 
{
  Serial.begin(9600);
  pinMode(motorpin1,OUTPUT);
  pinMode(motorpin2,OUTPUT);

  SPI.begin();
  mfrc522.PCD_Init();   // Initiate MFRC522
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, LOW);
}

void loop() 
{
  // Looks for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Selects one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }

  //Show UID on serial monitor
  Serial.print("Tag ID:");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  content.toUpperCase();

  if (content.substring(1) == "EC E7 19 49") //change here the UID of the card/cards that you want to give access
  {
    Serial.println("Dog 1 Present");
      if(digitalRead(motorpin1) == 1)
    {
      digitalWrite(motorpin1, LOW);
      delay(1000);
    }

     else if(digitalRead(motorpin1) ==  0)
    {
      digitalWrite(motorpin1, HIGH);
      delay(1000);
    }

    //   if(digitalRead(motorpin2) == 1)
    // {
    //   digitalWrite(motorpin2, LOW);
    //   delay(1000);
    // }
    
    delay(1000);
  }

 else if(content.substring(1) == "50 19 8F 1E")  {
   Serial.println("Dog 2 Present");

    if( digitalRead(motorpin1) == 1)
    {
      digitalWrite(motorpin1, LOW);
      delay(1000);
    } 

    delay(1000);
 }

  else{
    Serial.println(" Access denied");
  }
} 